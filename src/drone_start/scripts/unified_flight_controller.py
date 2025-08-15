#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
unified_flight_controller.py

统一飞行控制器 - 支持仿真和现实模式的无人机控制接口
设计目标：与mission_orchestrator.py保持接口兼容，支持仿真/现实无缝切换

主要功能:
1. 仿真/现实模式自动切换
2. 统一的飞行控制接口（起飞、降落、位置控制、速度控制）
3. 云台控制接口
4. 基于视觉引导的精准降落
5. 状态获取和监控

"""

import math
import threading
import time
from typing import Optional, Tuple, Callable, Any
from dataclasses import dataclass
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

# 识别节点自定义消息
from vision_pkg.msg import QRCodeResult

# DJI PSDK 消息和服务
from tta_m3e_rtsp.msg import flightByVel, uavdata
from tta_m3e_rtsp.srv import takeoffOrLanding, takeoffOrLandingRequest

from flight_manager import FlightManager

# 云台控制相关导入
try:
    from tta_m3e_rtsp.srv import gimbalControl, gimbalControlRequest
    _GIMBAL_AVAILABLE_IMPORT = True
except Exception:
    _GIMBAL_AVAILABLE_IMPORT = False

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import QuaternionStamped, Vector3Stamped


@dataclass
class PIDController:
    """简单的PID控制器"""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    
    def __post_init__(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
    
    def update(self, error: float, current_time: float) -> float:
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return self.kp * error
        
        dt = current_time - self.prev_time
        if dt <= 0:
            return self.kp * error
        
        # 积分项
        self.integral += error * dt
        
        # 微分项
        derivative = (error - self.prev_error) / dt
        
        # PID输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        # 更新历史值
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None


class UnifiedFlightController:
    """统一飞行控制器"""
    
    def __init__(self, uav_ns="uav1"):
        """
        初始化统一飞行控制器
        
        Args:
            uav_ns: 无人机命名空间
        """
        self.uav_ns = uav_ns
        
        # 读取ROS参数
        self._load_parameters()
        
        # 初始化状态变量
        self._init_state_variables()
        
        # 检测仿真/现实模式
        self._detect_mode()
        
        # 初始化ROS接口
        self._init_ros_interfaces()
        
        # 初始化云台接口
        self._init_gimbal_service()
        
        # 启动状态监控线程
        self._start_monitoring_threads()
        
        rospy.loginfo(f"[UnifiedFlightController] Initialized in {'SIMULATION' if self.simulation_mode else 'REAL'} mode")
    
    def _load_parameters(self):
        """加载ROS参数"""
        # 基础参数
        self.simulation_mode = rospy.get_param("~simulation_mode", True)
        self.auto_detect_mode = rospy.get_param("~auto_detect_mode", True)
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.2)
        self.takeoff_timeout = rospy.get_param("~takeoff_timeout", 15.0)
        self.land_timeout = rospy.get_param("~land_timeout", 30.0)
        
        # 速度控制参数
        self.max_velocity = rospy.get_param("~max_velocity", 2.0)
        self.position_tolerance = rospy.get_param("~position_tolerance", 0.1)
        self.yaw_tolerance_deg = rospy.get_param("~yaw_tolerance_deg", 5.0)

         # 分步移动控制参数
        self.step_base_velocity = rospy.get_param("~step_base_velocity", 0.8)  # 基础速度 m/s
        self.step_max_velocity = rospy.get_param("~step_max_velocity", 2)    # 最大速度 m/s
        self.step_min_velocity = rospy.get_param("~step_min_velocity", 0.05)    # 最小速度 m/s
        self.step_distance_thresholds = rospy.get_param("~step_distance_thresholds", [0.02, 0.06, 0.1, 0.2, 0.5, 1.0, 2.0, 4.0])  # 距离阶梯
        self.step_velocity_scales = rospy.get_param("~step_velocity_scales",         [0.05, 0.12, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0])      # 对应速度比例
        self.step_stabilize_time = rospy.get_param("~step_stabilize_time", 0.5)  # 每步后稳定等待时间
        self.step_max_iterations = rospy.get_param("~step_max_iterations", 30)   # 最大迭代次数
        
        # 高级降落参数
        self.landing_frame_width = rospy.get_param("~landing_frame_width", 640)
        self.landing_frame_height = rospy.get_param("~landing_frame_height", 480)
        self.landing_tolerance_x = rospy.get_param("~landing_tolerance_x", 80)
        self.landing_tolerance_y = rospy.get_param("~landing_tolerance_y", 40)
        self.landing_max_speed = rospy.get_param("~landing_max_speed", 0.5)
        self.landing_min_speed = rospy.get_param("~landing_min_speed", 0.25)
        self.landing_default_speed = rospy.get_param("~landing_default_speed", 0.15)
        self.landing_final_offset = rospy.get_param("~landing_final_offset", 0.15)

        
        # PID参数（降落用）
        self.landing_pid_x_kp = rospy.get_param("~landing_pid_x_kp", 0.015)
        self.landing_pid_x_ki = rospy.get_param("~landing_pid_x_ki", 0.0005)
        self.landing_pid_x_kd = rospy.get_param("~landing_pid_x_kd", 0.0000)
        self.landing_pid_y_kp = rospy.get_param("~landing_pid_y_kp", 0.016)
        self.landing_pid_y_ki = rospy.get_param("~landing_pid_y_ki", 0.0005)
        self.landing_pid_y_kd = rospy.get_param("~landing_pid_y_kd", 0.0000)
        
        # 话题和服务名称
        self.flight_by_vel_topic = rospy.get_param("~flight_by_vel_topic", "/flightByVel")
        self.takeoff_land_service = rospy.get_param("~takeoff_land_service", "/takeoffOrLanding")
        
        # 现实模式下的起始坐标偏移
        self.real_start_offset_x = rospy.get_param("~real_start_offset_x", 1.0)
        self.real_start_offset_y = rospy.get_param("~real_start_offset_y", 2.3)
        self.real_start_offset_z = rospy.get_param("~real_start_offset_z", 0.15)

       
    def _init_state_variables(self):
        """初始化状态变量"""
        # 当前状态
        self.current_state = None
        self.current_odom = None
        self.current_uav_data = None
        
        # 位置和姿态
        self._current_pos = (0.0, 0.0, 0.0)
        self._current_yaw = 0.0
        self._is_armed = False
        self._current_mode = "UNKNOWN"
        
        # 控制状态
        self._target_pos = (0.0, 0.0, 0.0)
        self._target_yaw = 0.0
        self._control_active = False
        
        # 线程安全锁
        self._state_lock = threading.Lock()
        self._control_lock = threading.Lock()
        
        # 精准降落状态
        self._landing_active = False
        self._detection_data = None
        self._landing_pids = None

        # 精准降落稳定性检测
        self._in_center_start_time = None      # 进入中心的时间
        self._required_stable_time = 2.0       # 需要稳定的时间（秒）
        
        # 云台状态
        self._enable_gimbal = rospy.get_param("~enable_gimbal", True)
        self._gimbal_service_name = rospy.get_param("~gimbal_service", f"/{self.uav_ns}/gimbalControl")
        self._is_simulation = rospy.get_param("~simulation_mode", True)  # 默认仿真模式
        self._gimbal_srv = None
        self._gimbal_available = False
        self._gimbal_cached = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._gimbal_connection_retries = 0
        self._max_gimbal_retries = 3

        # 仿真降落用控制对象
        self.fm = FlightManager()

    def _detect_mode(self):
        """自动检测仿真/现实模式"""
        if not self.auto_detect_mode:
            rospy.loginfo(f"[UnifiedFlightController] Mode detection disabled, using: {'SIMULATION' if self.simulation_mode else 'REAL'}")
            return
        
        # 尝试检测mavros话题（仿真模式特征）
        try:
            mavros_topics = [f"/{self.uav_ns}/mavros/state", f"/{self.uav_ns}/mavros/local_position/odom"]
            uavdata_topic = f"/{self.uav_ns}/uavdata"
            
            # 等待话题出现
            rospy.loginfo("[UnifiedFlightController] Auto-detecting mode...")
            start_time = rospy.Time.now()
            timeout = 5.0
            
            mavros_detected = False
            uavdata_detected = False
            
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                published_topics = [topic[0] for topic in rospy.get_published_topics()]
                
                if any(topic in published_topics for topic in mavros_topics):
                    mavros_detected = True
                if uavdata_topic in published_topics:
                    uavdata_detected = True
                
                if mavros_detected or uavdata_detected:
                    break
                
                rospy.sleep(0.1)
            
            if mavros_detected and not uavdata_detected:
                self.simulation_mode = True
                rospy.loginfo("[UnifiedFlightController] Detected SIMULATION mode (mavros topics found)")
            elif uavdata_detected and not mavros_detected:
                self.simulation_mode = False
                rospy.loginfo("[UnifiedFlightController] Detected REAL mode (uavdata topic found)")
            elif mavros_detected and uavdata_detected:
                rospy.logwarn("[UnifiedFlightController] Both mavros and uavdata detected, using parameter setting")
            else:
                rospy.logwarn("[UnifiedFlightController] No specific topics detected, using parameter setting")
                
        except Exception as e:
            rospy.logwarn(f"[UnifiedFlightController] Mode detection failed: {e}, using parameter setting")
    
    def _init_ros_interfaces(self):
        """初始化ROS接口"""
        # 控制发布器
        self.flight_vel_pub = rospy.Publisher(self.flight_by_vel_topic, flightByVel, queue_size=10)
        
        # 服务客户端
        self.takeoff_land_client = rospy.ServiceProxy(self.takeoff_land_service, takeoffOrLanding)
        
        if self.simulation_mode:
            # 仿真模式：使用mavros接口
            self._init_simulation_interfaces()
        else:
            # 现实模式：使用uavdata接口
            self._init_real_interfaces()
    
    def _init_simulation_interfaces(self):
        """初始化仿真模式接口"""
        # 状态订阅
        self.state_sub = rospy.Subscriber(f"/{self.uav_ns}/mavros/state", State, self._mavros_state_cb)
        self.odom_sub = rospy.Subscriber(f"/{self.uav_ns}/mavros/local_position/odom", Odometry, self._mavros_odom_cb)
        
        # 位置控制发布器（用于position模式的goto）
        self.pos_setpoint_pub = rospy.Publisher(f"/{self.uav_ns}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # mavros服务
        self.arm_client = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/cmd/arming", CommandBool)
        self.mode_client = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/set_mode", SetMode)
        
        rospy.loginfo("[UnifiedFlightController] Simulation interfaces initialized")
    
    def _init_real_interfaces(self):
        """初始化现实模式接口"""
        # 状态订阅
        self.uavdata_sub = rospy.Subscriber(f"/{self.uav_ns}/uavdata", uavdata, self._uavdata_cb)
        
        rospy.loginfo("[UnifiedFlightController] Real mode interfaces initialized")
    
    def _init_gimbal_service(self):
        """初始化云台服务连接"""
        if not self._enable_gimbal:
            rospy.loginfo("[FlightManager] Gimbal disabled by parameter.")
            return

        if not _GIMBAL_AVAILABLE_IMPORT:
            rospy.logwarn("[FlightManager] Gimbal service messages not imported; disabling gimbal.")
            self._enable_gimbal = False
            return

        # 在仿真模式下，给云台控制器更多时间启动
        gimbal_wait_timeout = rospy.get_param("~gimbal_wait_timeout", 5.0 if self._is_simulation else 3.0)
        
        rospy.loginfo("[FlightManager] Waiting for gimbal service %s (timeout=%.1fs, simulation=%s)...",
                      self._gimbal_service_name, gimbal_wait_timeout, self._is_simulation)

        # 尝试连接云台服务
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < gimbal_wait_timeout:
            try:
                rospy.wait_for_service(self._gimbal_service_name, timeout=0.5)
                self._gimbal_srv = rospy.ServiceProxy(self._gimbal_service_name, gimbalControl)
                self._gimbal_available = True
                rospy.loginfo("[FlightManager] Gimbal service connected successfully.")
                
                # 在仿真模式下，测试服务调用
                if self._is_simulation:
                    self._test_gimbal_service()
                break
                
            except Exception as e:
                rospy.logdebug("[FlightManager] Gimbal service wait attempt failed: %s", e)
                
        if not self._gimbal_available:
            if self._is_simulation:
                rospy.logwarn("[FlightManager] Gimbal service not available in simulation. "
                             "请确保已启动: roslaunch drone_start gimbal_simulation.launch")
            else:
                rospy.logwarn("[FlightManager] Gimbal service not available on real platform.")

    def _test_gimbal_service(self):
        """测试云台服务是否正常工作（仅仿真模式）"""
        try:
            # 发送一个测试命令（回到中性位置）
            req = gimbalControlRequest()
            req.pitch = 0.0
            req.roll = 0.0  
            req.yaw = 0.0
            
            resp = self._gimbal_srv.call(req)
            rospy.loginfo("[FlightManager] Gimbal service test successful.")
            self._gimbal_last = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
            
        except Exception as e:
            rospy.logwarn("[FlightManager] Gimbal service test failed: %s", e)
            self._gimbal_available = False

    def reconnect_gimbal_service(self) -> bool:
        """重新连接云台服务（用于运行时恢复）"""
        if self._gimbal_connection_retries >= self._max_gimbal_retries:
            rospy.logwarn("[FlightManager] Max gimbal reconnection attempts reached.")
            return False
            
        self._gimbal_connection_retries += 1
        rospy.loginfo("[FlightManager] Attempting gimbal reconnection (%d/%d)...", 
                     self._gimbal_connection_retries, self._max_gimbal_retries)
        
        self._gimbal_available = False
        self._init_gimbal_service()
        
        return self._gimbal_available
    
    def _start_monitoring_threads(self):
        """启动监控线程"""
        # 控制循环线程
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        
        # 等待初始状态
        self._wait_for_initial_state()
    
    def _wait_for_initial_state(self):
        """等待初始状态数据"""
        rospy.loginfo("[UnifiedFlightController] Waiting for initial state...")
        start_time = rospy.Time.now()
        timeout = 10.0
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.simulation_mode and self.current_odom is not None:
                break
            elif not self.simulation_mode and self.current_uav_data is not None:
                break
            rospy.sleep(0.1)
        
        if self.simulation_mode and self.current_odom is None:
            rospy.logwarn("[UnifiedFlightController] Timeout waiting for odometry data")
        elif not self.simulation_mode and self.current_uav_data is None:
            rospy.logwarn("[UnifiedFlightController] Timeout waiting for uavdata")
        else:
            rospy.loginfo("[UnifiedFlightController] Initial state received")
    
    # ==================== 回调函数 ====================
    
    def _mavros_state_cb(self, msg: State):
        """Mavros状态回调"""
        with self._state_lock:
            self.current_state = msg
            self._is_armed = msg.armed
            self._current_mode = msg.mode
    
    def _mavros_odom_cb(self, msg: Odometry):
        """Mavros里程计回调"""
        with self._state_lock:
            self.current_odom = msg
            # 更新当前位置
            pos = msg.pose.pose.position
            self._current_pos = (pos.x, pos.y, pos.z)
            
            # 更新当前偏航角
            quat = msg.pose.pose.orientation
            self._current_yaw = self._yaw_from_quat(quat.x, quat.y, quat.z, quat.w)
    
    def _uavdata_cb(self, msg: uavdata):
        """UAV数据回调（现实模式）"""
        with self._state_lock:
            self.current_uav_data = msg
            
            # 注意：现实模式下需要坐标修正
            # uavdata提供相对起始点坐标，需要加上起始偏移
            raw_x = getattr(msg, 'pos_x', 0.0) if hasattr(msg, 'pos_x') else 0.0
            raw_y = getattr(msg, 'pos_y', 0.0) if hasattr(msg, 'pos_y') else 0.0  
            raw_z = getattr(msg, 'pos_z', 0.0) if hasattr(msg, 'pos_z') else msg.altit
            
            # 坐标修正
            corrected_x = raw_x + self.real_start_offset_x
            corrected_y = raw_y + self.real_start_offset_y
            corrected_z = raw_z + self.real_start_offset_z
            
            self._current_pos = (corrected_x, corrected_y, corrected_z)
            self._current_yaw = math.degrees(msg.atti_yaw)
    
    def _gimbal_status_cb(self, msg: Vector3Stamped):
        """云台状态回调"""
        self._gimbal_cached = {
            'pitch': msg.vector.x,
            'yaw': msg.vector.y, 
            'roll': msg.vector.z
        }
    
    # ==================== 工具函数 ====================
    
    def _yaw_from_quat(self, x, y, z, w) -> float:
        """从四元数计算偏航角（度）"""
        # 使用tf转换
        euler = tf.transformations.euler_from_quaternion([x, y, z, w])
        return math.degrees(euler[2])  # yaw是第三个元素
    
    def _distance_xy(self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]) -> float:
        """计算两点间XY平面距离"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def _error_yaw_deg(self, target_yaw: float, current_yaw: float) -> float:
        """计算偏航角误差（-180到180度）"""
        error = target_yaw - current_yaw
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error
    
    def _control_loop(self):
        """控制循环线程"""
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            try:
                # 这里可以添加持续的控制逻辑
                # 目前主要用于精准降落等需要持续控制的功能
                if self._landing_active:
                    self._precision_landing_step()
                    
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logwarn(f"[UnifiedFlightController] Control loop error: {e}")
                rate.sleep()
    
    # ==================== 公共接口 ====================
    
    @property
    def current_position(self) -> Tuple[float, float, float]:
        """获取当前位置"""
        with self._state_lock:
            return self._current_pos
    
    @property
    def current_yaw_deg(self) -> float:
        """获取当前偏航角（度）"""
        with self._state_lock:
            return self._current_yaw
    
    @property
    def is_armed(self) -> bool:
        """获取解锁状态"""
        with self._state_lock:
            return self._is_armed
    
    @property
    def current_mode(self) -> str:
        """获取当前飞行模式"""
        with self._state_lock:
            return self._current_mode
    
    @property
    def is_simulation_mode(self) -> bool:
        """是否为仿真模式"""
        return self.simulation_mode
    
    # ==================== 基础控制接口 ====================
    
    def arm(self) -> bool:
        """解锁"""
        if self.simulation_mode:
            try:
                response = self.arm_client(True)
                if response.success:
                    rospy.loginfo("[UnifiedFlightController] Armed successfully")
                    return True
                else:
                    rospy.logerr("[UnifiedFlightController] Arm failed")
                    return False
            except Exception as e:
                rospy.logerr(f"[UnifiedFlightController] Arm service call failed: {e}")
                return False
        else:
            rospy.loginfo("[UnifiedFlightController] Arm not needed in real mode")
            return True
    
    def disarm(self) -> bool:
        """上锁"""
        if self.simulation_mode:
            try:
                response = self.arm_client(False)
                if response.success:
                    rospy.loginfo("[UnifiedFlightController] Disarmed successfully")
                    return True
                else:
                    rospy.logerr("[UnifiedFlightController] Disarm failed")
                    return False
            except Exception as e:
                rospy.logerr(f"[UnifiedFlightController] Disarm service call failed: {e}")
                return False
        else:
            rospy.loginfo("[UnifiedFlightController] Disarm not needed in real mode")
            return True
    
    def takeoff(self, target_height: float = None, timeout: float = None) -> bool:
        """
        起飞
        
        Args:
            target_height: 目标高度（米），默认使用参数设置
            timeout: 超时时间（秒），默认使用参数设置
        
        Returns:
            bool: 起飞是否成功
        """
        if target_height is None:
            target_height = self.takeoff_height
        if timeout is None:
            timeout = self.takeoff_timeout
        
        rospy.loginfo(f"[UnifiedFlightController] Takeoff to {target_height}m (timeout: {timeout}s)")
        
        try:
            # 调用起飞服务
            req = takeoffOrLandingRequest()
            req.takeoffOrLanding = 1  # 1表示起飞
            
            response = self.takeoff_land_client(req)
            
            if response.ack == 1:
                rospy.loginfo("[UnifiedFlightController] Takeoff command sent successfully")
                # 输出当前高度
                rospy.loginfo(f"[UnifiedFlightController] Takeoff completed at {self.current_position[2]:.2f}m")
                return True
                # # 等待起飞完成
                # start_time = rospy.Time.now()
                # while (rospy.Time.now() - start_time).to_sec() < timeout:
                #     current_z = self.current_position[2]
                #     if current_z >= target_height - 0.1:  # 允许10cm误差
                #         rospy.loginfo(f"[UnifiedFlightController] Takeoff completed at {current_z:.2f}m")
                #         return True
                #     rospy.sleep(0.1)
                
                # rospy.logwarn("[UnifiedFlightController] Takeoff timeout")
                # return False
            else:
                rospy.logerr("[UnifiedFlightController] Takeoff command failed")
                return False
                
        except Exception as e:
            rospy.logerr(f"[UnifiedFlightController] Takeoff failed: {e}")
            return False

    def land(self, timeout: float = None, final_z: float = 0.0) -> bool:
        """
        降落 使用服务降落
        
        Args:
            timeout: 超时时间（秒），默认使用参数设置
        
        Returns:
            bool: 降落是否成功
        """
        if timeout is None:
            timeout = self.land_timeout
        
        rospy.loginfo(f"[UnifiedFlightController] Landing (timeout: {timeout}s)")
        if self.simulation_mode:
            x0, y0, current_z = self._current_pos
            yaw0 = self._current_yaw
            
            rospy.loginfo("[UnifiedFlightController] 开始智能降落: 从 %.2fm 到 %.2fm", current_z, final_z)
            
            # 计算下降阶段
            t0 = rospy.Time.now()
            
            while not rospy.is_shutdown() and current_z > final_z + 0.05:
                x0, y0, current_z = self._current_pos
                yaw0 = self._current_yaw

                 # 如果当前高度已经很低，直接检查着陆条件
                if current_z <= final_z + 0.05:
                    rospy.loginfo("[UnifiedFlightController] 已接近目标高度，检查着陆条件...")
                    break
            
                 # 动态调整下降米数 
                descent_step = max(0.03, (current_z - final_z) / 2)

                # 计算下一个目标高度
                next_target_z = max(current_z - descent_step, final_z)
                
                rospy.loginfo("[UnifiedFlightController] 下降阶段: 目标高度 %.2fm", next_target_z)
                self.goto(x0, y0, next_target_z, yaw0)
                
                # 等待到达当前阶段目标
                stage_timeout = 12.0  # 每个阶段最多等待12秒
                stage_start = rospy.Time.now()
                
                while not rospy.is_shutdown():
                    current_pos_z = self.current_position[2]
                    
                    # 检查是否到达当前阶段目标
                    if abs(current_pos_z - next_target_z) < 0.05:
                        rospy.loginfo("[UnifiedFlightController] 到达阶段目标: %.2fm", next_target_z)
                        break
                    
                    # 检查阶段超时
                    if (rospy.Time.now() - stage_start).to_sec() > stage_timeout:
                        rospy.logwarn("[UnifiedFlightController] 下降阶段超时，继续下一阶段")
                        break
                    
                    # 检查总超时
                    if (rospy.Time.now() - t0).to_sec() > timeout:
                        rospy.logerr("[UnifiedFlightController] 降落总超时")
                        return True
                    
                    rospy.sleep(0.1)
            
            # 最终着陆检查
            rospy.loginfo("[UnifiedFlightController] 着陆完成")
            return True

        try:
            # 调用降落服务
            req = takeoffOrLandingRequest()
            req.takeoffOrLanding = 2  # 2表示降落
            
            response = self.takeoff_land_client(req)
            
            if response.ack == 1:
                rospy.loginfo("[UnifiedFlightController] Landing command sent successfully")
                rospy.loginfo(f"[UnifiedFlightController] Landing completed at {self.current_position[2]:.2f}m")
            else:
                rospy.logerr("[UnifiedFlightController] Landing command failed")
                return False
                
        except Exception as e:
            rospy.logerr(f"[UnifiedFlightController] Landing failed: {e}")
            return False
 
    def fly_by_velocity(self, vel_n: float, vel_e: float, vel_d: float,
                       target_yaw: float, fly_time: float) -> bool:
        """
        速度控制飞行
        
        Args:
            vel_n: 北向速度（m/s）
            vel_e: 东向速度（m/s）
            vel_d: 向下速度（m/s，正值表示向下）
            target_yaw: 目标偏航角（度）
            fly_time: 飞行时间（秒）
        
        Returns:
            bool: 命令是否成功发送
        """
        if fly_time <= 0:
            rospy.logwarn("[UnifiedFlightController] Invalid fly_time")
            return False
        if self.simulation_mode:  # 在仿真模式下放开速度限制
            max_vel = self.max_velocity * 20  # 放大速度限制
        else:
            max_vel = self.max_velocity  # 现实模式下使用正常速度限制

        # 限制速度
        vel_n = max(-max_vel, min(max_vel, vel_n))
        vel_e = max(-max_vel, min(max_vel, vel_e))
        vel_d = max(-max_vel, min(max_vel, vel_d))

        # rospy.loginfo(f"[UnifiedFlightController] Velocity command: N={vel_n:.2f} E={vel_e:.2f} D={vel_d:.2f} yaw={target_yaw:.1f}° time={fly_time:.2f}s")
        
        try:
            # 发布速度命令
            vel_msg = flightByVel()
            vel_msg.vel_n = vel_n
            vel_msg.vel_e = vel_e
            vel_msg.vel_d = vel_d
            vel_msg.targetYaw = target_yaw
            vel_msg.fly_time = fly_time
            
            self.flight_vel_pub.publish(vel_msg)
            return True
            
        except Exception as e:
            rospy.logerr(f"[UnifiedFlightController] Velocity command failed: {e}")
            return False
    
    # def goto(self, x: float, y: float, z: float, yaw: float = None, 
    #          timeout: float = 30.0, tolerance: float = None) -> bool:
    #     """
    #     位置控制 - 飞到指定坐标
        
    #     Args:
    #         x, y, z: 目标位置坐标（米）
    #         yaw: 目标偏航角（度），None表示保持当前角度
    #         timeout: 超时时间（秒）
    #         tolerance: 位置容忍度（米），默认使用参数设置
        
    #     Returns:
    #         bool: 是否成功到达目标位置
    #     """
    #     if tolerance is None:
    #         tolerance = self.position_tolerance
        
    #     if yaw is None:
    #         yaw = self.current_yaw_deg
        
    #     current_pos = self.current_position
    #     distance = self._distance_xy(current_pos, (x, y, z))
        
    #     rospy.loginfo(f"[UnifiedFlightController] Goto: ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw:.1f}° distance={distance:.2f}m")
        
    #     # 使用速度控制实现位置控制
    #     start_time = rospy.Time.now()
    #     control_rate = rospy.Rate(20)  # 20Hz控制频率

    #     while (rospy.Time.now() - start_time).to_sec() < timeout:
    #         current_pos = self.current_position
    #         current_yaw = self.current_yaw_deg
            
    #         # 计算位置误差
    #         error_x = x - current_pos[0]
    #         error_y = y - current_pos[1]
    #         error_z = z - current_pos[2]
    #         error_yaw = self._error_yaw_deg(yaw, current_yaw)
            
    #         # 检查是否到达目标
    #         distance_xy = math.sqrt(error_x**2 + error_y**2)
    #         if (distance_xy < tolerance and 
    #             abs(error_z) < tolerance and 
    #             abs(error_yaw) < self.yaw_tolerance_deg):
    #             rospy.loginfo(f"[UnifiedFlightController] Reached target position")
    #             # 延迟等待机身稳定
    #             rospy.sleep(1.0)
    #             return True
            
    #         # 计算速度命令（简单比例控制）
    #         if self.simulation_mode:
    #             kp = 10  # 比例增益
    #             max_vel = self.max_velocity
    #             max_vel *= 50
    #         else:
    #             kp = 0.5  # 比例增益
    #             max_vel = self.max_velocity * 0.8  # 位置控制使用较低的最大速度
    #         vel_n = kp * error_x
    #         vel_e = kp * error_y
    #         vel_d = -kp * error_z  # 注意符号，vel_d正值表示向下
            
    #         # 限制速度
    #         vel_n = max(-max_vel, min(max_vel, vel_n))
    #         vel_e = max(-max_vel, min(max_vel, vel_e))
    #         vel_d = max(-max_vel, min(max_vel, vel_d))
            
    #         # 发送速度命令（短时间）
    #         self.fly_by_velocity(vel_n, vel_e, vel_d, yaw, 0.05)
            
    #         control_rate.sleep()
        
    #     rospy.logwarn("[UnifiedFlightController] Goto timeout")
    #     return False


    def goto(self, x: float, y: float, z: float, yaw: float = None, 
         timeout: float = 60.0, tolerance: float = None,
         base_velocity: float = None, max_velocity: float = None, min_velocity: float = None,
         stabilize_time: float = None, max_iterations: int = None) -> bool:
        """
        位置控制 - 分步移动到指定坐标
        
        Args:
            x, y, z: 目标位置坐标（米）
            yaw: 目标偏航角（度），None表示保持当前角度
            timeout: 超时时间（秒）
            tolerance: 位置容忍度（米），默认使用参数设置
            base_velocity: 基础速度（m/s），None使用参数设置
            max_velocity: 最大速度（m/s），None使用参数设置  
            min_velocity: 最小速度（m/s），None使用参数设置
            stabilize_time: 每步稳定时间（秒），None使用参数设置
            max_iterations: 最大迭代次数，None使用参数设置
        
        Returns:
            bool: 是否成功到达目标位置
        """
        if tolerance is None:
            tolerance = self.position_tolerance
        if yaw is None:
            yaw = self.current_yaw_deg
        if base_velocity is None:
            base_velocity = self.step_base_velocity
        if max_velocity is None:
            max_velocity = self.step_max_velocity
        if min_velocity is None:
            min_velocity = self.step_min_velocity
        if stabilize_time is None:
            stabilize_time = self.step_stabilize_time
        if max_iterations is None:
            max_iterations = self.step_max_iterations
        
        current_pos = self.current_position
        total_distance = math.sqrt((x - current_pos[0])**2 + (y - current_pos[1])**2 + (z - current_pos[2])**2)
        
        rospy.loginfo(f"[UnifiedFlightController] Goto (step mode): ({x:.2f}, {y:.2f}, {z:.2f}) yaw={yaw:.1f}°")
        rospy.loginfo(f"[UnifiedFlightController] Total distance: {total_distance:.2f}m, tolerance: {tolerance:.2f}m")
        
        start_time = rospy.Time.now()
        iteration = 0
        
        while iteration < max_iterations and (rospy.Time.now() - start_time).to_sec() < timeout:
            iteration += 1
            
            # 获取当前位置
            current_pos = self.current_position
            current_yaw = self.current_yaw_deg
            
            # 计算位置误差
            error_x = x - current_pos[0]
            error_y = y - current_pos[1]
            error_z = z - current_pos[2]
            error_yaw = self._error_yaw_deg(yaw, current_yaw)
            
            # 计算距离
            distance_xy = math.sqrt(error_x**2 + error_y**2)
            distance_total = math.sqrt(error_x**2 + error_y**2 + error_z**2)
            
            rospy.loginfo(f"[UnifiedFlightController] Step {iteration}: distance={distance_total:.3f}m "
                        f"XY={distance_xy:.3f}m Z={abs(error_z):.3f}m Yaw={abs(error_yaw):.1f}°")
            
            # 检查是否到达目标
            if (distance_xy < tolerance and 
                abs(error_z) < 0.03 and 
                abs(error_yaw) < self.yaw_tolerance_deg):
                rospy.loginfo(f"[UnifiedFlightController] Target reached in {iteration} steps!")
                rospy.loginfo(f"[UnifiedFlightController] Final position: {current_pos}, Yaw: {current_yaw:.1f}°")
                
                # 发送停止命令确保稳定
                self.fly_by_velocity(0.0, 0.0, 0.0, yaw, 0.5)
                rospy.sleep(0.3)
                return True
            
            # 根据距离计算速度
            velocity = self._calculate_step_velocity(distance_total, base_velocity, max_velocity, min_velocity)
            
            # 计算飞行时间
            if velocity > 0:
                flight_time = min(distance_total / velocity, 5.0)  # 最大单次飞行时间5秒
                flight_time = max(flight_time, 0.2)  # 最小飞行时间0.2秒
            else:
                flight_time = 1.0
            
            # 计算单位方向向量
            if distance_total > 0.001:  # 避免除零
                unit_x = error_x / distance_total
                unit_y = error_y / distance_total
                unit_z = error_z / distance_total
            else:
                unit_x = unit_y = unit_z = 0.0
            
            # 计算速度分量（NED坐标系）
            vel_n = velocity * unit_x  # 北向（X）
            vel_e = velocity * unit_y  # 东向（Y）
            vel_d = -velocity * unit_z  # 下向（Z的负方向）
            
            rospy.loginfo(f"[UnifiedFlightController] Step {iteration}: velocity={velocity:.2f}m/s time={flight_time:.1f}s")
            rospy.loginfo(f"[UnifiedFlightController] Velocity components: N={vel_n:.2f} E={vel_e:.2f} D={vel_d:.2f}")
            
            # 发送速度命令
            if flight_time < 1:
                success = self.fly_by_velocity(vel_n / flight_time / 2, vel_e / flight_time / 2, vel_d / flight_time / 2, yaw, flight_time)
            else:
                success = self.fly_by_velocity(vel_n, vel_e, vel_d, yaw, flight_time)
            if not success:
                rospy.logwarn(f"[UnifiedFlightController] Failed to send velocity command in step {iteration}")
                continue
            
            # 等待飞行完成
            rospy.sleep(flight_time)
            
            # 稳定等待时间
            rospy.loginfo(f"[UnifiedFlightController] Stabilizing for {stabilize_time:.1f}s...")
            rospy.sleep(stabilize_time)
        
        # 超时或达到最大迭代次数
        final_pos = self.current_position
        final_distance = math.sqrt((x - final_pos[0])**2 + (y - final_pos[1])**2 + (z - final_pos[2])**2)
        
        if iteration >= max_iterations:
            rospy.logwarn(f"[UnifiedFlightController] Reached max iterations ({max_iterations})")
        else:
            rospy.logwarn(f"[UnifiedFlightController] Timeout after {iteration} steps")
        
        rospy.logwarn(f"[UnifiedFlightController] Final distance to target: {final_distance:.3f}m")
        rospy.logwarn(f"[UnifiedFlightController] Final position: {final_pos}, target: ({x}, {y}, {z})")
        
        # 发送停止命令
        self.fly_by_velocity(0.0, 0.0, 0.0, yaw, 0.5)
        
        return True if final_distance < tolerance else False

    def _calculate_step_velocity(self, distance: float, base_velocity: float, 
                            max_velocity: float, min_velocity: float) -> float:
        """
        根据距离计算步进速度
        
        Args:
            distance: 目标距离（米）
            base_velocity: 基础速度（m/s）
            max_velocity: 最大速度（m/s）
            min_velocity: 最小速度（m/s）
        
        Returns:
            float: 计算出的速度（m/s）
        """
        # 使用阶梯式速度计算
        thresholds = self.step_distance_thresholds
        scales = self.step_velocity_scales
        
        # 找到对应的速度比例
        velocity_scale = scales[-1]  # 默认使用最后一个比例
        for i, threshold in enumerate(thresholds):
            if distance <= threshold:
                velocity_scale = scales[i]
                break
        
        # 计算速度
        velocity = base_velocity * velocity_scale
        
        # 限制在最小最大速度范围内
        velocity = max(min_velocity, min(velocity, max_velocity))
        
        rospy.logdebug(f"[UnifiedFlightController] Distance={distance:.2f}m -> scale={velocity_scale:.1f} -> velocity={velocity:.2f}m/s")
        
        return velocity
    
    def hold(self, duration: float = 1.0):
        """
        悬停
        
        Args:
            duration: 悬停时间（秒）
        """
        rospy.loginfo(f"[UnifiedFlightController] Holding for {duration:.1f}s")
        current_pos = self.current_position
        current_yaw = self.current_yaw_deg
        
        # 发送零速度命令
        self.fly_by_velocity(0.0, 0.0, 0.0, current_yaw, duration)
        rospy.sleep(duration)
    
    def abort(self):
        """紧急停止 - 悬停"""
        rospy.logwarn("[UnifiedFlightController] ABORT - Emergency hover")
        current_yaw = self.current_yaw_deg
        
        # 立即发送零速度命令
        self.fly_by_velocity(0.0, 0.0, 0.0, current_yaw, 1.0)
        
        # 停止精准降落
        self._landing_active = False
    
    # ==================== 云台控制接口 ====================
    
    def set_gimbal(self,
                   pitch: float = None,
                   roll: float = None,
                   yaw: float = None,
                   relative: bool = False,
                   timeout: float = 2.0) -> bool:
        """
        设置云台姿态 (角度单位: 度)

        Args:
            pitch / roll / yaw: 期望角度 (绝对或相对, 取决于 relative)
            relative: True 时表示在当前缓存值基础上增量修改
            timeout: 服务调用超时时间

        行为:
            - 若仿真中没有云台服务, 仅更新内部缓存并输出日志 (不会报错, 方便顶层统一调用)
            - 真实平台上调用 gimbalControl 服务
        """
        # 如果全部是 None, 不做任何操作
        if pitch is None and roll is None and yaw is None:
            rospy.logwarn_throttle(2.0, "[FlightManager] set_gimbal called with all None; skip.")
            return False

        # 更新目标值 (绝对或相对)
        tgt = dict(self._gimbal_last)  # 基于上一次
        if relative:
            if pitch is not None: tgt["pitch"] += pitch
            if roll is not None:  tgt["roll"] += roll
            if yaw is not None:   tgt["yaw"] += yaw
        else:
            if pitch is not None: tgt["pitch"] = pitch
            if roll is not None:  tgt["roll"] = roll
            if yaw is not None:   tgt["yaw"] = yaw

        # 角度范围裁剪 (示例, 可根据实际云台限制调整)
        def clamp(v, lo, hi): return max(lo, min(hi, v))
        tgt["pitch"] = clamp(tgt["pitch"], -90.0, 30.0)   # 俯仰: 向下为负
        tgt["roll"]  = clamp(tgt["roll"], -45.0, 45.0)
        tgt["yaw"]   = clamp(tgt["yaw"], -180.0, 180.0)

        if not self._enable_gimbal:
            rospy.loginfo_throttle(5.0, "[FlightManager] Gimbal disabled. (Sim update only) tgt=%s", tgt)
            self._gimbal_last = tgt
            return True

        if not self._gimbal_available:
            rospy.logwarn_throttle(5.0, "[FlightManager] Gimbal service unavailable (simulate only). tgt=%s", tgt)
            self._gimbal_last = tgt
            return True

        # 调用服务
        try:
            req = gimbalControlRequest()
            req.pitch = tgt["pitch"]
            req.roll = tgt["roll"]
            req.yaw = tgt["yaw"]
            # 若服务有其它字段(如 mode / speed), 可在此扩展: req.mode = ...
            
            start = rospy.Time.now()
            resp = self._gimbal_srv.call(req)
            dt = (rospy.Time.now() - start).to_sec()
            
            # 检查响应 (如果服务定义有success字段)
            try:
                if hasattr(resp, 'success') and not resp.success:
                    rospy.logwarn("[FlightManager] Gimbal service returned success=False")
                    if hasattr(resp, 'message'):
                        rospy.logwarn("[FlightManager] Gimbal error message: %s", resp.message)
            except AttributeError:
                pass  # 服务可能没有这些字段
            
            rospy.loginfo("[FlightManager] Gimbal set (p=%.1f r=%.1f y=%.1f) in %.3fs",
                          req.pitch, req.roll, req.yaw, dt)
            self._gimbal_last = tgt
            self._gimbal_connection_retries = 0  # 重置重连计数
            return True
            
        except rospy.ServiceException as e:
            rospy.logerr("[FlightManager] Gimbal service call failed: %s", e)
            
            # 在仿真模式下尝试重连
            if self._is_simulation and self.reconnect_gimbal_service():
                rospy.loginfo("[FlightManager] Gimbal reconnected, retrying command...")
                try:
                    req = gimbalControlRequest()
                    req.pitch = tgt["pitch"]
                    req.roll = tgt["roll"]
                    req.yaw = tgt["yaw"]
                    resp = self._gimbal_srv.call(req)
                    self._gimbal_last = tgt
                    rospy.loginfo("[FlightManager] Gimbal retry successful")
                    return True
                except Exception as retry_e:
                    rospy.logerr("[FlightManager] Gimbal retry failed: %s", retry_e)
            
            # 如果重连失败或不在仿真模式，仅更新缓存
            rospy.logwarn("[FlightManager] Falling back to cache-only mode for gimbal")
            self._gimbal_last = tgt
            return False
            
        except Exception as e:
            rospy.logerr("[FlightManager] Gimbal set exception: %s", e)
            # 更新缓存以保持状态一致性
            self._gimbal_last = tgt
            return False

    def get_gimbal_cached(self) -> dict:
        """获取云台当前角度（缓存值）"""
        return self._gimbal_cached.copy()
    
    @property
    def gimbal_available(self) -> bool:
        """云台是否可用"""
        return self._gimbal_available
    
    # ==================== 高级精准降落 ====================
    
    def precision_landing(self, detection_result_topic: str, start_detection_topic: str,
                         target_height: float = 0.0,
                         frame_width: int = None, frame_height: int = None,
                         tolerance_x: int = None, tolerance_y: int = None,
                         stable_time: float = 2.0,  # 默认稳定2秒,
                         max_speed: float = None, min_speed: float = None,
                         pid_x_kp: float = None, pid_x_ki: float = None, pid_x_kd: float = None,
                         pid_y_kp: float = None, pid_y_ki: float = None, pid_y_kd: float = None,
                         timeout: float = 120.0) -> bool:
        """
        基于视觉引导的精准降落
        
        Args:
            detection_result_topic: 识别结果话题名
            start_detection_topic: 开始识别话题名
            target_height: 目标降落高度（米）
            frame_width, frame_height: 画幅尺寸（像素）
            tolerance_x, tolerance_y: 容忍度（像素）
            max_speed, min_speed: 降落速度范围（m/s）
            pid_x_kp, pid_x_ki, pid_x_kd: X轴PID参数
            pid_y_kp, pid_y_ki, pid_y_kd: Y轴PID参数
            timeout: 超时时间（秒）
        
        Returns:
            bool: 降落是否成功
        """
        # 使用默认参数
        if frame_width is None: frame_width = self.landing_frame_width
        if frame_height is None: frame_height = self.landing_frame_height
        if tolerance_x is None: tolerance_x = self.landing_tolerance_x
        if tolerance_y is None: tolerance_y = self.landing_tolerance_y
        if max_speed is None: max_speed = self.landing_max_speed
        if min_speed is None: min_speed = self.landing_min_speed
        if pid_x_kp is None: pid_x_kp = self.landing_pid_x_kp
        if pid_x_ki is None: pid_x_ki = self.landing_pid_x_ki
        if pid_x_kd is None: pid_x_kd = self.landing_pid_x_kd
        if pid_y_kp is None: pid_y_kp = self.landing_pid_y_kp
        if pid_y_ki is None: pid_y_ki = self.landing_pid_y_ki
        if pid_y_kd is None: pid_y_kd = self.landing_pid_y_kd
        
        rospy.loginfo(f"[UnifiedFlightController] Starting precision landing to height {target_height}m")
        
        try:
            # 初始化PID控制器
            self._landing_pids = {
                'x': PIDController(pid_x_kp, pid_x_ki, pid_x_kd),
                'y': PIDController(pid_y_kp, pid_y_ki, pid_y_kd)
            }
            
            # 设置云台朝下
            if self.gimbal_available:
                self.set_gimbal(pitch=-90.0)
                rospy.sleep(2.0)  # 等待云台转动
            
            # 设置降落参数
            self._landing_params = {
                'target_height': target_height,
                'frame_width': frame_width,
                'frame_height': frame_height,
                'tolerance_x': tolerance_x,
                'tolerance_y': tolerance_y,
                'max_speed': max_speed,
                'min_speed': min_speed,
                'final_height': target_height + self.landing_final_offset
            }

             # 设置稳定时间要求
            self._required_stable_time = stable_time
            self._in_center_start_time = None  # 重置计时
            
            # 订阅检测结果
            self._detection_data = None
            # 二维码识别
            if detection_result_topic.find("qrcode_result") != -1:
                detection_msg_type = QRCodeResult
            # 水果识别
            if detection_result_topic.find("fruit_result") != -1:
                detection_msg_type = None
            detection_sub = rospy.Subscriber(detection_result_topic, detection_msg_type, self._detection_cb)
            
            # 发布开始检测命令
            start_pub = rospy.Publisher(start_detection_topic, Empty, queue_size=1)
            rospy.sleep(0.5)  # 等待发布器建立连接
            start_pub.publish(Empty())
            rospy.loginfo("[UnifiedFlightController] Detection started")
            
            # 开始精准降落控制
            self._landing_active = True
            start_time = rospy.Time.now()
            
            while self._landing_active and (rospy.Time.now() - start_time).to_sec() < timeout:
                current_height = self.current_position[2]
                
                # 检查是否需要切换到标准降落
                if current_height <= self._landing_params['final_height'] + 0.15:
                    rospy.loginfo(f"[UnifiedFlightController] Switching to standard landing at {current_height:.2f}m")
                    break
                
                rospy.sleep(0.1)
            
            # 停止检测
            start_pub.publish(Empty())
            detection_sub.unregister()
            
            # 调用标准降落
            if current_height <= self._landing_params['final_height']+ 0.20:
                self._landing_active = False  # 停止精准降落
                self.set_gimbal(0, 0, 0)  # 重置云台
                return self.land(final_z=target_height)
            else:
                rospy.logwarn("[UnifiedFlightController] Precision landing timeout")
                return False
                
        except Exception as e:
            rospy.logerr(f"[UnifiedFlightController] Precision landing failed: {e}")
            return False
        finally:
            self._landing_active = False
    
    def _detection_cb(self, msg):
        """检测结果回调（通用消息处理）"""
        try:
            # 这里需要根据实际消息类型解析
            # 假设消息包含 detected, position, confidence 字段
            # 由于是AnyMsg，需要用户保证消息格式正确
            
            # 简化处理：假设消息已经是正确格式
            # 实际使用时需要根据具体消息类型进行解析
            self._detection_data = {
                'detected': getattr(msg, 'detected', False),
                'position': getattr(msg, 'position', Point()),
                'confidence': getattr(msg, 'confidence', 0.0),
                'timestamp': rospy.Time.now()
            }
            # rospy.loginfo(f"[UnifiedFlightController] Detection data: {self._detection_data}")
        except Exception as e:
            rospy.logwarn(f"[UnifiedFlightController] Detection callback error: {e}")
    
    def _precision_landing_step(self):
        """精准降落控制步骤"""
        if not self._landing_active:
            return
        
        try:
            p_con_freq = 10
            p_con_time = 0.1

            if not self._detection_data.get('detected') and self._current_pos[2] >= 0.9:  # 降低高度查看数据
                rospy.loginfo("[UnifiedFlightController] Cannot get detection data, lowering altitude.")
                self.fly_by_velocity(0, 0, 0.1 * p_con_freq, 0.0, p_con_time)
                return

            # 检查检测数据有效性
            detection = self._detection_data
            if not detection['detected'] or detection['confidence'] < 0.5:
                # 没有检测到目标或置信度太低，悬停
                self._in_center_start_time = None  # 重置计时
                self.hold(0.1)
                return
            
            # 获取目标位置（像素坐标）
            target_x = detection['position'].x
            target_y = detection['position'].y
            
            # 计算画幅中心
            center_x = self._landing_params['frame_width'] / 2
            center_y = self._landing_params['frame_height'] / 2
            
            # 计算像素误差
            error_x = -(target_x - center_x)
            error_y = -(target_y - center_y)

            # 检查是否在容忍范围内
            in_tolerance = (abs(error_x) <= self._landing_params['tolerance_x'] and 
                        abs(error_y) <= self._landing_params['tolerance_y'])
            
            current_time = rospy.Time.now()
            
            if in_tolerance:
                # 在容忍范围内
                if self._in_center_start_time is None:
                    # 第一次进入，开始计时
                    self._in_center_start_time = current_time
                    stable_duration = 0.0
                    rospy.loginfo("[UnifiedFlightController] Entered center tolerance - starting timer")
                else:
                    # 计算已稳定时间
                    stable_duration = (current_time - self._in_center_start_time).to_sec()
                rospy.loginfo(f"[UnifiedFlightController] In center: {stable_duration:.1f}/{self._required_stable_time:.1f}s")
                
                # 检查是否稳定足够时间
                if stable_duration >= self._required_stable_time:
                    # 可以开始降落
                    current_yaw = self.current_yaw_deg
                    descent_speed = self._landing_params['min_speed']
                    
                    # 根据误差调整下降速度
                    max_error = max(abs(error_x), abs(error_y))
                    max_tolerance = max(self._landing_params['tolerance_x'], self._landing_params['tolerance_y'])

                    if max_error < max_tolerance * 0.9:  # 中速下降
                        descent_speed = self._landing_params['min_speed'] * 1.5
                    elif max_error < max_tolerance * 0.7:  # 误差很小时可以快速下降
                        descent_speed = self._landing_params['max_speed'] * 0.5
                    elif max_error < max_tolerance * 0.5:  # 误差较小时可以快速下降
                        descent_speed = self._landing_params['max_speed'] * 0.8

                    rospy.loginfo(f"[UnifiedFlightController] DESCENDING at {descent_speed:.3f}m/s")
                    rospy.loginfo(f"[UnifiedFlightController] ERROR: ({error_x:.1f}, {error_y:.1f})")
                else:
                    # 稳定时间不够，继续控制
                    current_yaw = self.current_yaw_deg
                    descent_speed = 0
                    rospy.loginfo(f"[UnifiedFlightController] Waiting for stability...")
            else:
                # 不在容忍范围内，重置计时并调整位置
                self._in_center_start_time = None
                descent_speed = 0

            # 使用PID控制调整位置
            current_time_sec = current_time.to_sec()
            
            # PID控制计算速度
            vel_x = self._landing_pids['x'].update(error_x, current_time_sec)
            vel_y = self._landing_pids['y'].update(error_y, current_time_sec)
            
            # 限制速度
            if self.simulation_mode:
                max_vel = self._landing_params['max_speed'] * 10  # 仿真时使用稍大速度
            else:
                max_vel = self._landing_params['max_speed'] * 0.5  # 现实使用较小速度

            vel_x = max(-max_vel, min(max_vel, vel_x))
            vel_y = max(-max_vel, min(max_vel, vel_y))
            
            current_yaw = self.current_yaw_deg
            
            # 发送速度命令（NED坐标系：N=y向前, E=x向右）
            self.fly_by_velocity(vel_y * p_con_freq, vel_x * p_con_freq, descent_speed * p_con_freq, current_yaw, p_con_time)

            rospy.loginfo(f"[UnifiedFlightController] Adjusting: error=({error_x:.1f},{error_y:.1f}) vel=({vel_y:.3f},{vel_x:.3f})")
            # rospy.loginfo(f"[UnifiedFlightController] Tolerance: (x:{self._landing_params['tolerance_x']:.1f}, y:{self._landing_params['tolerance_y']:.1f})")

        except Exception as e:
            rospy.logwarn(f"[UnifiedFlightController] Precision landing step error: {e}")
            self._in_center_start_time = None  # 异常时重置计时 

    # ==================== 生命周期管理 ====================
    
    def shutdown(self):
        """关闭控制器"""
        rospy.loginfo("[UnifiedFlightController] Shutting down...")
        
        # 停止精准降落
        self._landing_active = False
        
        # 发送悬停命令
        try:
            self.hold(0.5)
        except:
            pass
        
        rospy.loginfo("[UnifiedFlightController] Shutdown complete")


# ==================== 测试和演示函数 ====================

def demo_basic_flight():
    """基础飞行演示"""
    rospy.init_node("unified_flight_demo", anonymous=True)
    
    controller = UnifiedFlightController("uav1")
    
    try:
        rospy.loginfo("=== Basic Flight Demo ===")
        
        # 解锁并起飞
        if controller.simulation_mode:
            controller.arm()
        
        controller.takeoff(1.5)
        rospy.sleep(2)
        
        # # 位置控制测试
        # controller.goto(1.0, 2.3, 1.5)
        # rospy.sleep(2)
        
        # # 速度控制测试
        # controller.fly_by_velocity(0.5, 0.0, 0.0, 0.0, 3.0)
        # rospy.sleep(4)

        # # 云台测试
        # controller.set_gimbal(30.0, 0.0, 0.0)
        # rospy.sleep(1)
        # controller.set_gimbal(-90.0, 0.0, 0.0)
        # rospy.sleep(1)
        # controller.set_gimbal(0.0, 0.0, -90.0)
        # rospy.sleep(1)
        # controller.set_gimbal(0.0, 0.0, 90.0)
        # rospy.sleep(1)
        # controller.set_gimbal(0.0, 0.0, 0.0)
        # rospy.sleep(1)
        
        # # 返回
        # controller.goto(1.0, 2.3, 1.5)

        # controller.hold(5)

        # controller.goto(1.2, 2.5, 1)
        rospy.sleep(2)
        # 高级降落
        controller.precision_landing("/vision/qrcode_result", "/start_qr_detection", 0.15, 640, 480, 60, 30)

        # 降落
        # controller.land()
        
        rospy.loginfo("=== Demo Complete ===")
        
    except Exception as e:
        rospy.logerr(f"Demo failed: {e}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    # 运行演示
    demo_basic_flight()
