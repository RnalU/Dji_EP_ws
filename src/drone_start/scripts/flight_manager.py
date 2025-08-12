#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_manager.py
  简化版 “中间层” 飞行管理器 (Mid-layer Flight Manager) – 面向顶层流程脚本的统一接口

设计目标:
  1. 为顶层使命/流程 (Mission Orchestrator) 提供一个简单的 Python API:
       - arm()
       - disarm()
       - enter_offboard()
       - takeoff(target_height, timeout)
       - goto(x, y, z, yaw_deg, timeout)
       - land(final_z=0.05, timeout=10.0)
       - hold(duration)
       - stop() / abort()
  2. 封装 MAVROS 基础交互 (arming, set_mode, position setpoint 持续发送)
  3. 鼓励后续替换为 actionlib（保留扩展挂钩）或接入真实机载控制
  4. 适配 “仿真优先” 的 Prometheus + PX4 SITL 环境；真实机上只需复写部分底层方法即可

主要特性:
  * 独立 setpoint 发布线程，自动维持 OFFBOARD（位置模式）
  * 线程安全（使用轻量锁），支持并发调用的基本防护
  * 提供状态查询： is_armed, current_mode, current_position, current_yaw
  * 统一超时处理和简易到达判据 (XY 平面距离 + Z 误差 + 航向误差)
  * 保持纯“位姿 setpoint”控制（后续可拓展速度/姿态控制模式）

依赖:
  - mavros_msgs (State, CommandBool, SetMode)
  - geometry_msgs (PoseStamped)
  - nav_msgs (Odometry)
  - tf.transformations

重要提示:
  * 使用前请确保已启动:
      /<uav_ns>/mavros/node
      PX4 SITL 及对应的定位仿真
  * OFFBOARD 模式切换前须预先发送一段持续 setpoint，否则会失败
  * takeoff / goto 采用阻塞式调用，直至成功、超时或出现中断

后续可扩展点:
  - 改为 actionlib 行为接口
  - 加入速度控制接口 (set_velocity)
  - 加入安全监控（高度限制、越界检测、急停）
  - attach/detach 钩子 (起飞前/降落后)

使用示例 (顶层脚本中):
  from flight_manager import FlightManager
  fm = FlightManager(uav_ns="uav1")
  fm.arm()
  fm.enter_offboard()
  fm.takeoff(1.2, timeout=8)
  fm.goto(3.0, 4.0, 1.2, 0.0, timeout=15)
  fm.hold(3)
  fm.land()
  fm.disarm()
  fm.shutdown()

"""

import math
import threading
import traceback
from typing import Optional, Tuple, Dict

import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse

from std_msgs.msg import Empty
from vision_pkg.msg import QRCodeResult
from dji_psdk_ros_driver.msg import flightByVel as FlightVelMsg
from dji_psdk_ros_driver.srv import takeoffOrLanding

# 可选: 云台控制服务 (真实平台上)
try:
    from dji_psdk_ros_driver.srv import gimbalControl, gimbalControlRequest
    _GIMBAL_AVAILABLE_IMPORT = True
except Exception:
    _GIMBAL_AVAILABLE_IMPORT = False

# 检查flightByVel消息是否可用
try:
    from dji_psdk_ros_driver.msg import flightByVel
    _FLIGHT_BY_VEL_AVAILABLE = True
except Exception:
    _FLIGHT_BY_VEL_AVAILABLE = False


class FlightManager:
    """
    中间层飞行管理器:
      - 管理 offboard setpoint 发布
      - 提供阻塞式原子操作 takeoff / goto / land
      - 维护内部状态 (当前目标、当前位姿)
    """

    def __init__(self,
                 uav_ns: str = "uav1",
                 frame_id: str = "map",
                 offboard_rate_hz: float = 30.0,
                 pos_reach_xy: float = 0.15,
                 pos_reach_z: float = 0.10,
                 yaw_reach_deg: float = 10.0,
                 arm_timeout: float = 5.0,
                 mode_timeout: float = 20.0,
                 wait_forever: bool = False):
        """
        初始化参数:
          uav_ns:                 MAVROS 命名空间 (例如 uav1)
          frame_id:               期望位置参考坐标系
          offboard_rate_hz:       发布 setpoint 周期
          pos_reach_xy:           位置到达判据 (水平)
          pos_reach_z:            位置到达判据 (垂直)
          yaw_reach_deg:          航向到达判据 (角度)
          arm_timeout:            解锁最大等待时间
          mode_timeout:           模式切换最大等待时间
        """
        self.uav_ns = uav_ns
        self.frame_id = frame_id
        self.offboard_rate_hz = offboard_rate_hz
        self.pos_reach_xy = pos_reach_xy
        self.pos_reach_z = pos_reach_z
        self.yaw_reach_deg = yaw_reach_deg
        self.arm_timeout = arm_timeout
        self.mode_timeout = mode_timeout
        self.wait_forever = wait_forever  # 若为 True, 关键动作(arm/offboard/takeoff/goto/land)不因超时中断, 而是持续等待

        rospy.loginfo("[FlightManager] Init with ns=%s", self.uav_ns)

        # Internal state
        self._current_state = State()
        self._odom = Odometry()
        self._has_odom = False

        self._target_pose = PoseStamped()
        self._target_pose.header.frame_id = self.frame_id
        self._target_lock = threading.Lock()

        self._offboard_thread = None
        self._offboard_active = False

        self._shutdown_flag = False

        # Subscribers
        rospy.Subscriber(f"/{self.uav_ns}/mavros/state", State, self._state_cb, queue_size=10)
        rospy.Subscriber(f"/{self.uav_ns}/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=10)

        # Publishers
        self._setpoint_pub = rospy.Publisher(
            f"/{self.uav_ns}/mavros/setpoint_position/local",
            PoseStamped,
            queue_size=20
        )
        
        # 真实无人机速度控制发布器
        if _FLIGHT_BY_VEL_AVAILABLE:
            self._flight_vel_pub = rospy.Publisher(
                f"/{self.uav_ns}/flightByVel",
                flightByVel,
                queue_size=10
            )
        else:
            self._flight_vel_pub = None

        # Services
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/cmd/arming")
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/set_mode")
        self._arming_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/cmd/arming", CommandBool)
        self._mode_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/set_mode", SetMode)

        # Optional: 提供一个简单 Trigger service 做“急停/中止”示例
        self._abort_srv = rospy.Service("~abort", Trigger, self._abort_cb)

        # ---------------- 云台控制初始化 ----------------
        self._enable_gimbal = rospy.get_param("~enable_gimbal", True)
        self._gimbal_service_name = rospy.get_param("~gimbal_service", f"/{self.uav_ns}/gimbalControl")
        self._is_simulation = rospy.get_param("~simulation_mode", True)  # 默认仿真模式
        self._gimbal_srv = None
        self._gimbal_available = False
        self._gimbal_last = {"pitch": 0.0, "roll": 0.0, "yaw": 0.0}
        self._gimbal_connection_retries = 0
        self._max_gimbal_retries = 3

        # goto_by_velocity相关参数
        self.velocity_goto_max_speed = rospy.get_param("~velocity_goto_max_speed", 1.0)  # m/s
        self.velocity_goto_kp_xy = rospy.get_param("~velocity_goto_kp_xy", 0.8)
        self.velocity_goto_kp_z = rospy.get_param("~velocity_goto_kp_z", 0.6)
        self.velocity_goto_kp_yaw = rospy.get_param("~velocity_goto_kp_yaw", 1.5)

        # 初始化云台服务连接
        self._init_gimbal_service()

        rospy.loginfo("[FlightManager] Ready.")

    # ------------------------------------------------------------------
    # Gimbal Initialization
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def _state_cb(self, msg: State):
        self._current_state = msg

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if not self._has_odom:
            self._has_odom = True

    # ------------------------------------------------------------------
    # Internal Helpers
    # ------------------------------------------------------------------
    def _wait_for_odom(self, timeout: float = 5.0) -> bool:
        """等待里程计可用"""
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and not self._has_odom:
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[FlightManager] Wait for odom timeout.")
                return False
            rospy.sleep(0.05)
        return True

    def _yaw_from_quat(self, q: Quaternion) -> float:
        roll, pitch, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def _set_target_pose(self, x: float, y: float, z: float, yaw_deg: float):
        """更新内部目标姿态"""
        with self._target_lock:
            self._target_pose.header.stamp = rospy.Time.now()
            self._target_pose.header.frame_id = self.frame_id
            self._target_pose.pose.position.x = x
            self._target_pose.pose.position.y = y
            self._target_pose.pose.position.z = z
            q = tft.quaternion_from_euler(0, 0, math.radians(yaw_deg))
            self._target_pose.pose.orientation = Quaternion(*q)

    def _publish_target_once(self):
        with self._target_lock:
            self._target_pose.header.stamp = rospy.Time.now()
            self._setpoint_pub.publish(self._target_pose)

    def _ensure_offboard_loop(self):
        if self._offboard_thread is None:
            self._offboard_active = True
            self._offboard_thread = threading.Thread(target=self._offboard_loop, daemon=True)
            self._offboard_thread.start()

    def _offboard_loop(self):
        rate = rospy.Rate(self.offboard_rate_hz)
        while not rospy.is_shutdown() and self._offboard_active:
            self._publish_target_once()
            rate.sleep()

    def _distance_xy(self, x, y) -> float:
        dx = x - self._odom.pose.pose.position.x
        dy = y - self._odom.pose.pose.position.y
        return math.hypot(dx, dy)

    def _error_z(self, z) -> float:
        return abs(z - self._odom.pose.pose.position.z)

    def _error_yaw_deg(self, yaw_deg_target: float) -> float:
        yaw_current = self._yaw_from_quat(self._odom.pose.pose.orientation)
        yaw_target = math.radians(yaw_deg_target)
        err = yaw_target - yaw_current
        err = (err + math.pi) % (2 * math.pi) - math.pi
        return abs(math.degrees(err))

    # ------------------------------------------------------------------
    # Public Status Query
    # ------------------------------------------------------------------
    @property
    def is_armed(self) -> bool:
        return self._current_state.armed

    @property
    def current_mode(self) -> str:
        return self._current_state.mode

    @property
    def current_position(self) -> Tuple[float, float, float]:
        p = self._odom.pose.pose.position
        return (p.x, p.y, p.z)

    @property
    def current_yaw_deg(self) -> float:
        return math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))

    # ------------------------------------------------------------------
    # Core Operations
    # ------------------------------------------------------------------
    def arm(self) -> bool:
        rospy.loginfo("[FlightManager] Arming request...")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.is_armed:
                rospy.loginfo("[FlightManager] Already armed.")
                return True
            try:
                resp = self._arming_srv(True)
                if not resp.success:
                    rospy.logwarn("[FlightManager] arm() service returned success=False")
            except Exception as e:
                rospy.logwarn("[FlightManager] Arming exception: %s", e)
            rospy.sleep(0.5)
            if self.is_armed:
                rospy.loginfo("[FlightManager] Armed OK.")
                return True
            if not self.wait_forever and (rospy.Time.now() - t0).to_sec() > self.arm_timeout:
                rospy.logerr("[FlightManager] Arming timeout.")
                return False
            # wait_forever=True 时继续循环直到成功
        return False

    def disarm(self) -> bool:
        rospy.loginfo("[FlightManager] Disarming...")
        try:
            self._arming_srv(False)
        except Exception as e:
            rospy.logwarn("[FlightManager] Disarm exception: %s", e)
        return True

    def enter_offboard(self) -> bool:
        """
        进入 OFFBOARD:
          - 若未获得里程计，先等待
          - 预先发布若干 setpoint
          - 请求模式切换
        """
        if not self._wait_for_odom():
            return False

        # 初始目标设为当前位姿，防止切换瞬间跳跃
        x0, y0, z0 = self.current_position
        yaw0 = self.current_yaw_deg
        self._set_target_pose(x0, y0, max(z0, 0.1), yaw0)

        # 先“预热”一批 setpoint
        rospy.loginfo("[FlightManager] Priming setpoints before OFFBOARD switch...")
        for _ in range(30):
            self._publish_target_once()
            rospy.sleep(0.03)

        self._ensure_offboard_loop()

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_mode == "OFFBOARD":
                rospy.loginfo("[FlightManager] Already in OFFBOARD.")
                return True
            try:
                resp = self._mode_srv(custom_mode="OFFBOARD")
                if not resp.mode_sent:
                    rospy.logwarn_throttle(2.0, "[FlightManager] OFFBOARD not accepted yet...")
            except Exception as e:
                rospy.logwarn("[FlightManager] Set mode exception: %s", e)
            rospy.sleep(0.4)
            if self.current_mode == "OFFBOARD":
                rospy.loginfo("[FlightManager] OFFBOARD mode OK.")
                return True
            if not self.wait_forever and (rospy.Time.now() - t0).to_sec() > self.mode_timeout:
                rospy.logerr("[FlightManager] OFFBOARD switch timeout.")
                return False
            # wait_forever=True 时持续尝试
        return False

    def takeoff(self, target_height: float, timeout: float = 10.0) -> bool:
        """阻塞式起飞: 垂直爬升到 target_height (相对地面绝对值)"""
        if not self.is_armed:
            if not self.arm():
                return False
        if self.current_mode != "OFFBOARD":
            if not self.enter_offboard():
                return False

        x0, y0, _ = self.current_position
        yaw0 = self.current_yaw_deg
        rospy.loginfo("[FlightManager] Takeoff to height=%.2f", target_height)
        self._set_target_pose(x0, y0, target_height, yaw0)

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._error_z(target_height) < self.pos_reach_z:
                rospy.loginfo("[FlightManager] Takeoff reached height.")
                return True
            if (not self.wait_forever) and (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[FlightManager] Takeoff timeout.")
                return False
            # wait_forever=True 时继续等待高度达成
            rospy.sleep(0.15)
        return False

    def goto(self, x: float, y: float, z: float, yaw_deg: float,
             timeout: float = 15.0) -> bool:
        """阻塞式位置 / 航向移动"""
        if self.current_mode != "OFFBOARD":
            rospy.logwarn("[FlightManager] goto() requires OFFBOARD, try entering...")
            if not self.enter_offboard():
                return False

        self._set_target_pose(x, y, z, yaw_deg)
        rospy.loginfo("[FlightManager] Goto (%.2f, %.2f, %.2f, yaw=%.1f°)", x, y, z, yaw_deg)

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            dxy = self._distance_xy(x, y)
            dz = self._error_z(z)
            dyaw = self._error_yaw_deg(yaw_deg)
            if dxy < self.pos_reach_xy and dz < self.pos_reach_z and dyaw < self.yaw_reach_deg:
                rospy.loginfo("[FlightManager] Goto target reached.")
                return True
            if (not self.wait_forever) and (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logerr("[FlightManager] Goto timeout (remain dxy=%.3f dz=%.3f dyaw=%.2f°)",
                             dxy, dz, dyaw)
                return False
            # wait_forever=True 时持续等待
            rospy.sleep(0.12)
        return False

    def land(self, final_z: float = 0.15, timeout: float = 20.0, descent_rate: float = 0.3) -> bool:
        """
        智能降落: 分阶段缓慢下降以确保稳定着陆
        
        Args:
            final_z: 最终着陆高度
            timeout: 总超时时间
            descent_rate: 下降速率 (米/秒)
        
        降落策略:
        1. 分阶段下降
        2. 每个阶段等待稳定后再继续下降
        3. 接近地面时切换到AUTO.LAND模式
        4. 检测着陆完成条件：高度低 + 垂直速度小
        """
        if self.current_mode != "OFFBOARD":
            rospy.logwarn("[FlightManager] land() expects OFFBOARD; attempting enter.")
            if not self.enter_offboard():
                return False

        x0, y0, current_z = self.current_position
        yaw0 = self.current_yaw_deg
        
        rospy.loginfo("[FlightManager] 开始智能降落: 从 %.2fm 到 %.2fm", current_z, final_z)
        
        # 如果当前高度已经很低，直接检查着陆条件
        if current_z <= final_z + 0.03:
            rospy.loginfo("[FlightManager] 已接近目标高度，检查着陆条件...")
            return self._check_landing_complete(final_z, timeout)
        
        # 计算下降阶段
        
        # 动态调整下降米数 
        descent_step = max(0.02, (current_z - final_z) / 2)
        t0 = rospy.Time.now()
        
        target_z = current_z
        
        while not rospy.is_shutdown() and target_z > final_z:
            # 计算下一个目标高度
            next_target_z = max(target_z - descent_step, final_z)
            
            rospy.loginfo("[FlightManager] 下降阶段: 目标高度 %.2fm", next_target_z)
            self._set_target_pose(x0, y0, next_target_z, yaw0)
            
            # 等待到达当前阶段目标
            stage_timeout = 8.0  # 每个阶段最多等待8秒
            stage_start = rospy.Time.now()
            
            while not rospy.is_shutdown():
                current_pos_z = self.current_position[2]
                
                # 检查是否到达当前阶段目标
                if abs(current_pos_z - next_target_z) < self.pos_reach_z:
                    rospy.loginfo("[FlightManager] 到达阶段目标: %.2fm", next_target_z)
                    rospy.sleep(0.5)  # 等待稳定
                    break
                
                # 检查阶段超时
                if (rospy.Time.now() - stage_start).to_sec() > stage_timeout:
                    rospy.logwarn("[FlightManager] 下降阶段超时，继续下一阶段")
                    break
                
                # 检查总超时
                if (rospy.Time.now() - t0).to_sec() > timeout:
                    rospy.logerr("[FlightManager] 降落总超时")
                    return False
                
                rospy.sleep(0.1)
            
            target_z = next_target_z
        
        # 最终着陆检查
        rospy.loginfo("[FlightManager] 进入最终着陆阶段...")
        return self._check_landing_complete(final_z, max(10.0, timeout - (rospy.Time.now() - t0).to_sec()))

    def _check_landing_complete(self, final_z: float, timeout: float) -> bool:
        """
        检查着陆完成条件
        
        Args:
            final_z: 目标着陆高度
            timeout: 超时时间
            
        Returns:
            bool: 是否着陆完成
        """
        t0 = rospy.Time.now()
        stable_count = 0
        required_stable_count = 10  # 需要连续10次检查都满足条件
        
        while not rospy.is_shutdown():
            current_z = self.current_position[2]
            
            # 获取垂直速度 与各方向加速度（如果可用）
            try:
                vz = abs(self._odom.twist.twist.linear.z)
            except:
                vz = 0.0
            
            # 着陆完成条件：
            # 1. 高度足够低
            # 2. 垂直速度足够小（稳定）
            height_ok = current_z <= (final_z + 0.08)
            velocity_ok = vz < 0.04  # 垂直速度小于0.04m/s
            
            if height_ok and velocity_ok:
                stable_count += 1
                if stable_count >= required_stable_count:
                    rospy.loginfo("[FlightManager] 着陆完成! 高度: %.3fm, 垂直速度: %.3fm/s", 
                                 current_z, vz)
                    
                    # 着陆后保持低高度一段时间确保稳定
                    x0, y0, _ = self.current_position
                    yaw0 = self.current_yaw_deg
                    self._set_target_pose(x0, y0, final_z, yaw0)
                    rospy.sleep(1.0)
                    self._offboard_active = False
                    self.fm._complete_landing_sequence()
                    
                    return True
            else:
                stable_count = 0
                rospy.loginfo_throttle(1.0, 
                    "[FlightManager] 等待着陆稳定... 高度: %.3fm, 垂直速度: %.3fm/s", 
                    current_z, vz)
            
            # 检查超时
            if (rospy.Time.now() - t0).to_sec() > timeout:
                rospy.logwarn("[FlightManager] 着陆检查超时，但仍尝试完成着陆")
                # 即使超时也返回True，因为可能已经接近着陆状态
                return True
            
            rospy.sleep(0.2)
        
        return False

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

    def precision_qr_land(self,
                           target_final_alt: float = 0.10,
                           descent_speed: float = 0.25,
                           max_xy_speed: float = 0.5,
                           align_tolerance_px: int = 30,
                           cancel_tolerance_px: int = 120,
                           alignment_stable_time: float = 1.0,
                           qr_timeout: float = 15.0,
                           camera_width: int = 640,
                           camera_height: int = 480,
                           control_rate_hz: float = 5,
                           pid_kp: float = 1,
                           pid_ki: float = 0,
                           pid_kd: float = 0,
                           pid_integral_limit: float = 0.5) -> bool:
        """
        基于二维码辅助的精确降落 (加入 PID 控制对齐):
          流程:
            1. 发布 /start_qr_detection (std_msgs/Empty) 触发二维码节点开始发布 /qrcode_result
            2. 对齐阶段: 通过 PID 控制生成水平速度 (vn, ve) 使二维码中心进入 align_tolerance_px 范围
            3. 若偏差过大 (cancel_tolerance_px) 或二维码丢失则暂停下降，仅水平对齐
            4. 对齐稳定 alignment_stable_time 且高度仍高于 target_final_alt + 0.10 -> 进入下降
            5. 下降阶段：保持 PID 微调，同时以 descent_speed (vel_d) 向下
            6. 高度 <= target_final_alt + 0.10 -> 归零速度并调用 /takeoffOrLanding (2) 触发最终降落
        
        参数:
            target_final_alt:       最终调用服务降落前接近地面的高度阈值 (m)
            descent_speed:          下降速度 (m/s) (正值表示向下, 对应 flightByVel.vel_d)
            max_xy_speed:           水平最大速度限幅 (m/s)
            align_tolerance_px:     判定已对齐的像素误差阈值
            cancel_tolerance_px:    降落过程中若偏差超出则暂停下降
            alignment_stable_time:  需要连续保持对齐的时间 (s) 才允许进入下降
            qr_timeout:             若在该时间内始终未检测到二维码则放弃
            camera_width/height:    图像尺寸 (像素)
            control_rate_hz:        控制循环频率
            pid_kp/ki/kd:           PID 控制器参数 (对归一化误差作用)
            pid_integral_limit:     积分项限幅 (归一化量), 防止积分风暴
        
        返回:
            bool: 服务降落是否成功触发 (ack=1)
        
        说明:
            - 采用归一化像素误差 ex_norm = ex / (W/2), ey_norm = ey / (H/2)
            - vn 对应 ENU 北向速度, ve 对应 ENU 东向速度
              图像坐标假设: 右=+East, 下=+South => North 误差与 ey_norm 取反
            - PID 输出直接限幅到 max_xy_speed
        """  
        # 调整云台姿态
        self.set_gimbal(pitch=-90.0, roll=0.0, yaw=0.0)

        # 发布器与服务代理
        start_qr_pub = rospy.Publisher("/start_qr_detection", Empty, queue_size=1, latch=True)
        vel_pub = rospy.Publisher("/flightByVel", FlightVelMsg, queue_size=5)
        rospy.wait_for_service("/takeoffOrLanding")
        land_srv = rospy.ServiceProxy("/takeoffOrLanding", takeoffOrLanding)
        
        qr_data = {
            "detected": False,
            "cx": 0.0,
            "cy": 0.0,
            "stamp": rospy.Time(0)
        }
        
        def _qr_cb(msg: QRCodeResult):
            qr_data["detected"] = msg.detected
            if msg.detected:
                qr_data["cx"] = msg.position.x
                qr_data["cy"] = msg.position.y
            qr_data["stamp"] = rospy.Time.now()
        
        qr_sub = rospy.Subscriber("/vision/qrcode_result", QRCodeResult, _qr_cb, queue_size=5)
        
        # 触发二维码检测
        rospy.loginfo("[FlightManager][QR-Land] 触发二维码检测...")
        # 如果未检测
        if not qr_data["detected"]:
            start_qr_pub.publish(Empty())
        
        rate = rospy.Rate(control_rate_hz)
        center_x = camera_width / 2.0
        center_y = camera_height / 2.0
        stable_start = None
        descending = False
        t_begin = rospy.Time.now()
        
        # PID 内部状态
        prev_err_x = 0.0
        prev_err_y = 0.0
        integ_x = 0.0
        integ_y = 0.0
        last_time = rospy.Time.now()
        
        def publish_velocity(vn, ve, vd, yaw, dur):
            msg = FlightVelMsg()
            msg.vel_n = float(vn)
            msg.vel_e = float(ve)
            msg.vel_d = float(vd)   # down 正
            msg.targetYaw = float(yaw)
            msg.fly_time = float(dur)
            vel_pub.publish(msg)
        
        yaw_ref = self.current_yaw_deg
        
        rospy.loginfo("[FlightManager][QR-Land] 开始对齐与下降循环 (PID 控制)...")
        success = False
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                dt = (now - last_time).to_sec()
                if dt <= 0.0:
                    dt = 1.0 / control_rate_hz
                last_time = now
                alt = self.current_position[2]
                
                # 超时
                if (now - t_begin).to_sec() > qr_timeout and not qr_data["detected"] and not qr_recent:
                    rospy.logwarn("[FlightManager][QR-Land] 超时未检测到二维码, 中止精确降落")
                    break
                
                # 最近一次有效检测时间 (判断丢失)
                qr_recent = (now - qr_data["stamp"]).to_sec() < 1.0
                
                # 计算像素误差 (若未检测到则视为 0)
                if qr_data["detected"] and qr_recent:
                    ex = qr_data["cx"] - center_x   # 像素 x 偏差 (右正)
                    ey = qr_data["cy"] - center_y   # 像素 y 偏差 (下正)
                else:
                    ex = 0.0
                    ey = 0.0
                
                # 归一化误差
                err_x_norm = -ex / (camera_width / 2.0)      
                err_y_norm = ey / (camera_height / 2.0)    
                
                aligned = (abs(ex) <= align_tolerance_px) and (abs(ey) <= align_tolerance_px) and qr_data["detected"] and qr_recent

                # 输出调试信息
                rospy.loginfo("[FlightManager][QR-Land] 当前二维码检测状态: %s", qr_data["detected"])
                rospy.loginfo("[FlightManager][QR-Land] 当前像素误差: ex=%.2f, ey=%.2f", ex, ey)

                # 对齐稳定计时
                if aligned and not descending:
                    if stable_start is None:
                        stable_start = now
                    elif (now - stable_start).to_sec() >= alignment_stable_time and alt > (target_final_alt + 0.10):
                        rospy.loginfo("[FlightManager][QR-Land] 对齐稳定, 开始下降...")
                        descending = True
                elif not aligned and not descending:
                    stable_start = None  # 重置稳定计时
                
                # 若正在下降但偏差过大 -> 暂停下降
                if descending and (abs(ex) > cancel_tolerance_px or abs(ey) > cancel_tolerance_px or not qr_data["detected"]):
                    rospy.logwarn("[FlightManager][QR-Land] 偏差过大/二维码丢失, 暂停下降重新对齐")
                    descending = False
                    stable_start = None
                
                # 接近最终高度 -> 服务降落
                if alt <= (target_final_alt + 0.10):
                    rospy.loginfo("[FlightManager][QR-Land] 接近目标高度 (%.2f m), 归零速度, 调用服务降落", alt)
                    publish_velocity(0.0, 0.0, 0.0, yaw_ref, 0.3)
                    rospy.sleep(0.3)
                    try:
                        resp = land_srv(takeoffOrLanding=2)
                        if getattr(resp, "ack", 0) == 1:
                            rospy.loginfo("[FlightManager][QR-Land] 服务降落指令发送成功 (ack=1)")
                            success = True
                        else:
                            rospy.logwarn("[FlightManager][QR-Land] 服务降落返回 ack!=1")
                        return success
                    except Exception as e:
                        rospy.logerr("[FlightManager][QR-Land] 服务降落调用失败: %s", e)
                        return False
                
                # PID 计算 (只有在检测到并近期有效时更新, 否则输出=0 并衰减积分)
                if qr_data["detected"] and qr_recent:
                    # East 方向 (ve) 使用 err_x_norm
                    # North 方向 (vn) 使用 -err_y_norm
                    err_e = err_x_norm
                    err_n = -err_y_norm
                    # 积分
                    integ_x += err_e * dt
                    integ_y += err_n * dt
                    # Anti-windup
                    integ_x = max(-pid_integral_limit, min(pid_integral_limit, integ_x))
                    integ_y = max(-pid_integral_limit, min(pid_integral_limit, integ_y))
                    # 微分
                    der_x = (err_e - prev_err_x) / dt
                    der_y = (err_n - prev_err_y) / dt
                    # PID 输出
                    ve = pid_kp * err_e + pid_ki * integ_x + pid_kd * der_x
                    vn = pid_kp * 1.5 * err_n + pid_ki * integ_y + pid_kd * der_y
                    prev_err_x = err_e
                    prev_err_y = err_n
                    # 限幅
                    ve = max(-max_xy_speed, min(max_xy_speed, ve))
                    vn = max(-max_xy_speed, min(max_xy_speed, vn))

                    rospy.loginfo("[FlightManager][QR-Land] PID 控制: ve=%.2f, vn=%.2f", ve, vn)

                else:
                    # 未检测到 -> 停止水平运动, 轻微积分衰减 (防止恢复时突跳)
                    ve = 0.0
                    vn = 0.0
                    integ_x *= 0.9
                    integ_y *= 0.9
                    prev_err_x = 0.0
                    prev_err_y = 0.0
                
                # 垂直速度
                if descending:
                    vd = descent_speed  # 正向下
                else:
                    vd = 0.0
                    # 若未开始下降也可以在完全对齐时轻微下探(可选), 目前保持 0
                
                # 若二维码丢失则完全悬停
                if not (qr_data["detected"] and qr_recent):
                    vd = 0.0
                
                publish_velocity(vn, ve, vd, yaw_ref, 1.0 / control_rate_hz)
                rate.sleep()
        finally:
            qr_sub.unregister()
        
        return success
    
    def get_gimbal_cached(self) -> Dict[str, float]:
        """返回最近一次 (缓存的) 云台姿态字典 {pitch, roll, yaw}"""
        return dict(self._gimbal_last)

    @property
    def gimbal_available(self) -> bool:
        """查询云台是否可用"""
        return self._gimbal_available

    @property
    def is_simulation_mode(self) -> bool:
        """查询是否为仿真模式"""
        return self._is_simulation

    def hold(self, duration: float):
        """保持当前目标位姿，阻塞 duration 秒"""
        rospy.loginfo("[FlightManager] Hold for %.2f s", duration)
        t_end = rospy.Time.now() + rospy.Duration(duration)
        while not rospy.is_shutdown() and rospy.Time.now() < t_end:
            rospy.sleep(0.1)

    def abort(self, reason: str = ""):
        """紧急中止: 停止 offboard 线程（不自动切模式/不上锁，交由外层处理）"""
        rospy.logwarn("[FlightManager] ABORT invoked. Reason: %s", reason)
        self._offboard_active = False

    # ------------------------------------------------------------------
    # Abort Service Callback
    # ------------------------------------------------------------------
    def _abort_cb(self, _req):
        self.abort("Abort service called")
        return TriggerResponse(success=True, message="Abort accepted")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        rospy.loginfo("[FlightManager] Shutting down...")
        self._shutdown_flag = True
        self._offboard_active = False
        if self._offboard_thread:
            self._offboard_thread.join(timeout=1.0)
            self._offboard_thread = None
        rospy.loginfo("[FlightManager] Shutdown complete.")

    def _complete_landing_sequence(self) -> bool:
        """
        切换模式并上锁
        """
        rospy.loginfo("[UnifiedFlightController] Completing landing sequence...")
        
        try:
            # 方法1：尝试切换到AUTO.LAND模式
            rospy.loginfo("[UnifiedFlightController] Attempting AUTO.LAND mode...")
            try:
                response = self._mode_srv(custom_mode="AUTO.LAND")
                if response.mode_sent:
                    rospy.loginfo("[UnifiedFlightController] Switched to AUTO.LAND mode")
                    rospy.sleep(3.0)  # 等待AUTO.LAND完成
                else:
                    rospy.logwarn("[UnifiedFlightController] AUTO.LAND mode switch failed")
            except Exception as e:
                rospy.logwarn(f"[UnifiedFlightController] AUTO.LAND failed: {e}")
            
            # 方法2：切换到MANUAL模式
            rospy.loginfo("[UnifiedFlightController] Switching to MANUAL mode...")
            try:
                response = self._mode_srv(custom_mode="MANUAL")
                if response.mode_sent:
                    rospy.loginfo("[UnifiedFlightController] Switched to MANUAL mode")
                    rospy.sleep(1.0)
                else:
                    rospy.logwarn("[UnifiedFlightController] MANUAL mode switch failed")
            except Exception as e:
                rospy.logwarn(f"[UnifiedFlightController] MANUAL mode switch failed: {e}")
            
            # 方法3：上锁
            rospy.loginfo("[UnifiedFlightController] Disarming...")
            disarm_success = self.disarm()
            if disarm_success:
                rospy.loginfo("[UnifiedFlightController] Successfully disarmed")
            else:
                rospy.logwarn("[UnifiedFlightController] Disarm failed")
            
            return disarm_success
            
        except Exception as e:
            rospy.logerr(f"[UnifiedFlightController] Landing sequence completion failed: {e}")
            return False
        
# ----------------------------------------------------------------------
# Standalone test / demo (optional)
# ----------------------------------------------------------------------
def _demo():
    """
    简单演示:
      1. Arm + Offboard
      2. Takeoff -> 1.0m
      3. Goto -> (x+0.5,y+0.5,1.0)
      4. Hold 2s
      5. Land
      6. Disarm
    """
    rospy.init_node("flight_manager_demo", anonymous=True)
    fm = FlightManager()
    try:
        if not fm.arm():
            return
        if not fm.enter_offboard():
            return
        if not fm.takeoff(1.0, timeout=8.0):
            return
        x0, y0, z0 = fm.current_position
        fm.goto(x0 + 0.5, y0 + 0.5, 1.0, fm.current_yaw_deg, timeout=10.0)
        fm.hold(2.0)
        
        # 使用改进的着陆方法
        rospy.loginfo("[FlightManager Demo] 开始着陆...")
        if not fm.land():
            rospy.logwarn("[FlightManager Demo] 智能着陆失败")
        
        fm.disarm()
    except Exception as e:
        rospy.logerr("[FlightManager Demo] Exception: %s", e)
        traceback.print_exc()
    finally:
        fm.shutdown()


if __name__ == "__main__":
    _demo()
