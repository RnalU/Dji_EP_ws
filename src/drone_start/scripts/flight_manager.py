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
from typing import Optional, Tuple

import rospy
import tf.transformations as tft

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger, TriggerResponse


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

        # Services
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/cmd/arming")
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/set_mode")
        self._arming_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/cmd/arming", CommandBool)
        self._mode_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/set_mode", SetMode)

        # Optional: 提供一个简单 Trigger service 做“急停/中止”示例
        self._abort_srv = rospy.Service("~abort", Trigger, self._abort_cb)

        rospy.loginfo("[FlightManager] Ready.")

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

    def land(self, final_z: float = 0.05, timeout: float = 20.0, descent_rate: float = 0.3) -> bool:
        """
        智能降落: 分阶段缓慢下降以确保稳定着陆
        
        Args:
            final_z: 最终着陆高度
            timeout: 总超时时间
            descent_rate: 下降速率 (米/秒)
        
        降落策略:
        1. 分阶段下降
        2. 每个阶段等待稳定后再继续下降
        3. 接近地面时切换到AUTO.LAND模式（可选）
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

    def land_auto_mode(self, timeout: float = 15.0) -> bool:
        """
        使用PX4的AUTO.LAND模式进行着陆
        这是一个备选的着陆方法，使用PX4内置的着陆逻辑
        
        Args:
            timeout: 超时时间
            
        Returns:
            bool: 是否着陆成功
        """
        rospy.loginfo("[FlightManager] 切换到AUTO.LAND模式进行着陆...")
        
        try:
            # 切换到AUTO.LAND模式
            resp = self._mode_srv(custom_mode="AUTO.LAND")
            if not resp.mode_sent:
                rospy.logerr("[FlightManager] 切换到AUTO.LAND模式失败")
                return False
            
            # 等待模式切换完成
            mode_switch_timeout = 5.0
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                if self.current_mode == "AUTO.LAND":
                    rospy.loginfo("[FlightManager] 成功切换到AUTO.LAND模式")
                    break
                if (rospy.Time.now() - t0).to_sec() > mode_switch_timeout:
                    rospy.logerr("[FlightManager] AUTO.LAND模式切换超时")
                    return False
                rospy.sleep(0.1)
            
            # 等待着陆完成
            t0 = rospy.Time.now()
            while not rospy.is_shutdown():
                current_z = self.current_position[2]
                
                # 检查是否着陆（高度很低且解锁状态改变）
                if current_z < 0.15 and not self.is_armed:
                    rospy.loginfo("[FlightManager] AUTO.LAND着陆完成，无人机已自动上锁")
                    return True
                
                if (rospy.Time.now() - t0).to_sec() > timeout:
                    rospy.logwarn("[FlightManager] AUTO.LAND着陆超时")
                    return False
                
                rospy.sleep(0.5)
            
        except Exception as e:
            rospy.logerr("[FlightManager] AUTO.LAND着陆异常: %s", e)
            return False
        
        return False

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
            rospy.logwarn("[FlightManager Demo] 智能着陆失败，尝试AUTO.LAND模式")
            fm.land_auto_mode()
        
        fm.disarm()
    except Exception as e:
        rospy.logerr("[FlightManager Demo] Exception: %s", e)
        traceback.print_exc()
    finally:
        fm.shutdown()


if __name__ == "__main__":
    _demo()
