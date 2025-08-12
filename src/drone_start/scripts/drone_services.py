#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_services.py

统一对外仿真飞行接口，尽量模拟真实无人机(dji_psdk_ros_driver)的调用方式，使得
上层空地协同逻辑在“仿真/实机”之间可无缝切换。

已实现接口:
1. 起飞/降落服务: 复用已有 srv: takeoffOrLanding.srv
     - 请求: int8 takeoffOrLanding   (1 = 起飞, 2 = 降落)
     - 应答: int8 ack                (1 = 成功, 0 = 失败)
   服务名默认: /takeoffOrLanding     (可通过参数 ~takeoff_land_service_name 修改)

2. 速度飞行话题: flightByVel.msg  (按你提供的消息结构)
     - 话题: /flightByVel  (可通过参数 ~flight_by_vel_topic 修改)
     - 处理逻辑: 接收到一次消息后，在指定 fly_time 内按照 (vel_n, vel_e, vel_d)
       期望速度积分得到期望位置，并持续通过 FlightManager 发布当前位置 setpoint。
       targetYaw: 目标偏航角(度)，会在飞行过程中保持/切换。
       (为了保持与现有 FlightManager 兼容，这里仍旧使用位置+yaw setpoint，
        内部用速度积分得到随时间变化的目标位置)

设计说明:
- FlightManager 当前只公开位置控制接口，因此这里通过速度积分方式生成“滚动位置目标”。
- 如果需要更高精度 / 真正的速度控制，可在后续扩展 FlightManager 提供速度 setpoint 发布。
- 速度命令是“原子”的：新消息会覆盖正在执行的速度飞行；起飞/降落会终止当前速度飞行。
- land 请求后：会调用 FlightManager.land()（智能降落），并清除任何速度飞行任务。
- takeoff 请求后：会调用 FlightManager.takeoff()，高度由参数 ~takeoff_height 控制。

关键参数(ROS 参数服务器):
~uav_ns:                     默认 "uav1"
~takeoff_height:             默认 1.2  (米)
~takeoff_timeout:            默认 15.0 (秒)
~land_timeout:               默认 25.0 (秒)
~flight_loop_rate:           默认 30.0 (速度积分/发布频率 Hz)
~takeoff_land_service_name:  默认 "/takeoffOrLanding"
~flight_by_vel_topic:        默认 "/flightByVel"
~auto_enter_offboard:        默认 True (若非 OFFBOARD 自动尝试切换)
~arm_if_needed:              默认 True (起飞前自动解锁)
~yaw_fallback_use_current:   默认 True (若 targetYaw=NaN 或无效则使用当前航向)

依赖:
- flight_manager.FlightManager
- dji_psdk_ros_driver.srv.takeoffOrLanding
- dji_psdk_ros_driver.msg.flightByVel

示例:
    rosrun drone_start drone_services.py _uav_ns:=uav1 _takeoff_height:=1.5
    # 起飞:
    rosservice call /takeoffOrLanding "takeoffOrLanding: 1"
    # 发布速度飞行(北向 0.5m/s, 东向 0, 上升 0.0, 航向 90 度, 持续 4 秒):
    rostopic pub /flightByVel dji_psdk_ros_driver/flightByVel "{vel_n: 0.5, vel_e: 0.0, vel_d: 0.0, targetYaw: 90.0, fly_time: 4.0}"
    # 降落:
    rosservice call /takeoffOrLanding "takeoffOrLanding: 2"

注意:
- vel_d 为“向下”速度(>0 表示向下)，因此在积分时高度变化量 dz = vel_d * dt * (-1)。
- 若 Fly 时间 fly_time <= 0 则忽略该命令。
"""

import math
import threading
from dataclasses import dataclass
from typing import Optional

import rospy
from dji_psdk_ros_driver.srv import takeoffOrLanding, takeoffOrLandingResponse
from dji_psdk_ros_driver.msg import flightByVel

from prometheus_msgs.msg import UAVCommand, UAVControlState, UAVState

# 导入 FlightManager (假定同一 catkin 工作空间下)
from flight_manager import FlightManager


@dataclass
class VelocityCommand:
    vel_n: float
    vel_e: float
    vel_d: float  # Down 正方向
    target_yaw_deg: float
    fly_time: float
    start_time: rospy.Time
    start_pos: tuple  # (x, y, z)


class DroneServiceNode:
    def __init__(self):
        # 读取参数
        self.uav_ns = rospy.get_param("~uav_ns", "uav1")
        self.takeoff_height = rospy.get_param("~takeoff_height", 0.5)
        self.takeoff_timeout = rospy.get_param("~takeoff_timeout", 15.0)
        self.land_timeout = rospy.get_param("~land_timeout", 25.0)
        self.flight_loop_rate = rospy.get_param("~flight_loop_rate", 30.0)
        self.service_name = rospy.get_param("~takeoff_land_service_name", "/takeoffOrLanding")
        self.flight_topic = rospy.get_param("~flight_by_vel_topic", "/flightByVel")
        self.auto_enter_offboard = rospy.get_param("~auto_enter_offboard", True)
        self.arm_if_needed = rospy.get_param("~arm_if_needed", True)
        self.yaw_fallback_use_current = rospy.get_param("~yaw_fallback_use_current", True)

        rospy.loginfo("[DroneServiceNode] Init: uav_ns=%s service=%s topic=%s",
                      self.uav_ns, self.service_name, self.flight_topic)

        # FlightManager
        self.fm = FlightManager(uav_ns=self.uav_ns)
        
        # 创建无人机相关数据变量
        self.uav_control_state_sv = UAVControlState()
        self.uav_command_pv = UAVCommand()
        self.uav_state_sv = UAVState()

        self.uav_command_pv.Command_ID = 0

        self.UavCommandPb = rospy.Publisher("/uav1/prometheus/command", UAVCommand, queue_size=10)

        # 速度命令状态
        self._vel_cmd_lock = threading.Lock()
        self._active_vel_cmd: Optional[VelocityCommand] = None
        self._velocity_thread = threading.Thread(target=self._velocity_control_loop, daemon=True)
        self._velocity_thread.start()

        # 服务
        self._takeoff_land_srv = rospy.Service(self.service_name,
                                               takeoffOrLanding,
                                               self._handle_takeoff_land)

        # 话题订阅
        self._flight_sub = rospy.Subscriber(self.flight_topic,
                                            flightByVel,
                                            self._flight_by_vel_cb,
                                            queue_size=10)

        rospy.loginfo("[DroneServiceNode] Ready.")

    # ------------------------------------------------------------------
    # Service: takeoff / landing
    # ------------------------------------------------------------------
    def _handle_takeoff_land(self, req) -> takeoffOrLandingResponse:
        code = req.takeoffOrLanding
        rospy.loginfo("[DroneServiceNode] takeoffOrLanding request=%d", code)

        # 来一个新的起飞/降落请求 -> 终止当前速度飞行
        self._cancel_velocity_flight(reason="takeoff/land service invoked")

        if code == 1:
            ok = self._do_takeoff()
        elif code == 2:
            ok = self._do_land()
        else:
            rospy.logwarn("[DroneServiceNode] Unknown takeoffOrLanding code=%d", code)
            ok = False

        ack = 1 if ok else 0
        return takeoffOrLandingResponse(ack=ack)

    def _do_takeoff(self) -> bool:
        if self.arm_if_needed and not self.fm.is_armed:
            rospy.loginfo("[DroneServiceNode] Arming before takeoff...")
            if not self.fm.arm():
                rospy.logerr("[DroneServiceNode] Arm failed.")
                return False

        if self.auto_enter_offboard and self.fm.current_mode != "OFFBOARD":
            rospy.loginfo("[DroneServiceNode] Enter OFFBOARD before takeoff...")
            if not self.fm.enter_offboard():
                rospy.logerr("[DroneServiceNode] Enter OFFBOARD failed.")
                return False

        rospy.loginfo("[DroneServiceNode] Takeoff to %.2fm", self.takeoff_height)
        ok = self.fm.takeoff(target_height=self.takeoff_height,
                             timeout=self.takeoff_timeout)
        if ok:
            rospy.loginfo("[DroneServiceNode] Takeoff success.")
        else:
            rospy.logerr("[DroneServiceNode] Takeoff failed.")
        return True

    def _do_land(self) -> bool:
        rospy.loginfo("[DroneServiceNode] Landing ...")
        ok = self.fm.land(timeout=self.land_timeout)
        ok = self.fm._complete_landing_sequence()
        if ok:
            rospy.loginfo("[DroneServiceNode] Land success.")
        else:
            rospy.logerr("[DroneServiceNode] Land failed.")
        return ok

    # ------------------------------------------------------------------
    # Topic: flightByVel (速度指令 -> 通过积分生成位置 setpoint)
    # ------------------------------------------------------------------
    def _flight_by_vel_cb(self, msg: flightByVel):
        if msg.fly_time <= 0.0:
            rospy.logwarn("[DroneServiceNode] Received flightByVel fly_time<=0 ignored.")
            return

        # 起飞/进入 OFFBOARD 保障（如果需要）
        if self.arm_if_needed and not self.fm.is_armed:
            rospy.loginfo("[DroneServiceNode] Auto arm before velocity flight...")
            if not self.fm.arm():
                rospy.logerr("[DroneServiceNode] Auto arm failed; reject velocity command.")
                return
        if self.auto_enter_offboard and self.fm.current_mode != "OFFBOARD":
            rospy.loginfo("[DroneServiceNode] Auto enter OFFBOARD before velocity flight...")
            if not self.fm.enter_offboard():
                rospy.logerr("[DroneServiceNode] Enter OFFBOARD failed; reject velocity command.")
                return

        # 取得当前姿态
        start_pos = self.fm.current_position
        yaw_target = msg.targetYaw

        # 如果 yaw 不是一个有限值且允许回退 -> 使用当前航向
        if (not math.isfinite(yaw_target)) and self.yaw_fallback_use_current:
            yaw_target = self.fm.current_yaw_deg

        vel_cmd = VelocityCommand(
            vel_n=msg.vel_n,
            vel_e=msg.vel_e,
            vel_d=msg.vel_d,
            target_yaw_deg=yaw_target,
            fly_time=msg.fly_time,
            start_time=rospy.Time.now(),
            start_pos=start_pos
        )

        with self._vel_cmd_lock:
            self._active_vel_cmd = vel_cmd

        rospy.loginfo("[DroneServiceNode] Accept velocity flight: "
                      "Vn=%.2f Ve=%.2f Vd=%.2f yaw=%.1f fly_time=%.2fs",
                      msg.vel_n, msg.vel_e, msg.vel_d, yaw_target, msg.fly_time)

    def _cancel_velocity_flight(self, reason: str = ""):
        with self._vel_cmd_lock:
            if self._active_vel_cmd is not None:
                rospy.loginfo("[DroneServiceNode] Cancel velocity flight (%s)", reason)
            self._active_vel_cmd = None

    def _velocity_control_loop(self):
        rate = rospy.Rate(self.flight_loop_rate)
        while not rospy.is_shutdown():
            active: Optional[VelocityCommand] = None
            with self._vel_cmd_lock:
                if self._active_vel_cmd is not None:
                    active = self._active_vel_cmd

            if active is not None:
                elapsed = (rospy.Time.now() - active.start_time).to_sec()
                if elapsed > active.fly_time:
                    # 完成
                    rospy.loginfo("[DroneServiceNode] Velocity flight finished (elapsed=%.2f / %.2f)",
                                  elapsed, active.fly_time)
                    self._cancel_velocity_flight(reason="completed")
                else:
                    # 积分：北->x (假设 map ENU)，东->y，Down->z(反号)
                    # 注意：你的坐标系如果是 ENU：north≈x, east≈y, up≈z
                    # msg.vel_d 为向下速度 => dz = - vel_d * dt (累计到 start_pos.z + dz)
                    xn, ye, z0 = active.start_pos
                    # 直接使用 elapsed*速度 计算位移 (避免 dt 累积误差)
                    dx = active.vel_n * elapsed
                    dy = active.vel_e * elapsed
                    dz = -active.vel_d * elapsed  # down 正 -> z 减少
                    target_x = xn + dx
                    target_y = ye + dy
                    target_z = z0 + dz

                    # 为避免降到负高度
                    if target_z < 0.05:
                        target_z = 0.05

                    # 设置 FlightManager 目标
                    self.fm._set_target_pose(target_x, target_y, target_z, active.target_yaw_deg)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("[DroneServiceNode] Velocity control loop interrupted by ROS shutdown.")
                break
            except Exception as e:
                rospy.logwarn("[DroneServiceNode] Velocity control loop sleep exception: %s", e)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def shutdown(self):
        rospy.loginfo("[DroneServiceNode] Shutdown requested.")
        self._cancel_velocity_flight(reason="node shutdown")
        # FlightManager 自身有 shutdown 逻辑（若需要可调用）
        try:
            self.fm.shutdown()
        except Exception:
            pass


def main():
    rospy.init_node("drone_services", anonymous=False)
    node = DroneServiceNode()
    rospy.on_shutdown(node.shutdown)
    rospy.loginfo("[drone_services] Node spinning.")
    rospy.spin()


if __name__ == "__main__":
    main()
