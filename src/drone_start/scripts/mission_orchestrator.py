#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mission_orchestrator.py
  顶级任务流程（高层编排）示例脚本

设计目的:
  - 使用中间层 FlightManager (flight_manager.py) 暴露的同步阻塞 API
  - 从 waypoints.yaml 读取命名航点
  - 根据一个简单的任务步骤列表顺序执行
  - 仅实现“调度/编排”逻辑，不直接调用 MAVROS
  - 为后续扩展（视觉识别、二维码、货架物品扫描、attach/detach 钩子）预留挂载点

特征:
  1. 自动加载配置文件 (waypoints.yaml)
  2. 提供参数化步骤序列 (ROS param ~mission_steps)
  3. 每个步骤有统一结构:
       {
         "action": "takeoff" | "goto" | "hold" | "land" | "sleep" | "log",
         "args": {...}    # 针对动作参数
         "wp": "waypoint_name"  # (可选) 若提供则从 YAML 中补全 x,y,z,yaw
       }
  4. goto动作优先使用 "wp" 指定命名点; 如果同时提供 "args": {"x":..,"y":..,"z":..,"yaw":..} 则 args 覆盖 YAML
  5. 将执行结果/进度通过日志输出 (可扩展发布到话题)

默认任务示例 (param ~mission_steps):
  - 起飞到 1.0m
  - 前往 shelf1 top col1
  - 悬停 2s
  - 前往 shelf1 row1 col2
  - 悬停
  - 返回 point1
  - 降落

运行:
  1) 确保已启动 PX4 SITL + MAVROS + 里程计
  2) rosrun drone-control-start mission_orchestrator.py
  3) 可通过 rosparam set 调整:
     rosparam set /mission_orchestrator/mission_steps "[{...},{...}]"

依赖:
  - flight_manager.py (同包 scripts 下)
  - config/waypoints.yaml

后续扩展点:
  - 将步骤抽象为插件式 (例如: register_action("scan_shelf", handler))
  - 集成视觉/感知触发
  - 故障恢复策略 (重试、跳过、终止)
  - 与车辆/平台的联动 (等待 attach/detach 完成再起飞)

"""

import os
import sys
import yaml
import rospy
import traceback
from typing import Dict, Any, List
from std_msgs.msg import Empty, Bool

# 相对导入同目录 flight_manager (确保此脚本在同一包 scripts 下)
try:
    from flight_manager import FlightManager
except ImportError:
    rospy.logerr("Cannot import flight_manager. Make sure it's in the same package scripts directory.")
    raise


class MissionOrchestrator:
    """
    高层任务编排:
      - 负责解析任务步骤列表
      - 调用 FlightManager 执行原子动作
      - 记录整体执行结果
    """

    def __init__(self):
        rospy.init_node("mission_orchestrator", anonymous=True)
        self.uav_ns = rospy.get_param("~uav_ns", "uav1")

        # Waypoint 文件路径 (默认使用包内 config/waypoints.yaml，可用参数覆盖)
        default_wp_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            "config",
            "waypoints.yaml"
        )
        self.waypoint_file = rospy.get_param("~waypoint_file", default_wp_path)

        # 加载航点
        self.waypoints = self._load_waypoints(self.waypoint_file)

        # 默认任务步骤
        self.default_steps = [
            {"action": "takeoff", "args": {"height": 0.5, "timeout": 10.0, "delay": 2.0}},
            {"action": "goto", "wp": "shelf1.row1.col1", "args": {"timeout": 15.0, "delay": 1.0}},
            {"action": "hold", "args": {"duration": 5.0, "delay": 0.5}},
            {"action": "goto", "wp": "shelf1.row1.col2", "args": {"timeout": 15.0, "delay": 1.0}},
            {"action": "hold", "args": {"duration": 5.0, "delay": 0.5}},
            {"action": "goto", "wp": "shelf1.row3.col3", "args": {"timeout": 15.0, "delay": 1.0}},
            {"action": "hold", "args": {"duration": 5.0, "delay": 0.5}},
            {"action": "goto", "wp": "points.point2", "args": {"timeout": 15.0, "delay": 1.0}},
            {"action": "land", "args": {"final_z": 0.15, "timeout": 12.0, "delay": 3.0}},
            {"action": "disarm", "args": {"delay": 0.0}}
        ]

        # 允许通过参数覆盖 mission_steps (列表形式)
        self.mission_steps: List[Dict[str, Any]] = rospy.get_param("~mission_steps", self.default_steps)

        # 航点 key 分隔符 (用 . 访问嵌套结构)
        self.wp_delim = rospy.get_param("~waypoint_delimiter", ".")

        # 创建 FlightManager
        self.fm = FlightManager(uav_ns=self.uav_ns)

        # Drone status publisher (/drone/status): Bool
        # 1 -> UAV airborne/executing, 0 -> UAV landed/idle
        self.status_pub = rospy.Publisher("/drone/status", Bool, queue_size=1, latch=True)

        # Takeoff trigger subscription (/drone/takeoff): Empty
        self.takeoff_trigger_topic = rospy.get_param("~takeoff_trigger_topic", "/drone/takeoff")
        self._takeoff_event = False
        self._takeoff_required = rospy.get_param("~wait_for_takeoff_trigger", True)
        rospy.Subscriber(self.takeoff_trigger_topic, Empty, self._takeoff_cb, queue_size=1)

        # Internal flag to mark airborne phase
        self._airborne = False

        # 运行标志
        self._abort_flag = False

    # ------------------------------------------------------------------
    # Waypoint loading and lookup
    # ------------------------------------------------------------------
    def _load_waypoints(self, path: str) -> Dict[str, Any]:
        if not os.path.exists(path):
            rospy.logwarn("Waypoint file does not exist: %s (using empty)", path)
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            rospy.loginfo("Waypoints loaded: %s", path)
            return data
        except Exception as e:
            rospy.logerr("Failed to load waypoints file %s: %s", path, e)
            return {}

    def _resolve_wp(self, wp_path: str) -> Dict[str, Any]:
        """
        支持通过 'shelf1.row1.col2' 形式访问嵌套字典
        返回包含 {x,y,z,yaw} 的字典
        """
        if not wp_path:
            raise ValueError("Empty waypoint path.")
        parts = wp_path.split(self.wp_delim)
        node = self.waypoints
        for p in parts:
            if not isinstance(node, dict) or p not in node:
                raise KeyError(f"Waypoint path component '{p}' not found in '{wp_path}'.")
            node = node[p]
        # 最终应为: {x:..., y:..., z:..., yaw:...}
        if not all(k in node for k in ("x", "y", "z", "yaw")):
            raise KeyError(f"Waypoint '{wp_path}' missing x/y/z/yaw fields: {node}")
        return dict(node)

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------
    def run(self):
        rospy.loginfo("[MissionOrchestrator] Starting mission with %d steps", len(self.mission_steps))
        # If configured, wait for external takeoff trigger before executing first takeoff
        if self._takeoff_required:
            rospy.loginfo("[MissionOrchestrator] Waiting for external takeoff trigger on %s ...", self.takeoff_trigger_topic)
            while not rospy.is_shutdown() and not self._takeoff_event:
                rospy.sleep(0.1)
            rospy.loginfo("[MissionOrchestrator] Takeoff trigger received, starting mission.")
        success = True
        for idx, step in enumerate(self.mission_steps, start=1):
            if rospy.is_shutdown() or self._abort_flag:
                rospy.logwarn("[MissionOrchestrator] Aborted / shutdown mid-mission.")
                success = False
                break

            action = step.get("action")
            args = step.get("args", {}) or {}
            wp_name = step.get("wp", None)

            rospy.loginfo("[MissionOrchestrator] Step %d/%d -> %s (wp=%s)",
                          idx, len(self.mission_steps), action, wp_name)

            try:
                # 执行任务动作
                action_success = True
                if action == "log":
                    self._do_log(args)
                elif action == "sleep":
                    self._do_sleep(args)
                elif action == "takeoff":
                    action_success = self._do_takeoff(args)
                elif action == "goto":
                    action_success = self._do_goto(args, wp_name)
                elif action == "hold":
                    self._do_hold(args)
                elif action == "land":
                    action_success = self._do_land(args)
                elif action == "disarm":
                    self._do_disarm()
                elif action == "arm":
                    self._do_arm()
                else:
                    rospy.logwarn("[MissionOrchestrator] Unknown action '%s' -> skip", action)
                    action_success = False
                
                # 检查动作是否成功
                if not action_success and action in ["takeoff", "goto", "land"]:
                    success = False
                    break
                
                # 添加任务间延迟
                delay = float(args.get("delay", 1.0))
                if delay > 0:
                    rospy.loginfo("[MissionOrchestrator] Post-action delay: %.2f s", delay)
                    rospy.sleep(delay)
            except Exception as e:
                rospy.logerr("[MissionOrchestrator] Exception in step %s: %s", action, e)
                traceback.print_exc()
                success = False
                break

        if success:
            rospy.loginfo("[MissionOrchestrator] Mission completed successfully.")
        else:
            rospy.logwarn("[MissionOrchestrator] Mission terminated with errors.")
        self.fm.shutdown()

    # ------------------------------------------------------------------
    # Action handlers
    # ------------------------------------------------------------------
    def _do_log(self, args: Dict[str, Any]):
        msg = args.get("message", "")
        rospy.loginfo("[MissionOrchestrator][LOG] %s", msg)

    def _do_sleep(self, args: Dict[str, Any]):
        duration = float(args.get("duration", 1.0))
        rospy.loginfo("[MissionOrchestrator] Sleeping %.2f s", duration)
        rospy.sleep(duration)

    def _do_takeoff(self, args: Dict[str, Any]) -> bool:
        height = float(args.get("height", 1.0))
        timeout = float(args.get("timeout", 10.0))
        # 进入 offboard + 起飞
        if not self.fm.is_armed:
            if not self.fm.arm():
                return False
        if self.fm.current_mode != "OFFBOARD":
            if not self.fm.enter_offboard():
                return False
        ok = self.fm.takeoff(height, timeout=timeout)
        if ok:
            self._airborne = True
            self.status_pub.publish(Bool(data=True))  # publish airborne status
        return ok

    def _do_goto(self, args: Dict[str, Any], wp_name: str) -> bool:
        timeout = float(args.get("timeout", 15.0))
        # 如果提供 wp_name，从 YAML 获取基础坐标
        x = args.get("x")
        y = args.get("y")
        z = args.get("z")
        yaw = args.get("yaw")

        if wp_name:
            try:
                wp = self._resolve_wp(wp_name)
                # YAML 值作为默认，显式 args 覆盖
                if x is None: x = wp["x"]
                if y is None: y = wp["y"]
                if z is None: z = wp["z"]
                if yaw is None: yaw = wp["yaw"]
            except Exception as e:
                rospy.logerr("[MissionOrchestrator] Failed to resolve waypoint '%s': %s", wp_name, e)
                return False

        if x is None or y is None or z is None or yaw is None:
            rospy.logerr("[MissionOrchestrator] Goto requires complete x/y/z/yaw (after merging wp).")
            return False

        return self.fm.goto(float(x), float(y), float(z), float(yaw), timeout=timeout)

    def _do_hold(self, args: Dict[str, Any]):
        duration = float(args.get("duration", 2.0))
        self.fm.hold(duration)

    def _do_land(self, args: Dict[str, Any]) -> bool:
        final_z = float(args.get("final_z", 0.05))
        timeout = float(args.get("timeout", 12.0))
        ok = self.fm.land(final_z=final_z, timeout=timeout)
        if ok:
            self._airborne = False
            self.status_pub.publish(Bool(data=False))  # publish landed status
        return ok

    def _do_disarm(self):
        self.fm.disarm()

    def _do_arm(self):
        if not self.fm.is_armed:
            self.fm.arm()
        else:
            rospy.loginfo("[MissionOrchestrator] UAV already armed.")

    # ------------------------------------------------------------------
    # External Abort (可拓展为订阅某个话题触发)
    # ------------------------------------------------------------------
    def abort(self, reason: str = ""):
        rospy.logwarn("[MissionOrchestrator] Abort called. Reason: %s", reason)
        self._abort_flag = True
        self.fm.abort(reason)
        self.fm.disarm()

    # ------------------------------------------------------------------
    # Utility: print loaded waypoints (debug)
    # ------------------------------------------------------------------
    def print_waypoints(self):
        rospy.loginfo("Loaded waypoints (top-level keys): %s", list(self.waypoints.keys()))

    # ------------------------------------------------------------------
    # Takeoff trigger callback
    # ------------------------------------------------------------------
    def _takeoff_cb(self, _msg: Empty):
        if not self._takeoff_event:
            self._takeoff_event = True
            rospy.loginfo("[MissionOrchestrator] Received takeoff trigger.")


def main():
    orchestrator = MissionOrchestrator()
    try:
        orchestrator.print_waypoints()
        orchestrator.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as ex:
        rospy.logerr("Mission orchestrator encountered exception: %s", ex)
        traceback.print_exc()
    finally:
        orchestrator.fm.shutdown()


if __name__ == "__main__":
    main()
