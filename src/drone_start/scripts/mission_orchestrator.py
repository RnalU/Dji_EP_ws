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
import random
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
      - 负责解析高级任务步骤
      - 调用 FlightManager 执行原子动作
      - 实现了包含扫描、识别和配送的复杂任务流
    """

    def __init__(self):
        rospy.init_node("mission_orchestrator", anonymous=True)
        self.uav_ns = rospy.get_param("~uav_ns", "uav1")

        # Waypoint 文件路径
        default_wp_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            "config", "waypoints.yaml"
        )
        self.waypoint_file = rospy.get_param("~waypoint_file", default_wp_path)

        # 加载航点
        self.waypoints = self._load_waypoints(self.waypoint_file)
        if not self.waypoints:
            rospy.logerr("Waypoints are empty. Shutting down.")
            rospy.signal_shutdown("Waypoints not loaded.")
            return

        # --- 任务配置参数 (分货架) ---
        self.scan_config = {
            "shelf1": {
                "scan_row": rospy.get_param("~scan_shelf1_row", 1),
                "special_pane": rospy.get_param("~special_pane_shelf1", "shelf1.row1.col4")
            },
            "shelf2": {
                "scan_row": rospy.get_param("~scan_shelf2_row", 2),
                "special_pane": rospy.get_param("~special_pane_shelf2", "shelf2.row2.col3")
            }
        }
        rospy.loginfo("[MissionOrchestrator] Mission Config: %s", self.scan_config)

        # 物品到配送点的映射 (物品 -> 顶部列名)
        self.delivery_mapping = {
            "apple": "col3", "banana": "col1", "watermelon": "col4", "orange": "col2"
        }
        rospy.loginfo("[MissionOrchestrator] Delivery Mapping: %s", self.delivery_mapping)

        # 动态生成高级别任务步骤
        self.mission_steps = self._generate_high_level_mission()

        # 航点 key 分隔符
        self.wp_delim = rospy.get_param("~waypoint_delimiter", ".")

        # 创建 FlightManager
        self.fm = FlightManager(uav_ns=self.uav_ns)

        # 创建任务结束标志 （用于当任务结束后，飞机触发降落命令时，待完成降落后发布状态话题给小车，继续执行下一个任务）
        self.mission_complete_flag = False

        # Status publisher and takeoff trigger
        self.status_pub = rospy.Publisher("/drone/status", Bool, queue_size=1, latch=True)
        self.takeoff_trigger_topic = rospy.get_param("~takeoff_trigger_topic", "/drone/takeoff")
        self._takeoff_event = False
        self._takeoff_required = rospy.get_param("~wait_for_takeoff_trigger", True)
        rospy.Subscriber(self.takeoff_trigger_topic, Empty, self._takeoff_cb, queue_size=1)

        self._airborne = False
        self._abort_flag = False

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
        if not wp_path: raise ValueError("Empty waypoint path.")
        parts, node = wp_path.split(self.wp_delim), self.waypoints
        for p in parts:
            if not isinstance(node, dict) or p not in node: raise KeyError(f"Waypoint path component '{p}' not found in '{wp_path}'.")
            node = node[p]
        if not all(k in node for k in ("x", "y", "z", "yaw")): raise KeyError(f"Waypoint '{wp_path}' missing x/y/z/yaw fields: {node}")
        return dict(node)

    def _generate_high_level_mission(self) -> List[Dict[str, Any]]:
        """生成顶层任务序列 (高级动作)"""
        rospy.loginfo("[MissionOrchestrator] Generating high-level mission steps...")
        steps = [
            {"action": "takeoff", "args": {"height": 1, "timeout": 10.0}},
            {"action": "scan_and_deliver", "args": {"shelf_name": "shelf1"}},
            {"action": "scan_and_deliver", "args": {"shelf_name": "shelf2"}},
            {"action": "goto", "wp": "points.point2", "args": {"timeout": 20.0}},
            {"action": "land", "args": {"final_z": 0.15, "timeout": 15.0}},
            {"action": "disarm"}
        ]
        rospy.loginfo("[MissionOrchestrator] Generated %d high-level steps.", len(steps))
        return steps

    def run(self):
        rospy.loginfo("[MissionOrchestrator] Starting mission with %d steps", len(self.mission_steps))
        if self._takeoff_required:
            rospy.loginfo("[MissionOrchestrator] Waiting for external takeoff trigger on %s ...", self.takeoff_trigger_topic)
            while not rospy.is_shutdown() and not self._takeoff_event: rospy.sleep(0.1)
            rospy.loginfo("[MissionOrchestrator] Takeoff trigger received, starting mission.")
        
        success = True
        for idx, step in enumerate(self.mission_steps, start=1):
            if rospy.is_shutdown() or self._abort_flag:
                rospy.logwarn("[MissionOrchestrator] Aborted / shutdown mid-mission.")
                success = False
                break

            action = step.get("action")
            args = step.get("args", {}) or {}
            wp_name = step.get("wp")
            delay = float(args.get("delay", 1.0))

            rospy.loginfo("[MissionOrchestrator] Step %d/%d -> %s (wp=%s)", idx, len(self.mission_steps), action, wp_name)

            try:
                action_success = self._execute_action(action, args, wp_name)
                if not action_success:
                    success = False
                    rospy.logerr(f"[MissionOrchestrator] Action '{action}' failed. Terminating mission.")
                    break
                if delay > 0:
                    rospy.loginfo("[MissionOrchestrator] Post-action delay: %.2f s", delay)
                    rospy.sleep(delay)
            except Exception as e:
                rospy.logerr("[MissionOrchestrator] Exception in step %s: %s", action, e)
                traceback.print_exc()
                success = False
                break

        rospy.loginfo("[MissionOrchestrator] Mission %s.", "completed successfully" if success else "terminated with errors")
        self.fm.shutdown()

    def _execute_action(self, action: str, args: Dict, wp_name: str) -> bool:
        """分发并执行单个动作"""
        if action == "log": return self._do_log(args)
        if action == "sleep": return self._do_sleep(args)
        if action == "takeoff": return self._do_takeoff(args)
        if action == "goto": return self._do_goto(args, wp_name)
        if action == "hold": return self._do_hold(args)
        if action == "land": return self._do_land(args)
        if action == "disarm": return self._do_disarm()
        if action == "arm": return self._do_arm()
        if action == "scan_and_deliver": return self._do_scan_and_deliver(args)
        
        rospy.logwarn("[MissionOrchestrator] Unknown action '%s' -> skip", action)
        return False

    def _simulate_visual_recognition(self, shelf_name: str, pane_name: str) -> str:
        """
        视觉识别接口。
        TODO: 替换为真实的视觉识别逻辑。
        """
        rospy.loginfo(f"[MissionOrchestrator] Simulating recognition for {pane_name} on {shelf_name}...")
        rospy.sleep(2.0) # 模拟识别耗时
        recognized_item = random.choice(list(self.delivery_mapping.keys()))
        rospy.loginfo(f"[MissionOrchestrator] Recognized item: '{recognized_item}'")
        return recognized_item

    def _do_scan_and_deliver(self, args: Dict) -> bool:
        """处理单个货架的“扫描-识别-配送”完整流程"""
        shelf_name = args.get("shelf_name")
        if not shelf_name:
            rospy.logerr("[MissionOrchestrator] 'shelf_name' not provided for scan_and_deliver.")
            return False

        config = self.scan_config.get(shelf_name, {})
        scan_row_name = f"row{config.get('scan_row')}"
        special_pane_wp = config.get("special_pane")
        rows_to_scan = [scan_row_name, "row3"]

        rospy.loginfo(f"[MissionOrchestrator] Starting 'scan_and_deliver' for {shelf_name}. Rows: {rows_to_scan}")

        try:
            num_cols = len(self.waypoints[shelf_name][rows_to_scan[0]])
        except KeyError:
            rospy.logerr(f"Cannot determine column count for {shelf_name}, skipping.")
            return False

        item_to_deliver = None

        # --- 1. 扫描阶段 ---
        rospy.loginfo(f"--- Phase 1: Scanning {shelf_name} ---")
        for row_name in rows_to_scan:
            for col_idx in range(1, num_cols + 1):
                wp_name = f"{shelf_name}.{row_name}.col{col_idx}"
                rospy.loginfo(f"[MissionOrchestrator] Scanning -> {wp_name}")
                # 开始扫描前根据扫描的货架调整云台角度
                if shelf_name == "shelf1":
                    if not self._do_gimbal_control({"yaw": 90, "pitch": 0}): return False
                if shelf_name == "shelf2":
                    if not self._do_gimbal_control({"yaw": -90, "pitch": 0}): return False
                if not self._do_goto({}, wp_name): return False
                if not self._do_hold({"duration": 3.0}): return False

                # 如果是特殊窗格，识别并记录物品，但不立即配送
                if wp_name == special_pane_wp:
                    rospy.loginfo(f"Special pane {wp_name} found. Performing recognition.")
                    item_to_deliver = self._simulate_visual_recognition(shelf_name, wp_name)
        
        rospy.loginfo(f"--- Phase 1: Scan of {shelf_name} complete. ---")

        # --- 2. 配送阶段 ---
        if item_to_deliver:
            rospy.loginfo(f"--- Phase 2: Delivering item '{item_to_deliver}' for {shelf_name} ---")
            delivery_col = self.delivery_mapping.get(item_to_deliver)
            if not delivery_col:
                rospy.logwarn(f"No delivery spot found for item '{item_to_deliver}'. Skipping delivery.")
            else:
                delivery_wp_name = f"{shelf_name}.top.{delivery_col}"
                rospy.loginfo(f"Delivery task: item '{item_to_deliver}' -> spot '{delivery_wp_name}'")
                
                # 在开始配送前先调整云台姿态
                if not self._do_gimbal_control({"yaw": 0, "pitch": -90}): return False

                # 1. 上升到安全高度 (比顶部航点高0.5m)
                try:
                    top_z = self._resolve_wp(delivery_wp_name)['z']
                    safe_z = top_z + 0.5
                    rospy.loginfo(f"Ascending to safe height: {safe_z}m")
                    if not self.fm.goto(self.fm.current_position[0], self.fm.current_position[1], safe_z, self.fm.current_yaw_deg, timeout=10.0):
                        return False
                except Exception as e:
                    rospy.logerr(f"Could not resolve delivery waypoint Z for safety check: {e}")
                    return False

                # 2. 飞到配送点
                rospy.loginfo(f"Moving to delivery spot: {delivery_wp_name}")
                if not self._do_goto({}, delivery_wp_name): return False
                if not self._do_hold({"duration": 5.0}): return False
                
                # 3. 临时降落配送
                rospy.loginfo("Simulating delivery...")
                # if not self._do_hold({"duration": 5.0}): return False
                if not self._do_gimbal_control({"yaw": 0, "pitch": -90}): return False
                if not self._do_land({"final_z": 2.02, "timeout": 10.0}):
                    rospy.logerr("Failed to land for delivery simulation.")
                    return False
                rospy.loginfo("Delivery complete.")

                # 4. 检测本次配送的货架ID，并执行相应的后续动作
                # 这里无论是货架1还是货架2，配送完成后都需要回到中心点
                rospy.loginfo("Moving to center for next shelf delivery...")
                # 先在当前位置升高
                if not self.fm.goto(self.fm.current_position[0], self.fm.current_position[1], safe_z + 0.5, self.fm.current_yaw_deg, timeout=10.0):
                    rospy.logerr("Failed to ascend to center position after delivery.")
                    return False
                # 然后前往中心点
                if not self.fm.goto(4.5, 2.3, safe_z + 0.5, self.fm.current_yaw_deg, timeout=10.0):
                    rospy.logerr("Failed to ascend to center position after delivery.")
                    return False
                # 下降到合适的高度
                if not self.fm.goto(4.5, 2.3, 1, self.fm.current_yaw_deg, timeout=10.0):
                    rospy.logerr("Failed to descend to center position after delivery.")
                    return False
                # 悬停稳定
                # if not self._do_hold({"duration": 2.0}):
                #     rospy.logerr("Failed to hold at center position after delivery.")
                #     return False
                
                # 货架1配送完成，准备开始货架2的扫描和配送
                if shelf_name == "shelf1":    
                    self._do_gimbal_control({"yaw": 0, "pitch": 0})
                    rospy.loginfo("Ready for shelf2 delivery.")

                # 如果是货架2的配送，配送完成后回到
                elif shelf_name == "shelf2":
                    # 货架2的配送已完成
                    rospy.loginfo("Shelf2 delivery completed. Mission will end after landing.")
                    self._do_gimbal_control({"yaw": 0, "pitch": 0})
                    self.mission_complete_flag = True

        else:
            rospy.logwarn(f"No special item found during scan of {shelf_name}. No delivery will be made.")

        rospy.loginfo(f"[MissionOrchestrator] 'scan_and_deliver' for {shelf_name} completed.")
        return True

    # --- 原子动作处理器 ---
    def _do_log(self, args: Dict) -> bool:
        rospy.loginfo("[MissionOrchestrator][LOG] %s", args.get("message", ""))
        return True

    def _do_sleep(self, args: Dict) -> bool:
        duration = float(args.get("duration", 1.0))
        rospy.loginfo("[MissionOrchestrator] Sleeping %.2f s", duration)
        rospy.sleep(duration)
        return True

    def _do_takeoff(self, args: Dict) -> bool:
        height = float(args.get("height", 1.0))
        timeout = float(args.get("timeout", 10.0))
        if not self.fm.is_armed and not self.fm.arm(): return False
        if self.fm.current_mode != "OFFBOARD" and not self.fm.enter_offboard(): return False
        ok = self.fm.takeoff(height, timeout=timeout)
        if ok:
            self._airborne = True
            self.status_pub.publish(Bool(data=True))
        return ok

    def _do_goto(self, args: Dict, wp_name: str) -> bool:
        timeout = float(args.get("timeout", 15.0))
        x, y, z, yaw = args.get("x"), args.get("y"), args.get("z"), args.get("yaw")
        if wp_name:
            try:
                wp = self._resolve_wp(wp_name)
                if x is None: x = wp["x"]
                if y is None: y = wp["y"]
                if z is None: z = wp["z"]
                if yaw is None: yaw = wp["yaw"]
            except Exception as e:
                rospy.logerr("[MissionOrchestrator] Failed to resolve waypoint '%s': %s", wp_name, e)
                return False
        if any(v is None for v in [x, y, z, yaw]):
            rospy.logerr("[MissionOrchestrator] Goto requires complete x/y/z/yaw.")
            return False
        return self.fm.goto(float(x), float(y), float(z), float(yaw), timeout=timeout)

    def _do_hold(self, args: Dict) -> bool:
        duration = float(args.get("duration", 2.0))
        self.fm.hold(duration)
        return True

    def _do_land(self, args: Dict) -> bool:
        final_z = float(args.get("final_z", 0.05))
        timeout = float(args.get("timeout", 12.0))
        ok = self.fm.land(final_z=final_z, timeout=timeout)
        if ok and self.mission_complete_flag:
            self._airborne = False
            self.status_pub.publish(Bool(data=False))
        return ok
    
    def _do_gimbal_control(self, args: Dict) -> bool:
        yaw = float(args.get("yaw", 0.0))
        pitch = float(args.get("pitch", 0.0))
        return self.fm.set_gimbal(pitch, 0, yaw, timeout=5.0)

    def _do_disarm(self) -> bool:
        self.fm.disarm()
        return True

    def _do_arm(self) -> bool:
        if not self.fm.is_armed: return self.fm.arm()
        rospy.loginfo("[MissionOrchestrator] UAV already armed.")
        return True

    def abort(self, reason: str = ""):
        rospy.logwarn("[MissionOrchestrator] Abort called. Reason: %s", reason)
        self._abort_flag = True
        self.fm.abort(reason)
        self.fm.disarm()

    def print_waypoints(self):
        rospy.loginfo("Loaded waypoints (top-level keys): %s", list(self.waypoints.keys()))

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
        if orchestrator and hasattr(orchestrator, 'fm'):
            orchestrator.fm.shutdown()

if __name__ == "__main__":
    main()
