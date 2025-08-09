#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulation reset (lightweight version).

Refactored: 仅重定位(teleport) 小车与无人机, 不再删除/重新生成模型, 不修改其它场景对象。
适合频繁复位流程调试, 避免模型重建带来的状态乱套。

功能概述:
  订阅一个触发话题 (默认: /sim_reset_trigger, 消息类型 std_msgs/Empty)
  收到触发后:
    1. (可选) 暂停物理
    2. (可选) reset_world / reset_simulation (可通过参数关闭; 默认关闭, 避免时间/控制器被清零)
    3. set_model_state 将 smartcar / 无人机定位到预设 Pose (仅位置与 yaw)
       - 可选清零线速度与角速度
    4. (可选) 恢复物理
    5. 发布完成通知 (默认: /sim_reset_done)

与原版本差异:
  - 不调用 delete_model / spawn_model
  - 不依赖 robot_description 或 SDF param
  - 只使用 /gazebo/set_model_state

潜在注意:
  - 如果 PX4 SITL 控制器当前处于解锁/起飞中, 强制平移无人机会造成飞控估计突变; 建议在地面或锁定状态使用。
  - 若 attach 方案存在 (平台和无人机以关节固定), 在重定位前最好先 detach, 再分别 set_model_state, 最后需要的话重新 attach。
  - 如果模型名不一致(例如命名空间或多实例), 用参数覆盖默认名。

参数 (rosparam):
  ~smartcar_model_name   (string) 默认 "smartcar"
  ~drone_model_name      (string) 默认 "p230_0"
  ~smartcar_target_pose  (list[4]) [x,y,z,yaw_deg] 默认 [1.0, 2.3, 0.01, 0.0]
  ~drone_target_pose     (list[4]) 默认 [1.0, 2.3, 0.16, 0.0]
  ~zero_twist            (bool)    默认 True  (将线速度角速度清零)
  ~pause_during_reset    (bool)    默认 True
  ~unpause_after_reset   (bool)    默认 True
  ~reset_world           (bool)    默认 False
  ~reset_sim             (bool)    默认 False
  ~trigger_topic         (string)  默认 "/sim_reset_trigger"
  ~done_topic            (string)  默认 "/sim_reset_done"
  ~frame_id              (string)  默认 "world"
  ~warn_if_missing       (bool)    默认 True (若模型不存在给出警告)

使用:
  1) 启动脚本:
     rosrun ep_start sim_reset.py
  2) 触发复位:
     rostopic pub /sim_reset_trigger std_msgs/Empty "{}"
  3) 等待完成:
     rostopic echo /sim_reset_done

扩展建议:
  - 若需同时复位多个附加模型, 可添加一个模型列表参数并循环 set_model_state。
  - 若需保持当前高度不变(仅复位水平), 可以运行时添加选项改写 z。
"""

import rospy
from std_msgs.msg import Empty
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_srvs.srv import Empty as EmptySrv
import math
import traceback


def yaw_to_quat(yaw_rad: float):
    """仅绕Z轴的 quaternion."""
    half = yaw_rad * 0.5
    return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))


class SimpleSimulationReset:
    def __init__(self):
        rospy.init_node("simulation_reset_helper_light", anonymous=True)

        # 参数读取
        self.smartcar_model = rospy.get_param("~smartcar_model_name", "smartcar")
        self.drone_model = rospy.get_param("~drone_model_name", "p230_0")

        self.smartcar_target_pose = self._load_pose_param("~smartcar_target_pose", [1.0, 2.3, 0.01, 0.0])
        self.drone_target_pose = self._load_pose_param("~drone_target_pose", [1.0, 2.3, 0.16, 0.0])

        self.zero_twist = rospy.get_param("~zero_twist", True)
        self.pause_during_reset = rospy.get_param("~pause_during_reset", True)
        self.unpause_after_reset = rospy.get_param("~unpause_after_reset", True)
        self.reset_world = rospy.get_param("~reset_world", False)
        self.reset_sim = rospy.get_param("~reset_sim", False)
        self.trigger_topic = rospy.get_param("~trigger_topic", "/sim_reset_trigger")
        self.done_topic = rospy.get_param("~done_topic", "/sim_reset_done")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.warn_if_missing = rospy.get_param("~warn_if_missing", True)

        # 服务缓存
        self._srv_cache = {}

        # 订阅触发
        rospy.Subscriber(self.trigger_topic, Empty, self._on_trigger, queue_size=1)
        # 完成发布
        self._done_pub = rospy.Publisher(self.done_topic, Empty, queue_size=1)

        rospy.loginfo("[sim_reset_light] Ready. Publish Empty to %s to reset models.", self.trigger_topic)

    def _load_pose_param(self, name, default):
        val = rospy.get_param(name, default)
        if not (isinstance(val, list) and len(val) >= 4):
            rospy.logwarn("Param %s invalid, expect list[4], use default %s", name, default)
            return default
        return [float(val[0]), float(val[1]), float(val[2]), float(val[3])]

    def _wait_service(self, name, srv_type, timeout=10.0):
        if name in self._srv_cache:
            return self._srv_cache[name]
        try:
            rospy.wait_for_service(name, timeout=timeout)
            proxy = rospy.ServiceProxy(name, srv_type)
            self._srv_cache[name] = proxy
            return proxy
        except rospy.ROSException:
            rospy.logerr("[sim_reset_light] Service %s not available (timeout).", name)
            return None

    def _pause(self):
        srv = self._wait_service("/gazebo/pause_physics", EmptySrv, 5.0)
        if srv:
            try:
                srv()
            except Exception:
                rospy.logwarn("[sim_reset_light] pause_physics call failed.")

    def _unpause(self):
        srv = self._wait_service("/gazebo/unpause_physics", EmptySrv, 5.0)
        if srv:
            try:
                srv()
            except Exception:
                rospy.logwarn("[sim_reset_light] unpause_physics call failed.")

    def _reset_world_time(self):
        if self.reset_world:
            srv = self._wait_service("/gazebo/reset_world", EmptySrv, 5.0)
            if srv:
                try:
                    srv()
                except Exception:
                    rospy.logwarn("[sim_reset_light] reset_world call failed.")
        if self.reset_sim:
            srv = self._wait_service("/gazebo/reset_simulation", EmptySrv, 5.0)
            if srv:
                try:
                    srv()
                except Exception:
                    rospy.logwarn("[sim_reset_light] reset_simulation call failed.")

    def _model_exists(self, model_name):
        srv = self._wait_service("/gazebo/get_model_state", GetModelState, 5.0)
        if not srv:
            return False
        try:
            resp = srv(model_name, self.frame_id)
            return resp.success
        except Exception:
            return False

    def _set_model_state(self, model_name, target_pose):
        """
        target_pose: [x, y, z, yaw(deg)]
        """
        srv = self._wait_service("/gazebo/set_model_state", SetModelState, 5.0)
        if not srv:
            return False

        x, y, z, yaw_deg = target_pose
        yaw_rad = math.radians(yaw_deg)
        pose = Pose()
        pose.position = Point(x, y, z)
        pose.orientation = yaw_to_quat(yaw_rad)

        twist = Twist()
        if self.zero_twist:
            twist.linear = Vector3(0.0, 0.0, 0.0)
            twist.angular = Vector3(0.0, 0.0, 0.0)
        else:
            # 保持原速度：需要先 get_model_state 再写回；当前简单实现忽略
            pass

        state = ModelState()
        state.model_name = model_name
        state.pose = pose
        state.twist = twist
        state.reference_frame = self.frame_id  # world 坐标系

        try:
            resp = srv(state)
            if not resp.success:
                rospy.logwarn("[sim_reset_light] set_model_state failed for %s: %s",
                              model_name, resp.status_message)
            else:
                rospy.loginfo("[sim_reset_light] %s repositioned to (%.3f, %.3f, %.3f, yaw=%.2f°)",
                              model_name, x, y, z, yaw_deg)
            return resp.success
        except Exception as e:
            rospy.logerr("[sim_reset_light] set_model_state exception for %s: %s", model_name, e)
            return False

    def _on_trigger(self, _msg):
        rospy.loginfo("[sim_reset_light] Reset trigger received.")
        try:
            if self.pause_during_reset:
                self._pause()

            self._reset_world_time()

            # smartcar
            if self._model_exists(self.smartcar_model):
                self._set_model_state(self.smartcar_model, self.smartcar_target_pose)
            else:
                if self.warn_if_missing:
                    rospy.logwarn("[sim_reset_light] Model %s not found (skip).", self.smartcar_model)

            # drone
            if self._model_exists(self.drone_model):
                self._set_model_state(self.drone_model, self.drone_target_pose)
            else:
                if self.warn_if_missing:
                    rospy.logwarn("[sim_reset_light] Model %s not found (skip).", self.drone_model)

            if self.unpause_after_reset:
                self._unpause()

            self._done_pub.publish(Empty())
            rospy.loginfo("[sim_reset_light] Reset complete.")
        except Exception as e:
            rospy.logerr("[sim_reset_light] Reset failed: %s", e)
            traceback.print_exc()

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    SimpleSimulationReset().spin()
