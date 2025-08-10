#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
direct_gimbal_controller.py
  直接使用Gazebo API的云台控制器 - 避免ros_control冲突

功能:
  1. 提供gimbalControl服务
  2. 直接通过Gazebo关节控制API控制云台
  3. 发布关节状态
  4. 完全避免ros_control系统

使用方法:
  rosrun sim_pkg direct_gimbal_controller.py _uav_ns:=uav1
"""

import rospy
import math
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetJointProperties, GetJointProperties, ApplyJointEffort
from gazebo_msgs.msg import ModelStates
from dji_psdk_ros_driver.srv import gimbalControl, gimbalControlRequest, gimbalControlResponse

class DirectGimbalController:
    """直接Gazebo API云台控制器"""
    
    def __init__(self):
        rospy.init_node("direct_gimbal_controller", anonymous=True)
        
        # 参数
        self.uav_ns = rospy.get_param("~uav_ns", "uav1")
        self.model_name = "gimbal_camera_attachment"
        
        # 当前目标角度
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        
        # 当前实际角度
        self.current_yaw = 0.0
        self.current_pitch = 0.0
        
        # 关节状态发布器
        self.joint_state_pub = rospy.Publisher(
            f"/{self.model_name}/joint_states", 
            JointState, 
            queue_size=1
        )
        
        # 等待Gazebo服务
        rospy.wait_for_service('/gazebo/get_joint_properties')
        rospy.wait_for_service('/gazebo/apply_joint_effort')
        
        # Gazebo服务代理
        self.get_joint_props = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        self.apply_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        
        # 云台控制服务
        self.gimbal_service = rospy.Service(
            f"/{self.uav_ns}/gimbalControl",
            gimbalControl,
            self.gimbal_control_callback
        )
        
        # 定时器 - 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.02), self.control_loop)  # 50Hz
        
        rospy.loginfo(f"[DirectGimbalController] Started for {self.uav_ns}")
        
    def get_joint_state(self, joint_name):
        """获取关节状态"""
        try:
            joint_full_name = f"{self.model_name}::{joint_name}"
            props = self.get_joint_props(joint_full_name)
            if props.position:
                return props.position[0]
            return 0.0
        except Exception as e:
            rospy.logdebug(f"[DirectGimbalController] Failed to get joint state for {joint_name}: {e}")
            return 0.0
    
    def control_joint(self, joint_name, target_angle):
        """控制关节到目标角度"""
        try:
            # 获取当前角度
            current_angle = self.get_joint_state(joint_name)
            
            # 计算误差
            error = target_angle - current_angle
            
            # 简单PD控制
            kp = 0.5  # 比例增益
            kd = 0.0   # 微分增益
            
            # 计算控制力矩
            effort = kp * error
            
            # 限制力矩
            max_effort = 5.0
            effort = max(-max_effort, min(max_effort, effort))
            
            # 应用力矩
            joint_full_name = f"{self.model_name}::{joint_name}"
            self.apply_joint_effort(
                joint_name=joint_full_name,
                effort=effort,
                duration=rospy.Duration(0.1)
            )
            
            return current_angle
            
        except Exception as e:
            rospy.logdebug(f"[DirectGimbalController] Failed to control joint {joint_name}: {e}")
            return 0.0
    
    def control_loop(self, event):
        """控制循环"""
        try:
            # 控制各关节
            self.current_yaw = self.control_joint("gimbal_yaw_joint", self.target_yaw)
            self.current_pitch = self.control_joint("gimbal_pitch_joint", self.target_pitch)
            
            # 发布关节状态
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            joint_state.name = ["gimbal_yaw_joint", "gimbal_pitch_joint"]
            joint_state.position = [self.current_yaw, self.current_pitch]
            joint_state.velocity = [0.0, 0.0]
            joint_state.effort = [0.0, 0.0]
            
            self.joint_state_pub.publish(joint_state)
            
        except Exception as e:
            rospy.logdebug(f"[DirectGimbalController] Control loop error: {e}")
    
    def gimbal_control_callback(self, req):
        """云台控制服务回调"""
        try:
            # 角度转换 (度转弧度)
            target_yaw = math.radians(req.yaw)
            target_pitch = math.radians(req.pitch)
            
            # 角度限制 (匹配新的URDF配置)
            target_yaw = max(-math.pi, min(math.pi, target_yaw))
            target_pitch = max(-0.6109, min(1.5708, target_pitch))  # -35° to +90°
            
            # 更新目标角度
            self.target_yaw = target_yaw
            self.target_pitch = target_pitch
            
            rospy.loginfo(f"[DirectGimbalController] Gimbal command: yaw={math.degrees(target_yaw):.1f}°, pitch={math.degrees(target_pitch):.1f}°")
            
            # 创建响应
            resp = gimbalControlResponse()
            try:
                resp.success = True
                resp.message = "Gimbal command executed"
            except AttributeError:
                pass
                
            return resp
            
        except Exception as e:
            rospy.logerr(f"[DirectGimbalController] Error in gimbal control: {e}")
            resp = gimbalControlResponse()
            try:
                resp.success = False
                resp.message = f"Error: {e}"
            except AttributeError:
                pass
            return resp

def main():
    try:
        controller = DirectGimbalController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"DirectGimbalController error: {e}")

if __name__ == "__main__":
    main()
