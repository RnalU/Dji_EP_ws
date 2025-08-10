#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
setup_gimbal_controllers.py
  延迟设置云台控制器的脚本

功能:
  1. 等待云台模型完全加载
  2. 加载控制器配置到正确的命名空间
  3. 启动控制器

使用方法:
  rosrun sim_pkg setup_gimbal_controllers.py
"""

import rospy
import subprocess
import time
import os
from gazebo_msgs.srv import GetModelState

def main():
    rospy.init_node("setup_gimbal_controllers", anonymous=True)
    
    rospy.loginfo("[SetupGimbalControllers] 等待云台模型加载...")
    
    # 等待Gazebo服务
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    # 等待云台模型完全加载
    max_wait = 30
    wait_count = 0
    while not rospy.is_shutdown() and wait_count < max_wait:
        try:
            state = get_model_state("gimbal_camera_attachment", "")
            if state.success:
                rospy.loginfo("[SetupGimbalControllers] ✓ 云台模型已加载")
                break
        except Exception as e:
            rospy.logdebug(f"[SetupGimbalControllers] 等待模型: {e}")
        
        time.sleep(1)
        wait_count += 1
    
    if wait_count >= max_wait:
        rospy.logerr("[SetupGimbalControllers] 等待云台模型超时!")
        return
    
    # 额外等待确保模型稳定
    rospy.loginfo("[SetupGimbalControllers] 等待模型稳定...")
    time.sleep(3)
    
    try:
        # 加载控制器配置
        rospy.loginfo("[SetupGimbalControllers] 加载控制器配置...")
        config_file = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            "config",
            "gimbal_controllers.yaml"
        )
        
        cmd = f"rosparam load {config_file} /gimbal_camera_attachment"
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            rospy.loginfo("[SetupGimbalControllers] ✓ 控制器配置已加载")
        else:
            rospy.logerr(f"[SetupGimbalControllers] 加载配置失败: {result.stderr}")
            return
        
        # 启动控制器
        rospy.loginfo("[SetupGimbalControllers] 启动控制器...")
        time.sleep(1)
        
        cmd = "rosrun controller_manager spawner joint_state_controller gimbal_yaw_joint_position_controller gimbal_pitch_joint_position_controller __ns:=/gimbal_camera_attachment"
        
        # 使用subprocess.Popen来异步启动，因为spawner会持续运行
        process = subprocess.Popen(cmd, shell=True)
        
        rospy.loginfo("[SetupGimbalControllers] ✓ 控制器启动命令已执行")
        
        # 等待一段时间检查控制器是否成功启动
        time.sleep(5)
        
        if process.poll() is None:
            rospy.loginfo("[SetupGimbalControllers] ✓ 控制器正在运行")
        else:
            rospy.logwarn("[SetupGimbalControllers] 控制器进程可能已退出")
        
    except Exception as e:
        rospy.logerr(f"[SetupGimbalControllers] 设置控制器失败: {e}")

if __name__ == "__main__":
    try:
        main()
        rospy.loginfo("[SetupGimbalControllers] 设置完成")
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[SetupGimbalControllers] 异常: {e}")
