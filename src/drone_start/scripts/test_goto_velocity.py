#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通过flightByVel话题控制速度来实现位置控制

使用方法:
    rosrun drone_start test_goto_velocity.py _uav_ns:=uav1

    确保真实无人机已经连接并启动相应的ROS驱动
    确保flightByVel话题可用
    初始位置假设为: (1, 2.3, 0.15)米
"""

import rospy
import math
from flight_manager import FlightManager

def test_goto_velocity():
    """测试goto_by_velocity功能"""
    rospy.init_node("test_goto_velocity", anonymous=False)
    
    # 获取参数
    uav_ns = rospy.get_param("~uav_ns", "uav1")
    
    rospy.loginfo("[test_goto_velocity] Starting test with UAV namespace: %s", uav_ns)
    
    # 创建FlightManager实例
    fm = FlightManager(
        uav_ns=uav_ns,
        pos_reach_xy=0.2,      # 水平到达精度 20cm
        pos_reach_z=0.15,      # 垂直到达精度 15cm
        yaw_reach_deg=8.0,     # 偏航角到达精度 8度
        wait_forever=False     # 使用超时机制
    )
    
    try:
        # 等待一下让系统稳定
        rospy.sleep(2.0)
        
        # 检查初始状态
        rospy.loginfo("[test_goto_velocity] Current position: (%.2f, %.2f, %.2f)", 
                      *fm.current_position)
        rospy.loginfo("[test_goto_velocity] Current yaw: %.1f°", fm.current_yaw_deg)
        rospy.loginfo("[test_goto_velocity] Armed: %s", fm.is_armed)
        rospy.loginfo("[test_goto_velocity] Mode: %s", fm.current_mode)
        
        # 如果需要，先解锁
        if not fm.is_armed:
            rospy.loginfo("[test_goto_velocity] Arming...")
            if not fm.arm():
                rospy.logerr("[test_goto_velocity] Failed to arm!")
                return False
            rospy.loginfo("[test_goto_velocity] Armed successfully")
        
        # 如果需要，进入OFFBOARD模式
        if fm.current_mode != "OFFBOARD":
            rospy.loginfo("[test_goto_velocity] Entering OFFBOARD mode...")
            if not fm.enter_offboard():
                rospy.logerr("[test_goto_velocity] Failed to enter OFFBOARD!")
                return False
            rospy.loginfo("[test_goto_velocity] OFFBOARD mode entered")
        
        # 测试序列：从初始位置(1, 2.3, 0.15)开始
        test_waypoints = [
            # (x, y, z, yaw_deg, max_velocity, description)
            (1.5, 2.3, 1.0, 0.0, 0.5, "上升到1米高度"),
            (2.0, 2.3, 1.0, 0.0, 0.8, "向前移动0.5米"),
            (2.0, 3.0, 1.0, 90.0, 0.8, "向右移动0.7米并转向90度"),
            (1.5, 3.0, 1.2, 180.0, 0.6, "向后移动0.5米，上升到1.2米，转向180度"),
            (1.0, 2.5, 0.8, 270.0, 0.7, "返回接近初始位置，下降到0.8米，转向270度"),
            (1.0, 2.3, 0.3, 0.0, 0.3, "缓慢降落到0.3米，恢复0度航向")
        ]
        
        rospy.loginfo("[test_goto_velocity] Starting waypoint navigation test...")
        
        for i, (x, y, z, yaw, max_vel, desc) in enumerate(test_waypoints):
            rospy.loginfo("[test_goto_velocity] Waypoint %d: %s", i+1, desc)
            rospy.loginfo("[test_goto_velocity] Target: (%.2f, %.2f, %.2f, yaw=%.1f°), max_vel=%.2f m/s", 
                          x, y, z, yaw, max_vel)
            
            # 执行goto_by_velocity
            success = fm.goto_by_velocity(
                x=x, y=y, z=z, yaw_deg=yaw,
                max_velocity=max_vel,
                timeout=25.0
            )
            
            if success:
                rospy.loginfo("[test_goto_velocity] Waypoint %d completed successfully!", i+1)
                # 在每个航点停留2秒
                rospy.loginfo("[test_goto_velocity] Holding position for 2 seconds...")
                rospy.sleep(2.0)
            else:
                rospy.logerr("[test_goto_velocity] Waypoint %d failed!", i+1)
                break
                
        rospy.loginfo("[test_goto_velocity] Test sequence completed!")
        
        # 最终状态检查
        final_pos = fm.current_position
        final_yaw = fm.current_yaw_deg
        rospy.loginfo("[test_goto_velocity] Final position: (%.2f, %.2f, %.2f)", *final_pos)
        rospy.loginfo("[test_goto_velocity] Final yaw: %.1f°", final_yaw)
        
        return True
        
    except KeyboardInterrupt:
        rospy.loginfo("[test_goto_velocity] Test interrupted by user")
        return False
    except Exception as e:
        rospy.logerr("[test_goto_velocity] Test failed with exception: %s", str(e))
        return False
    finally:
        # 清理
        rospy.loginfo("[test_goto_velocity] Shutting down FlightManager...")
        fm.shutdown()

def test_basic_movement():
    """简单的基础移动测试"""
    rospy.init_node("test_basic_movement", anonymous=False)
    
    uav_ns = rospy.get_param("~uav_ns", "uav1")
    fm = FlightManager(uav_ns=uav_ns)
    
    try:
        rospy.loginfo("[test_basic_movement] Testing basic movement...")
        
        # 简单测试：从初始位置移动到附近一点
        current_pos = fm.current_position
        target_x = current_pos[0] + 0.5
        target_y = current_pos[1] + 0.3
        target_z = max(current_pos[2], 1.0)  # 至少1米高
        
        rospy.loginfo("[test_basic_movement] Moving from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
                      *current_pos, target_x, target_y, target_z)
        
        success = fm.goto_by_velocity(
            x=target_x, y=target_y, z=target_z, yaw_deg=0.0,
            max_velocity=0.5, timeout=20.0
        )
        
        if success:
            rospy.loginfo("[test_basic_movement] Basic movement test successful!")
        else:
            rospy.logerr("[test_basic_movement] Basic movement test failed!")
            
        return success
        
    except Exception as e:
        rospy.logerr("[test_basic_movement] Exception: %s", str(e))
        return False
    finally:
        fm.shutdown()

def main():
    """主函数"""
    try:
        # 选择测试模式
        test_mode = rospy.get_param("~test_mode", "full")  # "full" 或 "basic"
        
        if test_mode == "basic":
            rospy.loginfo("Running basic movement test...")
            success = test_basic_movement()
        else:
            rospy.loginfo("Running full waypoint test...")
            success = test_goto_velocity()
            
        if success:
            rospy.loginfo("Test completed successfully!")
        else:
            rospy.logerr("Test failed!")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted by ROS shutdown")
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")

if __name__ == "__main__":
    main()
