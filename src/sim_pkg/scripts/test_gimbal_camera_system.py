#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_gimbal_camera_system.py
  综合测试脚本 - 验证云台相机系统

功能:
  1. 测试Gazebo云台关节控制
  2. 验证相机图像发布
  3. 测试与flight_manager的集成
  4. 验证TF变换正确性

使用方法:
  # 先启动云台相机仿真
  roslaunch sim_pkg uav_with_gimbal_simulation.launch show_camera:=true
  
  # 运行测试
  rosrun sim_pkg test_gimbal_camera_system.py
"""

import rospy
import math
import cv2
import numpy as np
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from dji_psdk_ros_driver.srv import gimbalControl, gimbalControlRequest
import tf2_ros
from geometry_msgs.msg import TransformStamped

class GimbalCameraSystemTester:
    """云台相机系统测试器"""
    
    def __init__(self):
        rospy.init_node("gimbal_camera_system_tester", anonymous=True)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.joint_states = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅器
        rospy.Subscriber("/uav1/gimbal_camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/gimbal/joint_states", JointState, self.joint_state_callback)
        
        # 等待云台服务
        rospy.wait_for_service("/uav1/gimbalControl", timeout=10.0)
        self.gimbal_srv = rospy.ServiceProxy("/uav1/gimbalControl", gimbalControl)
        
        rospy.loginfo("[GimbalCameraSystemTester] 初始化完成")
        
    def image_callback(self, msg):
        """图像回调"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"图像转换失败: {e}")
    
    def joint_state_callback(self, msg):
        """关节状态回调"""
        for i, name in enumerate(msg.name):
            if name in ["gimbal_yaw_joint", "gimbal_pitch_joint"]:
                self.joint_states[name] = msg.position[i]
    
    def test_image_publishing(self):
        """测试图像发布"""
        rospy.loginfo("=== 测试1: 相机图像发布 ===")
        
        # 等待图像
        timeout = 10.0
        start_time = rospy.Time.now()
        
        while self.latest_image is None and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("等待相机图像超时！")
                return False
            rospy.sleep(0.1)
        
        if self.latest_image is not None:
            h, w = self.latest_image.shape[:2]
            rospy.loginfo(f"✓ 相机图像接收成功: {w}x{h}")
            
            # 保存测试图像
            cv2.imwrite("/tmp/gimbal_camera_test.jpg", self.latest_image)
            rospy.loginfo("✓ 测试图像已保存到 /tmp/gimbal_camera_test.jpg")
            return True
        
        return False
    
    def test_gimbal_control(self):
        """测试云台控制"""
        rospy.loginfo("=== 测试2: 云台关节控制 ===")
        
        # 测试角度列表
        test_angles = [
            (0, 0),      # 中性位置
            (-30, 45),   # 向下看，右转
            (-60, -45),  # 向下看，左转
            (0, 90),     # 水平，右转90度
            (0, -90),    # 水平，左转90度
            (0, 0),      # 回到中性
        ]
        
        success_count = 0
        
        for pitch_deg, yaw_deg in test_angles:
            rospy.loginfo(f"测试角度: pitch={pitch_deg}°, yaw={yaw_deg}°")
            
            try:
                # 发送云台命令
                req = gimbalControlRequest()
                req.pitch = pitch_deg
                req.yaw = yaw_deg
                req.roll = 0
                
                resp = self.gimbal_srv(req)
                
                # 等待运动完成
                rospy.sleep(3.0)
                
                # 检查关节状态
                expected_pitch_rad = math.radians(pitch_deg)
                expected_yaw_rad = math.radians(yaw_deg)
                
                if "gimbal_pitch_joint" in self.joint_states:
                    actual_pitch = self.joint_states["gimbal_pitch_joint"]
                    pitch_error = abs(actual_pitch - expected_pitch_rad)
                    
                    if pitch_error < 0.1:  # 允许5.7度误差
                        rospy.loginfo(f"✓ Pitch角度正确: 期望={pitch_deg:.1f}°, 实际={math.degrees(actual_pitch):.1f}°")
                        success_count += 1
                    else:
                        rospy.logwarn(f"✗ Pitch角度误差过大: 期望={pitch_deg:.1f}°, 实际={math.degrees(actual_pitch):.1f}°")
                
                if "gimbal_yaw_joint" in self.joint_states:
                    actual_yaw = self.joint_states["gimbal_yaw_joint"]
                    yaw_error = abs(actual_yaw - expected_yaw_rad)
                    
                    # 处理角度环绕
                    if yaw_error > math.pi:
                        yaw_error = 2 * math.pi - yaw_error
                    
                    if yaw_error < 0.1:
                        rospy.loginfo(f"✓ Yaw角度正确: 期望={yaw_deg:.1f}°, 实际={math.degrees(actual_yaw):.1f}°")
                        success_count += 1
                    else:
                        rospy.logwarn(f"✗ Yaw角度误差过大: 期望={yaw_deg:.1f}°, 实际={math.degrees(actual_yaw):.1f}°")
                
            except Exception as e:
                rospy.logerr(f"云台控制失败: {e}")
        
        success_rate = success_count / (len(test_angles) * 2) * 100
        rospy.loginfo(f"云台控制测试完成，成功率: {success_rate:.1f}%")
        return success_rate > 80
    
    def test_tf_transforms(self):
        """测试TF变换"""
        rospy.loginfo("=== 测试3: TF变换 ===")
        
        try:
            # 等待TF可用
            rospy.sleep(2.0)
            
            # 检查关键变换
            transforms_to_test = [
                ("base_link", "gimbal_base_link"),
                ("gimbal_base_link", "gimbal_yaw_link"),
                ("gimbal_yaw_link", "gimbal_pitch_link"),
                ("gimbal_pitch_link", "camera_link"),
                ("camera_link", "camera_optical_link"),
            ]
            
            success_count = 0
            
            for parent, child in transforms_to_test:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        parent, child, rospy.Time(), rospy.Duration(1.0)
                    )
                    rospy.loginfo(f"✓ TF变换存在: {parent} -> {child}")
                    success_count += 1
                except Exception as e:
                    rospy.logwarn(f"✗ TF变换失败: {parent} -> {child}, 错误: {e}")
            
            success_rate = success_count / len(transforms_to_test) * 100
            rospy.loginfo(f"TF变换测试完成，成功率: {success_rate:.1f}%")
            return success_rate > 80
            
        except Exception as e:
            rospy.logerr(f"TF测试异常: {e}")
            return False
    
    def test_image_with_gimbal_motion(self):
        """测试云台运动时的图像"""
        rospy.loginfo("=== 测试4: 云台运动图像测试 ===")
        
        if self.latest_image is None:
            rospy.logwarn("没有图像数据，跳过此测试")
            return False
        
        # 测试不同角度下的图像
        test_positions = [
            ("水平前视", 0, 0),
            ("向下30度", -30, 0),
            ("向下60度", -60, 0),
            ("右转90度", 0, 90),
            ("左转90度", 0, -90),
        ]
        
        for desc, pitch, yaw in test_positions:
            rospy.loginfo(f"测试位置: {desc}")
            
            # 控制云台
            req = gimbalControlRequest()
            req.pitch = pitch
            req.yaw = yaw
            req.roll = 0
            
            try:
                self.gimbal_srv(req)
                rospy.sleep(2.0)  # 等待运动完成
                
                # 检查图像
                if self.latest_image is not None:
                    # 在图像上添加信息
                    img_copy = self.latest_image.copy()
                    text = f"{desc}: P={pitch} Y={yaw}"
                    cv2.putText(img_copy, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.7, (0, 255, 0), 2)
                    
                    # 保存图像
                    filename = f"/tmp/gimbal_test_{desc.replace(' ', '_')}.jpg"
                    cv2.imwrite(filename, img_copy)
                    rospy.loginfo(f"✓ 图像已保存: {filename}")
                
            except Exception as e:
                rospy.logerr(f"测试位置 {desc} 失败: {e}")
        
        return True
    
    def run_all_tests(self):
        """运行所有测试"""
        rospy.loginfo("开始云台相机系统综合测试...")
        
        results = {}
        
        # 运行各项测试
        results["图像发布"] = self.test_image_publishing()
        results["云台控制"] = self.test_gimbal_control()
        results["TF变换"] = self.test_tf_transforms()
        results["运动图像"] = self.test_image_with_gimbal_motion()
        
        # 统计结果
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("测试结果汇总:")
        rospy.loginfo("="*50)
        
        passed = 0
        total = len(results)
        
        for test_name, result in results.items():
            status = "✓ 通过" if result else "✗ 失败"
            rospy.loginfo(f"{test_name:12s}: {status}")
            if result:
                passed += 1
        
        rospy.loginfo("="*50)
        rospy.loginfo(f"总体结果: {passed}/{total} 项测试通过")
        
        if passed == total:
            rospy.loginfo("🎉 所有测试通过！云台相机系统工作正常！")
        elif passed >= total * 0.75:
            rospy.logwarn("⚠️  大部分测试通过，系统基本可用")
        else:
            rospy.logerr("❌ 多项测试失败，请检查系统配置")

def main():
    try:
        tester = GimbalCameraSystemTester()
        rospy.sleep(2.0)  # 等待初始化完成
        tester.run_all_tests()
    except rospy.ROSInterruptException:
        rospy.loginfo("测试被用户中断")
    except Exception as e:
        rospy.logerr(f"测试异常: {e}")

if __name__ == "__main__":
    main()
