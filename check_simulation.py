#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
仿真状态检查脚本
检查无人机和小车的仿真状态，验证物理配置是否正确
"""

import rospy
import time
import numpy as np
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_geometry_msgs import do_transform_pose


class SimulationChecker:
    def __init__(self):
        rospy.init_node('simulation_checker', anonymous=False)
        
        # 状态变量
        self.uav_state = None
        self.uav_extended_state = None
        self.uav_position = None
        self.uav_imu = None
        self.smartcar_position = None
        self.model_states = None
        
        # 订阅器
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.uav_state_sub = rospy.Subscriber('/uav1/mavros/state', State, self.uav_state_callback)
        self.uav_extended_state_sub = rospy.Subscriber('/uav1/mavros/extended_state', ExtendedState, self.uav_extended_state_callback)
        self.uav_imu_sub = rospy.Subscriber('/uav1/mavros/imu/data', Imu, self.uav_imu_callback)
        self.uav_local_pos_sub = rospy.Subscriber('/uav1/mavros/local_position/pose', Odometry, self.uav_local_position_callback)
        
        # 发布器（用于测试）
        self.smartcar_cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy('/uav1/mavros/cmd/takeoff', CommandTOL)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.loginfo("仿真检查器已启动")
        
    def model_states_callback(self, msg):
        """处理模型状态消息"""
        self.model_states = msg
        
        # 查找无人机位置
        try:
            uav_names = [name for name in msg.name if 'p230' in name]
            if uav_names:
                uav_idx = msg.name.index(uav_names[0])
                self.uav_position = msg.pose[uav_idx]
        except ValueError:
            pass
            
        # 查找小车位置
        try:
            if 'smartcar' in msg.name:
                car_idx = msg.name.index('smartcar')
                self.smartcar_position = msg.pose[car_idx]
        except ValueError:
            pass
    
    def uav_state_callback(self, msg):
        """处理无人机状态消息"""
        self.uav_state = msg
    
    def uav_extended_state_callback(self, msg):
        """处理无人机扩展状态消息"""
        self.uav_extended_state = msg
    
    def uav_imu_callback(self, msg):
        """处理IMU消息"""
        self.uav_imu = msg
    
    def uav_local_position_callback(self, msg):
        """处理本地位置消息"""
        pass
    
    def check_models_physics(self):
        """检查模型物理状态"""
        print("\n=== 模型物理状态检查 ===")
        
        if self.model_states is None:
            print("❌ 无法获取模型状态信息")
            return False
            
        print(f"✅ 检测到 {len(self.model_states.name)} 个模型:")
        for i, name in enumerate(self.model_states.name):
            pose = self.model_states.pose[i]
            twist = self.model_states.twist[i]
            print(f"   {name}:")
            print(f"     位置: x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}")
            print(f"     速度: vx={twist.linear.x:.3f}, vy={twist.linear.y:.3f}, vz={twist.linear.z:.3f}")
        
        return True
    
    def check_uav_position(self):
        """检查无人机位置"""
        print("\n=== 无人机位置检查 ===")
        
        if self.uav_position is None:
            print("❌ 无法获取无人机位置")
            return False
        
        if self.smartcar_position is None:
            print("❌ 无法获取小车位置")
            return False
            
        # 计算相对位置
        dx = self.uav_position.position.x - self.smartcar_position.position.x
        dy = self.uav_position.position.y - self.smartcar_position.position.y
        dz = self.uav_position.position.z - self.smartcar_position.position.z
        
        print(f"✅ 无人机位置: x={self.uav_position.position.x:.3f}, y={self.uav_position.position.y:.3f}, z={self.uav_position.position.z:.3f}")
        print(f"✅ 小车位置: x={self.smartcar_position.position.x:.3f}, y={self.smartcar_position.position.y:.3f}, z={self.smartcar_position.position.z:.3f}")
        print(f"✅ 相对位置: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        
        # 检查是否在停机坪上方
        if abs(dx) < 0.15 and abs(dy) < 0.20 and dz > 0.15 and dz < 0.5:
            print("✅ 无人机正确位于小车停机坪上方")
            return True
        else:
            print("⚠️  无人机位置可能不正确")
            return False
    
    def check_uav_connection(self):
        """检查无人机连接状态"""
        print("\n=== 无人机连接检查 ===")
        
        if self.uav_state is None:
            print("❌ 无法获取无人机状态")
            return False
            
        print(f"✅ 连接状态: {'已连接' if self.uav_state.connected else '未连接'}")
        print(f"✅ 解锁状态: {'已解锁' if self.uav_state.armed else '未解锁'}")
        print(f"✅ 飞行模式: {self.uav_state.mode}")
        
        if self.uav_extended_state:
            vtol_states = {
                0: "未定义",
                1: "过渡到前飞",
                2: "过渡到多旋翼",
                3: "多旋翼",
                4: "前飞"
            }
            landed_states = {
                0: "未定义",
                1: "在地面",
                2: "在空中",
                3: "起飞中",
                4: "降落中"
            }
            
            vtol_state = vtol_states.get(self.uav_extended_state.vtol_state, "未知")
            landed_state = landed_states.get(self.uav_extended_state.landed_state, "未知")
            
            print(f"✅ VTOL状态: {vtol_state}")
            print(f"✅ 着陆状态: {landed_state}")
        
        return self.uav_state.connected
    
    def check_imu_data(self):
        """检查IMU数据"""
        print("\n=== IMU数据检查 ===")
        
        if self.uav_imu is None:
            print("❌ 无法获取IMU数据")
            return False
            
        # 检查加速度计
        accel = self.uav_imu.linear_acceleration
        accel_magnitude = np.sqrt(accel.x**2 + accel.y**2 + accel.z**2)
        
        print(f"✅ 线性加速度: x={accel.x:.3f}, y={accel.y:.3f}, z={accel.z:.3f}")
        print(f"✅ 加速度幅值: {accel_magnitude:.3f} m/s² (重力应该约为9.8)")
        
        # 检查角速度
        gyro = self.uav_imu.angular_velocity
        print(f"✅ 角速度: x={gyro.x:.3f}, y={gyro.y:.3f}, z={gyro.z:.3f} rad/s")
        
        # 检查姿态
        orient = self.uav_imu.orientation
        print(f"✅ 姿态四元数: x={orient.x:.3f}, y={orient.y:.3f}, z={orient.z:.3f}, w={orient.w:.3f}")
        
        # 简单的健康检查
        if 8.0 < accel_magnitude < 11.0:
            print("✅ IMU数据看起来正常")
            return True
        else:
            print("⚠️  IMU数据可能异常")
            return False
    
    def test_smartcar_movement(self):
        """测试小车运动"""
        print("\n=== 小车运动测试 ===")
        
        if self.smartcar_position is None:
            print("❌ 无法获取小车初始位置")
            return False
            
        initial_pos = self.smartcar_position.position
        print(f"✅ 小车初始位置: x={initial_pos.x:.3f}, y={initial_pos.y:.3f}, z={initial_pos.z:.3f}")
        
        # 发送前进命令
        cmd = Twist()
        cmd.linear.x = 0.2  # 0.2 m/s 前进
        cmd.angular.z = 0.0
        
        print("🚗 发送前进命令...")
        for _ in range(20):  # 发送2秒
            self.smartcar_cmd_pub.publish(cmd)
            rospy.sleep(0.1)
        
        # 停止
        cmd.linear.x = 0.0
        self.smartcar_cmd_pub.publish(cmd)
        
        rospy.sleep(1.0)  # 等待1秒
        
        if self.smartcar_position:
            final_pos = self.smartcar_position.position
            distance_moved = np.sqrt(
                (final_pos.x - initial_pos.x)**2 + 
                (final_pos.y - initial_pos.y)**2
            )
            print(f"✅ 小车最终位置: x={final_pos.x:.3f}, y={final_pos.y:.3f}, z={final_pos.z:.3f}")
            print(f"✅ 移动距离: {distance_moved:.3f} m")
            
            if distance_moved > 0.1:
                print("✅ 小车运动正常")
                return True
            else:
                print("❌ 小车可能无法正常运动")
                return False
        
        return False
    
    def check_height_estimation(self):
        """检查高度估计"""
        print("\n=== 高度估计检查 ===")
        
        # 检查是否有必要的话题
        topics = rospy.get_published_topics()
        topic_names = [topic[0] for topic in topics]
        
        required_topics = [
            '/uav1/mavros/local_position/pose',
            '/uav1/mavros/global_position/local',
            '/uav1/mavros/altitude'
        ]
        
        for topic in required_topics:
            if topic in topic_names:
                print(f"✅ 话题存在: {topic}")
            else:
                print(f"⚠️  话题缺失: {topic}")
        
        # 检查视觉位置估计
        vision_topics = [topic for topic in topic_names if 'vision' in topic]
        if vision_topics:
            print("✅ 检测到视觉定位话题:")
            for topic in vision_topics:
                print(f"   {topic}")
        else:
            print("⚠️  未检测到视觉定位话题")
        
        return True
    
    def run_full_check(self):
        """运行完整检查"""
        print("🔍 开始仿真状态检查...")
        print("=" * 50)
        
        # 等待数据
        print("⏳ 等待数据...")
        rospy.sleep(3.0)
        
        results = []
        
        # 执行各项检查
        results.append(("模型物理状态", self.check_models_physics()))
        results.append(("无人机位置", self.check_uav_position()))
        results.append(("无人机连接", self.check_uav_connection()))
        results.append(("IMU数据", self.check_imu_data()))
        results.append(("高度估计", self.check_height_estimation()))
        results.append(("小车运动", self.test_smartcar_movement()))
        
        # 输出总结
        print("\n" + "=" * 50)
        print("🏁 检查结果总结:")
        print("=" * 50)
        
        passed = 0
        total = len(results)
        
        for name, result in results:
            status = "✅ 通过" if result else "❌ 失败"
            print(f"{name}: {status}")
            if result:
                passed += 1
        
        print(f"\n总体评分: {passed}/{total} ({passed/total*100:.1f}%)")
        
        if passed == total:
            print("🎉 所有检查都通过！仿真环境配置正确。")
        elif passed >= total * 0.7:
            print("⚠️  大部分检查通过，但有一些问题需要注意。")
        else:
            print("❌ 检查失败较多，建议检查仿真配置。")
        
        return passed >= total * 0.7


if __name__ == '__main__':
    try:
        checker = SimulationChecker()
        rospy.sleep(1.0)  # 给订阅器一些时间初始化
        success = checker.run_full_check()
        
        if success:
            rospy.loginfo("仿真检查完成 - 状态良好")
        else:
            rospy.logwarn("仿真检查完成 - 发现问题")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("仿真检查被中断")
    except Exception as e:
        rospy.logerr(f"仿真检查失败: {e}")
