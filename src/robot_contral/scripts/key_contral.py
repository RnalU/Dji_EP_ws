#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import pygame
import sys
import math
from geometry_msgs.msg import Twist

class RobotKeyboardControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('robot_keyboard_control', anonymous=True)
        
        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 初始化pygame
        pygame.init()
        pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Robot Keyboard Control")
        
        # 控制参数
        self.max_linear_speed = 1.0   # 最大线速度 m/s
        self.max_angular_speed = 3.5  # 最大角速度 rad/s
        self.acceleration = 8.0       # 加速度
        self.deceleration = 12.0       # 减速度
        
        # 当前速度
        self.current_linear_x = 0.0
        self.current_linear_y = 0.0
        self.current_angular_z = 0.0
        
        # 目标速度
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        # 按键状态
        self.keys_pressed = {
            'w': False,  # 前进
            's': False,  # 后退
            'a': False,  # 左平移
            'd': False,  # 右平移
            'q': False,  # 左转
            'e': False,  # 右转
        }
        
        # 控制频率
        self.rate = rospy.Rate(50)  # 50Hz
        
        print("机器人键盘控制已启动!")
        print("控制说明:")
        print("W/S: 前进/后退")
        print("A/D: 左平移/右平移")
        print("Q/E: 左转/右转")
        print("ESC: 退出")
        print("请点击pygame窗口以获取焦点，然后使用按键控制")
    
    def smooth_velocity_update(self, dt):
        """平滑速度更新，实现加速减速效果"""
        # 更新线速度X（前进后退）
        if self.target_linear_x != self.current_linear_x:
            diff = self.target_linear_x - self.current_linear_x
            if abs(diff) > 0.01:
                if diff > 0:
                    self.current_linear_x += self.acceleration * dt
                    if self.current_linear_x > self.target_linear_x:
                        self.current_linear_x = self.target_linear_x
                else:
                    self.current_linear_x -= self.deceleration * dt
                    if self.current_linear_x < self.target_linear_x:
                        self.current_linear_x = self.target_linear_x
            else:
                self.current_linear_x = self.target_linear_x
        
        # 更新线速度Y（左右平移）
        if self.target_linear_y != self.current_linear_y:
            diff = self.target_linear_y - self.current_linear_y
            if abs(diff) > 0.01:
                if diff > 0:
                    self.current_linear_y += self.acceleration * dt
                    if self.current_linear_y > self.target_linear_y:
                        self.current_linear_y = self.target_linear_y
                else:
                    self.current_linear_y -= self.deceleration * dt
                    if self.current_linear_y < self.target_linear_y:
                        self.current_linear_y = self.target_linear_y
            else:
                self.current_linear_y = self.target_linear_y
        
        # 更新角速度Z（转向）
        if self.target_angular_z != self.current_angular_z:
            diff = self.target_angular_z - self.current_angular_z
            if abs(diff) > 0.01:
                if diff > 0:
                    self.current_angular_z += self.acceleration * dt
                    if self.current_angular_z > self.target_angular_z:
                        self.current_angular_z = self.target_angular_z
                else:
                    self.current_angular_z -= self.deceleration * dt
                    if self.current_angular_z < self.target_angular_z:
                        self.current_angular_z = self.target_angular_z
            else:
                self.current_angular_z = self.target_angular_z
    
    def update_target_velocity(self):
        """根据按键状态更新目标速度"""
        # 重置目标速度
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        
        # 前进后退
        if self.keys_pressed['w']:
            self.target_linear_x += self.max_linear_speed
        if self.keys_pressed['s']:
            self.target_linear_x -= self.max_linear_speed
        
        # 左右平移
        if self.keys_pressed['a']:
            self.target_linear_y -= self.max_linear_speed
        if self.keys_pressed['d']:
            self.target_linear_y += self.max_linear_speed
        
        # 左右转向
        if self.keys_pressed['q']:
            self.target_angular_z -= self.max_angular_speed
        if self.keys_pressed['e']:
            self.target_angular_z += self.max_angular_speed
    
    def publish_velocity(self):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = self.current_linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z
        
        self.cmd_vel_pub.publish(twist)
    
    def run(self):
        """主循环"""
        clock = pygame.time.Clock()
        
        while not rospy.is_shutdown():
            dt = clock.tick(50) / 1000.0  # 转换为秒
            
            # 处理pygame事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        return
                    elif event.key == pygame.K_w:
                        self.keys_pressed['w'] = True
                    elif event.key == pygame.K_s:
                        self.keys_pressed['s'] = True
                    elif event.key == pygame.K_a:
                        self.keys_pressed['a'] = True
                    elif event.key == pygame.K_d:
                        self.keys_pressed['d'] = True
                    elif event.key == pygame.K_q:
                        self.keys_pressed['q'] = True
                    elif event.key == pygame.K_e:
                        self.keys_pressed['e'] = True
                
                elif event.type == pygame.KEYUP:
                    if event.key == pygame.K_w:
                        self.keys_pressed['w'] = False
                    elif event.key == pygame.K_s:
                        self.keys_pressed['s'] = False
                    elif event.key == pygame.K_a:
                        self.keys_pressed['a'] = False
                    elif event.key == pygame.K_d:
                        self.keys_pressed['d'] = False
                    elif event.key == pygame.K_q:
                        self.keys_pressed['q'] = False
                    elif event.key == pygame.K_e:
                        self.keys_pressed['e'] = False
            
            # 更新目标速度
            self.update_target_velocity()
            
            # 平滑速度更新
            self.smooth_velocity_update(dt)
            
            # 发布速度命令
            self.publish_velocity()
            
            # 显示当前状态
            active_keys = [k for k, v in self.keys_pressed.items() if v]
            if active_keys:
                print(f"\r按键: {' '.join(active_keys)} | "
                      f"线速度: ({self.current_linear_x:.2f}, {self.current_linear_y:.2f}) | "
                      f"角速度: {self.current_angular_z:.2f}        ", end="")
            
            self.rate.sleep()
        
        # 退出时停止机器人
        self.stop_robot()
    
    def stop_robot(self):
        """停止机器人"""
        twist = Twist()
        for _ in range(10):  # 发送多次停止命令确保停止
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)
        print("\n机器人已停止")

if __name__ == '__main__':
    try:
        controller = RobotKeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        print("\n程序被中断")
    except KeyboardInterrupt:
        print("\n程序被手动中断")
    finally:
        pygame.quit()
        sys.exit()