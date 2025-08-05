#!/usr/bin/python3

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion

class DistanceController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('distance_controller', log_level=rospy.INFO)

        # 运动参数
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 30.0)
        self.default_speed = rospy.get_param("~default_speed", 0.7)

        # 控制参数
        self.tolerance_pos = 0.05  # 位置容差 (m)
        self.tolerance_angle = 2.0  # 角度容差 (度)
        self.control_rate = 20  # 控制频率 (Hz)

        # PID参数
        self.Kp_linear = 1.0
        self.Kp_angular = 1.0

        # 跟踪参数
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0

        # 状态标志
        self.is_moving = False
        self.lock = threading.Lock()

        # 创建发布者
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move_completed_pub = rospy.Publisher('/move_completed', Empty, queue_size=1)

        # 创建订阅者
        rospy.Subscriber('/move_distance', Float32MultiArray, self.move_distance_cb)
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        rospy.Subscriber('/set_speed', Float32MultiArray, self.set_speed_cb)

        # 定时控制循环
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_loop)

        rospy.loginfo("Distance Controller Initialized")

    def odom_cb(self, msg):
        """里程计回调函数"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 提取当前位置
        self.current_x = position.x
        self.current_y = position.y

        # 从四元数提取欧拉角
        (_, _, self.current_theta) = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_theta = math.degrees(self.current_theta)  # 转换为度

    def set_speed_cb(self, msg):
        """设置速度参数回调"""
        if len(msg.data) != 3:
            rospy.logwarn("Invalid set_speed command! Need 3 values: [linear_scale, angular_scale, default_speed]")
            return

        # 更新速度参数
        old_linear_scale = self.linear_scale
        old_angular_scale = self.angular_scale
        old_default_speed = self.default_speed

        self.linear_scale = msg.data[0] if msg.data[0] > 0 else self.linear_scale
        self.angular_scale = msg.data[1] if msg.data[1] > 0 else self.angular_scale
        self.default_speed = msg.data[2] if msg.data[2] > 0 else self.default_speed

        rospy.loginfo(f"Speed parameters updated - Linear Scale: {old_linear_scale} -> {self.linear_scale}, "
                      f"Angular Scale: {old_angular_scale} -> {self.angular_scale}, "
                      f"Default Speed: {old_default_speed} -> {self.default_speed}")

    def move_distance_cb(self, msg):
        """位移控制回调"""
        if len(msg.data) < 3:
            rospy.logwarn("Invalid move_distance command!")
            return

        with self.lock:
            if self.is_moving:
                rospy.logwarn("Robot is already moving!")
                return

            self.is_moving = True

            # 获取当前位置作为起点
            start_x = self.current_x
            start_y = self.current_y
            start_theta = self.current_theta

            # 计算目标位置
            dx, dy, dtheta = msg.data[0], msg.data[1], msg.data[2]

            # 计算目标位置（全局坐标系）
            theta_rad = math.radians(start_theta)
            self.target_x = start_x + dx * math.cos(theta_rad) - dy * math.sin(theta_rad)
            self.target_y = start_y + dx * math.sin(theta_rad) + dy * math.cos(theta_rad)
            self.target_theta = start_theta + dtheta

            # 标准化目标角度到[-180, 180]
            self.target_theta = ((self.target_theta + 180) % 360) - 180

            rospy.loginfo(f"Starting move from ({start_x:.2f}, {start_y:.2f}, {start_theta:.2f}) "
                          f"to ({self.target_x:.2f}, {self.target_y:.2f}, {self.target_theta:.2f})")

    def control_loop(self, event):
        """控制循环，由定时器调用"""
        if not self.is_moving:
            return

        with self.lock:
            # 计算误差
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y

            # 计算当前位置到目标位置的距离
            distance = math.sqrt(dx*dx + dy*dy)

            # 计算角度误差 (标准化到[-180, 180])
            angle_error = self.target_theta - self.current_theta
            angle_error = ((angle_error + 180) % 360) - 180

            # 检查是否已到达目标
            if distance < self.tolerance_pos and abs(angle_error) < self.tolerance_angle:
                self.send_zero_velocity()
                self.is_moving = False
                self.move_completed_pub.publish(Empty())
                rospy.loginfo("Target reached!")
                return

            # 计算机器人当前朝向与目标点连线的夹角
            target_heading = math.atan2(dy, dx)
            current_heading = math.radians(self.current_theta)
            heading_error = target_heading - current_heading

            # 标准化到[-pi, pi]
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            # 控制命令
            cmd = Twist()

            # 如果方向对准，优先移动到目标位置
            if distance > self.tolerance_pos:
                if abs(math.degrees(heading_error)) < 10:
                    # 朝向基本正确，前进
                    linear_vel = min(self.default_speed, self.Kp_linear * distance)
                    cmd.linear.x = linear_vel
                    cmd.angular.z = self.Kp_angular * heading_error  # 同时微调朝向
                else:
                    # 先调整朝向
                    cmd.angular.z = self.Kp_angular * heading_error
            else:
                # 已到达位置，调整最终朝向
                cmd.angular.z = math.radians(self.Kp_angular * angle_error / 10.0)

            # 发布速度命令
            self.cmd_vel_pub.publish(cmd)

    def send_zero_velocity(self):
        """发送零速度命令"""
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        self.cmd_vel_pub.publish(cmd)

    def shutdown(self):
        """关闭节点时的清理"""
        rospy.loginfo("Shutting down distance controller...")
        self.send_zero_velocity()
        self.control_timer.shutdown()

if __name__ == '__main__':
    try:
        controller = DistanceController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
