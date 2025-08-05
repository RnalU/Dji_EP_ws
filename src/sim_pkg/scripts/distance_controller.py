#!/usr/bin/python3

import rospy
import math
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class DistanceController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('distance_controller', log_level=rospy.INFO)

        # 运动参数
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 90.0)
        self.default_speed = rospy.get_param("~default_speed", 0.7)

        # 控制参数
        self.tolerance_pos = 0.05  # 位置容差 (m)
        self.tolerance_angle = 0.5  # 角度容差 (度)
        self.control_rate = 50  # 控制频率 (Hz) - 提高控制频率

        # PID参数
        self.Kp_linear = 1.0
        self.Ki_linear = 0.01
        self.Kd_linear = 0.0

        self.Kp_angular = 1.5         
        self.Ki_angular = 0.01      
        self.Kd_angular = 0.0      

        # 积分和微分项
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        # 速度平滑参数
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0
        self.accel_limit = 0.5  # 线速度加速度限制 (m/s²)
        self.angular_accel_limit = 2  # 角速度加速度限制 (rad/s²)

        # 跟踪参数
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = 0.0
        self.final_theta = 0.0
        self.need_to_reset_theta = False

        # 绝对坐标
        self.absolute_x = 0.0
        self.absolute_y = 0.0
        self.absolute_theta = 0.0

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
        rospy.Subscriber('/back_homepoint', Empty, self.move_to_home)  # 用于回到原点

        # 定时控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0, int(1000000000.0/float(self.control_rate))), self.control_loop)

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

        # 更新绝对坐标
        self.absolute_x = position.x
        self.absolute_y = position.y
        self.absolute_theta = self.current_theta

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
            self.final_theta = start_theta + dtheta

            rospy.loginfo("final_theta: {}".format(self.final_theta))
            # 标准化目标角度到[-180, 180]
            self.final_theta = ((self.final_theta + 180) % 360) - 180

            # 设定小车临时目标朝向
            target_heading = math.atan2(dy, dx) * 57.3
            current_heading = math.radians(self.current_theta) * 57.3
            self.target_theta = (target_heading - current_heading)
            rospy.loginfo("target_heading: {}, current_heading: {}, target_theta: {}".format(target_heading, current_heading, self.target_theta))

            self.target_theta = ((self.target_theta + 180) % 360) - 180

            rospy.loginfo(f"Starting move from ({start_x:.2f}, {start_y:.2f}, {start_theta:.2f}) "
                          f"to ({self.target_x:.2f}, {self.target_y:.2f}, {self.final_theta:.2f})"
                          f" with angle {self.target_theta:.2f}°")

    def move_to_home(self, event=None):
        """根据绝对坐标记录的值回到原点，无速度上限要求"""
        with self.lock:
            if self.is_moving:
                rospy.logwarn("Robot is already moving!")
                return

            # 计算回到原点(1,2.3,0)所需的移动距离和角度
            dx = 1-self.absolute_x
            dy = 2.3-self.absolute_y
            dtheta = 0-self.absolute_theta

            # 创建移动指令 - 格式与move_distance_cb期望的一致
            move_cmd = Float32MultiArray()
            move_cmd.data = [dx, dy, dtheta]

            rospy.loginfo(f"Moving back to home point (1, 2.3, 0) from current position "
                         f"({dx:.2f}, {dy:.2f}, {dtheta:.2f})")

            # 发布移动指令
            self.move_distance_cb(move_cmd)

    def control_loop(self, event):
        """控制循环，由定时器调用"""
        if not self.is_moving:
            return

        dt = 1.0/self.control_rate  # 控制周期时间

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
            if distance < self.tolerance_pos and not self.need_to_reset_theta:
                self.send_zero_velocity()
                # self.is_moving = False
                self.need_to_reset_theta = True
                rospy.loginfo("Target reached!"
                              f"Now fix the final orientation. {self.target_theta}->{self.final_theta}")

                # 设置目标朝向
                self.target_theta = self.final_theta

                # 重新计算角度误差
                angle_error = self.target_theta - self.current_theta
                angle_error = ((angle_error + 180) % 360) - 180

            # 检查是否已经到达目标朝向
            if abs(angle_error) < self.tolerance_angle and self.need_to_reset_theta:
                # 到达目标朝向，停止运动
                self.send_zero_velocity()
                self.is_moving = False
                self.need_to_reset_theta = False
                rospy.loginfo("Final orientation reached. Stopping movement.")

                # 重置积分项
                self.linear_error_integral = 0.0
                self.angular_error_integral = 0.0
                return

            # 计算机器人当前朝向与目标点连线的夹角
            target_heading = math.atan2(dy, dx)
            current_heading = math.radians(self.current_theta)
            heading_error = target_heading - current_heading

            # 标准化到[-pi, pi]
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

            # 平滑控制策略 - 使用sigmoid函数实现平滑过渡
            heading_factor = 1.0 / (1.0 + math.exp(0.5 * abs(math.degrees(heading_error)) - 5))

            # 更新PID控制
            # 线速度PID
            self.linear_error_integral += distance * dt
            linear_error_derivative = (distance - self.prev_linear_error) / dt
            self.prev_linear_error = distance

            # 角速度PID
            self.angular_error_integral += heading_error * dt
            angular_error_derivative = (heading_error - self.prev_angular_error) / dt
            self.prev_angular_error = heading_error

            # 控制命令
            cmd = Twist()

            # 计算目标线速度和角速度
            target_linear_vel = 0.0
            target_angular_vel = 0.0

            if distance > self.tolerance_pos:
                # 使用平滑因子计算线速度 - 方向误差越大，线速度越小
                p_term = self.Kp_linear * distance
                i_term = self.Ki_linear * self.linear_error_integral
                d_term = self.Kd_linear * linear_error_derivative

                base_linear_vel = min(self.default_speed, p_term + i_term + d_term)
                target_linear_vel = base_linear_vel * heading_factor

                # 角速度控制 - 始终进行方向修正
                p_term = self.Kp_angular * heading_error * 2
                i_term = self.Ki_angular * self.angular_error_integral
                d_term = self.Kd_angular * angular_error_derivative

                target_angular_vel = p_term + i_term + d_term
            else:
                # 已到达位置，只调整最终朝向
                angle_error_rad = math.radians(angle_error)
                target_angular_vel = self.Kp_angular * angle_error_rad * 2

            # 速度平滑处理 - 限制加速度
            linear_accel = (target_linear_vel - self.last_linear_vel) / dt
            if abs(linear_accel) > self.accel_limit:
                linear_accel = math.copysign(self.accel_limit, linear_accel)
                target_linear_vel = self.last_linear_vel + linear_accel * dt

            angular_accel = (target_angular_vel - self.last_angular_vel) / dt
            if abs(angular_accel) > self.angular_accel_limit:
                angular_accel = math.copysign(self.angular_accel_limit, angular_accel)
                target_angular_vel = self.last_angular_vel + angular_accel * dt

            # 更新速度记录
            self.last_linear_vel = target_linear_vel
            self.last_angular_vel = target_angular_vel

            # 设置最终速度命令
            cmd.linear.x = target_linear_vel
            cmd.angular.z = target_angular_vel

            # 发布速度命令
            self.cmd_vel_pub.publish(cmd)

            # 调试信息
            if rospy.get_param("~debug", False):
                rospy.loginfo(f"Distance: {distance:.3f}, Heading error: {math.degrees(heading_error):.2f}°, "
                              f"Lin vel: {cmd.linear.x:.3f}, Ang vel: {cmd.angular.z:.3f}")

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
        # 重置控制状态
        self.is_moving = False
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.send_zero_velocity()
        self.control_timer.shutdown()

if __name__ == '__main__':
    try:
        controller = DistanceController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
