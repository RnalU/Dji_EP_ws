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
        
        # PID控制参数
        self.kp_linear = 2.0 
        self.ki_linear = 0.02
        self.kd_linear = 0.00
        
        self.kp_angular = 2.5
        self.ki_angular = 0.01
        self.kd_angular = 0.10

        # 误差积分和前一次误差
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0

        # 其他控制参数
        self.stage = 0  # 控制阶段：0:平移  1:旋转
        self.tolerance_pos = 0.05       # 位置容差 （m）
        self.tolerance_angle = 0.5      # 角度容差 （°）
        self.control_rate = 20          # 控制频率 (Hz) 

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
        else:
            rospy.loginfo(f"Received move_distance command: {msg.data}")

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

            rospy.loginfo(f"Starting move from ({start_x:.2f}, {start_y:.2f}, {start_theta:.2f}) "
                          f"to ({self.target_x:.2f}, {self.target_y:.2f}, {self.final_theta:.2f})")

    def move_to_home(self, event=None):
        """根据绝对坐标记录的值回到原点"""
        with self.lock:
            if self.is_moving:
                rospy.logwarn("Robot is already moving!")
                return

            # 计算回到原点(1,2.3,0)所需的移动距离和角度
            dx = 1-self.absolute_x
            dy = 2.3-self.absolute_y
            dtheta = 0-self.absolute_theta

            # 创建移动指令 
            move_cmd = Float32MultiArray()
            move_cmd.data = [dx, dy, dtheta]

            rospy.loginfo(f"Moving back to home point (1, 2.3, 0) from current position "
                         f"({dx:.2f}, {dy:.2f}, {dtheta:.2f})")

            # 发布移动指令
            self.move_distance_cb(move_cmd)

    def control_loop(self, event):
        """控制循环，由定时器调用"""
        with self.lock:
            if not self.is_moving:
                return

            # 计算到目标的距离和角度
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            distance = math.sqrt(dx * dx + dy * dy)
            
            # 小车当前朝向 弧度
            current_theta_rad = math.radians(self.current_theta)
            
            # 先平移后旋转
            cmd = Twist()
            
            #  直接平移到目标位置
            if self.stage == 0:
                if distance > self.tolerance_pos:
                    # 计算全局坐标系中的方向向量
                    direction_x = dx / distance if distance > 0 else 0
                    direction_y = dy / distance if distance > 0 else 0
                    
                    # 将全局坐标系的方向向量转换到机器人坐标系
                    robot_dir_x = direction_x * math.cos(-current_theta_rad) - direction_y * math.sin(-current_theta_rad)
                    robot_dir_y = direction_x * math.sin(-current_theta_rad) + direction_y * math.cos(-current_theta_rad)
                    
                    # PID控制线速度
                    self.linear_error_integral += distance
                    linear_derivative = distance - self.prev_linear_error
                    self.prev_linear_error = distance
                    
                    # 限制积分项大小防止积分饱和
                    max_integral = 1.0
                    self.linear_error_integral = max(-max_integral, min(max_integral, self.linear_error_integral))
                    
                    speed_scale = self.kp_linear * distance + self.ki_linear * self.linear_error_integral + self.kd_linear * linear_derivative
                    
                    # 根据距离目标点的远近调整速度，近处减速
                    speed = min(self.default_speed, speed_scale)
                    
                    # 确保至少有最小速度
                    min_speed = 0.05
                    speed = max(min_speed, speed)
                    
                    # 设置线速度分量
                    cmd.linear.x = speed * robot_dir_x * self.linear_scale
                    cmd.linear.y = speed * robot_dir_y * self.linear_scale
                    
                    # rospy.loginfo(f"Stage: Translate - Distance: {distance:.2f}m, Speed: {speed:.2f}m/s, " +
                    #               f"Vector: ({cmd.linear.x:.2f}, {cmd.linear.y:.2f})")
                else:
                    # 位置调整完成，进入旋转调整阶段
                    self.stage = 1
                    self.linear_error_integral = 0
                    self.prev_linear_error = 0
                    rospy.loginfo("Reached target position. Switching to rotation stage")
            
            # 调整到最终朝向
            elif self.stage == 1:
                # 计算最终朝向与当前朝向的角度差
                angle_error = self.final_theta - self.current_theta
                # 标准化到[-180, 180]
                angle_error = ((angle_error + 180) % 360) - 180
                
                # rospy.loginfo(f"Stage: Rotate - Error: {angle_error:.2f}°, Target: {self.final_theta:.2f}°")
                
                if abs(angle_error) > self.tolerance_angle:
                    # PID控制角速度
                    self.angular_error_integral += angle_error
                    angular_derivative = angle_error - self.prev_angular_error
                    self.prev_angular_error = angle_error
                    
                    # 限制积分项大小防止积分饱和
                    max_integral = 10.0
                    self.angular_error_integral = max(-max_integral, min(max_integral, self.angular_error_integral))
                    
                    angular_output = (self.kp_angular * angle_error + 
                                     self.ki_angular * self.angular_error_integral + 
                                     self.kd_angular * angular_derivative)
                    
                    # 限制最大角速度
                    max_angular_speed = self.angular_scale * (math.pi/180.0)
                    cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, angular_output * (math.pi/180.0)))
                else:
                    # 运动完成
                    rospy.loginfo("Movement completed successfully!")
                    self.is_moving = False
                    self.send_zero_velocity()
                    self.move_completed_pub.publish(Empty())
                    self.stage = 0
                    self.linear_error_integral = 0.0
                    self.angular_error_integral = 0.0
                    self.prev_linear_error = 0.0
                    self.prev_angular_error = 0.0
                    return
            
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
        # 重置控制状态
        self.is_moving = False
        self.linear_error_integral = 0.0
        self.angular_error_integral = 0.0
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        if hasattr(self, 'stage'):
            del self.stage
        self.send_zero_velocity()
        self.control_timer.shutdown()

if __name__ == '__main__':
    try:
        controller = DistanceController()
        rospy.on_shutdown(controller.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
