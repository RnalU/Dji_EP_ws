#!/usr/bin/env python3
import rospy
import math
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty
from robomaster import robot, led

class EPRobotDriver:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('ep_robot_driver', log_level=rospy.INFO)
        
        # 机器人连接参数
        self.sn = rospy.get_param("~sn", "3JKDH5D00169PE")
        self.protocol = rospy.get_param("~protocol", "udp")
        
        # 运动参数
        self.linear_scale = rospy.get_param("~linear_scale", 0.5)
        self.angular_scale = rospy.get_param("~angular_scale", 30.0)
        self.default_speed = rospy.get_param("~default_speed", 0.7)
        
        # 初始化机器人
        self._init_robot()
        
        # ROS订阅者
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/move_distance", Float32MultiArray, self.move_distance_cb)
        rospy.Subscriber("/wheel_speeds", Float32MultiArray, self.wheel_speeds_cb)
        rospy.Subscriber("/set_speed", Float32MultiArray, self.set_speed_cb)  # 添加速度设置订阅者
        
        # ROS发布者
        self.move_completed_pub = rospy.Publisher('/move_completed', Empty, queue_size=1)

        # ROS服务
        # self.stop_service = rospy.Service('~emergency_stop', Trigger, self.emergency_stop)
        
        # 状态跟踪
        self.is_moving = False
        self.lock = threading.Lock()
        
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("EP Robot Driver Initialized")

    def _init_robot(self):
        """初始化机器人连接"""
        try:
            self.robot = robot.Robot()
            self.robot.initialize(conn_type="sta")
            
            # 初始化模块
            self.chassis = self.robot.chassis
            self.led = self.robot.led
            self.battery = self.robot.battery
            
            # 设置LED状态
            self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
            rospy.loginfo(f"Robot connected! SN: {self.robot.get_sn()}")
            rospy.loginfo(f"Set Params - Linear Scale: {self.linear_scale}, Angular Scale: {self.angular_scale}, Default Speed: {self.default_speed}")
            
        except Exception as e:
            rospy.logerr(f"Robot initialization failed: {str(e)}")
            rospy.signal_shutdown("Robot connection error")

    def cmd_vel_cb(self, msg):
        """速度控制回调"""
        if self.is_moving:
            return
            
        x = msg.linear.x * self.linear_scale
        y = msg.linear.y * self.linear_scale
        z = msg.angular.z * self.angular_scale
        
        with self.lock:
            self.chassis.drive_speed(x=x, y=y, z=z, timeout=0.1)
            rospy.logdebug(f"Set velocity: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def move_distance_cb(self, msg):
        """位移控制回调"""
        if len(msg.data) < 3:
            rospy.logwarn("Invalid move_distance command!")
            return
            
        if self.is_moving:
            rospy.logwarn("Robot is already moving!")
            return
            
        self.is_moving = True
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        
        try:
            # 设置运动LED (蓝色)
            self.led.set_led(comp=led.COMP_ALL, r=0, g=0, b=255, effect=led.EFFECT_SCROLLING)
            
            # 执行位移运动
            self.chassis.move(x=x, y=y, z=z, 
                            xy_speed=self.default_speed, 
                            z_speed=self.angular_scale).wait_for_completed()
                            
            rospy.loginfo(f"Moved: x={x}m, y={y}m, z={z}deg")
            
            # 发布移动完成消息
            self.move_completed_pub.publish(Empty())
            
        except Exception as e:
            rospy.logerr(f"Movement failed: {str(e)}")
        finally:
            # 恢复LED状态
            self.led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_BREATH)
            self.is_moving = False

    def wheel_speeds_cb(self, msg):
        """麦轮单独控制"""
        if len(msg.data) != 4:
            rospy.logwarn("Need 4 wheel speeds!")
            return
            
        w1, w2, w3, w4 = msg.data
        with self.lock:
            self.chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4)
            rospy.logdebug(f"Set wheel speeds: {w1}, {w2}, {w3}, {w4}")

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

    def shutdown(self):
        """关闭节点时的清理"""
        rospy.loginfo("Shutting down...")
        with self.lock:
            self.chassis.drive_speed(x=0, y=0, z=0)
            self.led.set_led(comp=led.COMP_ALL, r=255, g=0, b=0, effect=led.EFFECT_ON)
            self.robot.close()
            

if __name__ == '__main__':
    try:
        driver = EPRobotDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass