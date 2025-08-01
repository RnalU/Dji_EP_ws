#!/usr/bin/env python3
import rospy
import math
import threading
import actionlib
from enum import Enum, auto
from actionlib import SimpleActionServer
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty, Bool
from robomaster import robot, led
from ep_start.msg import MissionAction, MissionFeedback, MissionResult  # 需要自定义Action消息

class MissionState(Enum):
    """任务状态枚举"""
    IDLE = auto()
    STARTING = auto()
    MOVING_TO_POINT1 = auto()
    LAUNCHING_DRONE = auto()
    MOVING_TO_POINT2 = auto()
    WAITING_FOR_DRONE = auto()
    MOVING_TO_CHARGE = auto()
    CHARGING = auto()
    RETURNING_HOME = auto()
    COMPLETED = auto()
    FAILED = auto()

class MissionController:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('mission_controller', log_level=rospy.INFO)
        
        # 任务参数
        point1_param = rospy.get_param("~point1", "[2.0, 0.0, 0.0]")  # [x, y, z]
        point2_param = rospy.get_param("~point2", "[5.0, 0.0, 0.0]")
        charge_point_param = rospy.get_param("~charge_point", "[0.0, 2.0, 0.0]")
        home_point_param = rospy.get_param("~home_point", "[0.0, 0.0, 0.0]")
        
        # 解析参数为浮点数列表
        import ast
        self.point1 = [float(x) for x in ast.literal_eval(point1_param)] if isinstance(point1_param, str) else [float(x) for x in point1_param]
        self.point2 = [float(x) for x in ast.literal_eval(point2_param)] if isinstance(point2_param, str) else [float(x) for x in point2_param]
        self.charge_point = [float(x) for x in ast.literal_eval(charge_point_param)] if isinstance(charge_point_param, str) else [float(x) for x in charge_point_param]
        self.home_point = [float(x) for x in ast.literal_eval(home_point_param)] if isinstance(home_point_param, str) else [float(x) for x in home_point_param]
        
        self.charging_time = float(rospy.get_param("~charging_time", 30.0))  # 秒
        
        # 状态跟踪
        self.current_state = MissionState.IDLE
        self.last_state_change = rospy.Time.now()
        self.state_timeout = rospy.Duration(60)  # 默认状态超时时间
        
        # Action服务器
        self.action_server = SimpleActionServer(
            'mission_control', 
            MissionAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        self.action_server.start()
        
        # 发布器
        self.drone_cmd_pub = rospy.Publisher('/drone/command', Empty, queue_size=1)
        self.move_pub = rospy.Publisher('/move_distance', Float32MultiArray, queue_size=1)
        
        # 订阅器
        rospy.Subscriber('/drone/status', Bool, self.drone_status_cb)
        rospy.Subscriber('/move_completed', Empty, self.move_completed_cb)
        
        # 无人机状态
        self.drone_landed = False
        self.move_completed = False
        
        rospy.loginfo("Mission Controller Initialized")
    
    def execute_cb(self, goal):
        """Action服务器回调函数"""
        rospy.loginfo("Starting mission...")
        result = MissionResult()
        
        try:
            # 重置状态
            self.current_state = MissionState.STARTING
            self.drone_landed = False
            self.move_completed = False
            
            # 主任务循环
            rate = rospy.Rate(10)  # 10Hz
            while not rospy.is_shutdown() and not self.action_server.is_preempt_requested():
                # 处理当前状态
                self.handle_state()
                
                # 检查任务完成
                if self.current_state == MissionState.COMPLETED:
                    rospy.loginfo("Mission completed successfully!")
                    result.success = True
                    self.action_server.set_succeeded(result)
                    return
                
                # 检查任务失败
                if self.current_state == MissionState.FAILED:
                    rospy.logerr("Mission failed!")
                    result.success = False
                    self.action_server.set_aborted(result)
                    return
                
                # 发送反馈
                feedback = MissionFeedback()
                feedback.current_state = self.current_state.name
                feedback.progress = self.get_progress()
                self.action_server.publish_feedback(feedback)
                
                rate.sleep()
                
            # 处理任务被抢占
            if self.action_server.is_preempt_requested():
                rospy.logwarn("Mission preempted!")
                self.action_server.set_preempted()
                self.transition_state(MissionState.FAILED)
        
        except Exception as e:
            rospy.logerr(f"Mission execution failed: {str(e)}")
            result.success = False
            self.action_server.set_aborted(result)
    
    def handle_state(self):
        """处理当前状态"""
        # 检查状态超时
        if (rospy.Time.now() - self.last_state_change) > self.state_timeout:
            rospy.logwarn(f"State {self.current_state.name} timed out!")
            self.transition_state(MissionState.FAILED)
            return
        
        # 状态处理
        if self.current_state == MissionState.STARTING:
            rospy.loginfo("Mission starting...")
            self.transition_state(MissionState.MOVING_TO_POINT1)
        
        elif self.current_state == MissionState.MOVING_TO_POINT1:
            if not self.move_in_progress:
                rospy.loginfo("Moving to Point 1...")
                self.send_move_command(self.point1)
                self.state_timeout = rospy.Duration(30)  # 30秒超时
        
        elif self.current_state == MissionState.LAUNCHING_DRONE:
            rospy.loginfo("Launching drone...")
            self.drone_cmd_pub.publish(Empty())
            rospy.sleep(2)  # 等待命令发送
            self.transition_state(MissionState.MOVING_TO_POINT2)
        
        elif self.current_state == MissionState.MOVING_TO_POINT2:
            if not self.move_in_progress:
                rospy.loginfo("Moving to Point 2...")
                self.send_move_command(self.point2)
                self.state_timeout = rospy.Duration(30)
        
        elif self.current_state == MissionState.WAITING_FOR_DRONE:
            rospy.loginfo("Waiting for drone to land...")
            if self.drone_landed:
                rospy.loginfo("Drone landed successfully!")
                self.transition_state(MissionState.MOVING_TO_CHARGE)
        
        elif self.current_state == MissionState.MOVING_TO_CHARGE:
            if not self.move_in_progress:
                rospy.loginfo("Moving to charging station...")
                self.send_move_command(self.charge_point)
                self.state_timeout = rospy.Duration(30)
        
        elif self.current_state == MissionState.CHARGING:
            if not hasattr(self, 'charge_start_time'):
                rospy.loginfo(f"Charging for {self.charging_time} seconds...")
                self.charge_start_time = rospy.Time.now()
            
            elapsed = (rospy.Time.now() - self.charge_start_time).to_sec()
            if elapsed >= self.charging_time:
                self.transition_state(MissionState.RETURNING_HOME)
        
        elif self.current_state == MissionState.RETURNING_HOME:
            if not self.move_in_progress:
                rospy.loginfo("Returning to home position...")
                self.send_move_command(self.home_point)
                self.state_timeout = rospy.Duration(45)
    
    def transition_state(self, new_state):
        """转换到新状态"""
        rospy.loginfo(f"State transition: {self.current_state.name} -> {new_state.name}")
        self.current_state = new_state
        self.last_state_change = rospy.Time.now()
        
        # 重置状态相关变量
        if new_state != MissionState.WAITING_FOR_DRONE:
            self.drone_landed = False
        if new_state != MissionState.CHARGING:
            if hasattr(self, 'charge_start_time'):
                del self.charge_start_time
    
    def send_move_command(self, target):
        """发送移动命令"""
        msg = Float32MultiArray()
        # 确保target是浮点数列表
        if isinstance(target, list):
            msg.data = [float(x) for x in target]
        else:
            # 如果是其他类型，转换为浮点数列表
            msg.data = [float(x) for x in target]
        self.move_pub.publish(msg)
        self.move_in_progress = True
        self.move_completed = False
    
    def drone_status_cb(self, msg):
        """无人机状态回调"""
        self.drone_landed = msg.data
        
        # 如果正在等待无人机降落并且无人机已着陆
        if self.current_state == MissionState.WAITING_FOR_DRONE and self.drone_landed:
            self.transition_state(MissionState.MOVING_TO_CHARGE)
    
    def move_completed_cb(self, msg):
        """移动完成回调"""
        self.move_completed = True
        self.move_in_progress = False
        
        # 状态转换
        if self.current_state == MissionState.MOVING_TO_POINT1:
            self.transition_state(MissionState.LAUNCHING_DRONE)
        elif self.current_state == MissionState.MOVING_TO_POINT2:
            self.transition_state(MissionState.WAITING_FOR_DRONE)
        elif self.current_state == MissionState.MOVING_TO_CHARGE:
            self.transition_state(MissionState.CHARGING)
        elif self.current_state == MissionState.RETURNING_HOME:
            self.transition_state(MissionState.COMPLETED)
    
    def get_progress(self):
        """获取任务进度百分比"""
        states = list(MissionState)
        current_index = states.index(self.current_state)
        
        # 排除开始和结束状态
        if current_index <= states.index(MissionState.STARTING):
            return 0
        if current_index >= states.index(MissionState.COMPLETED):
            return 100
        
        # 计算基本进度
        progress = (current_index - 1) * 100 / (states.index(MissionState.COMPLETED) - 1)
        
        # 特殊状态处理
        if self.current_state == MissionState.CHARGING:
            if hasattr(self, 'charge_start_time'):
                elapsed = (rospy.Time.now() - self.charge_start_time).to_sec()
                charging_progress = min(100, elapsed / self.charging_time * 100)
                # 充电状态占整体进度的10%
                return min(90 + charging_progress / 10, 100)
        
        return min(progress, 100)
    
    @property
    def move_in_progress(self):
        """检查是否正在移动"""
        return hasattr(self, '_move_in_progress') and self._move_in_progress
    
    @move_in_progress.setter
    def move_in_progress(self, value):
        """设置移动状态"""
        self._move_in_progress = value

if __name__ == '__main__':
    try:
        controller = MissionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass