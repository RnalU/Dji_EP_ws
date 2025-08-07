#!/usr/bin/env python3

import rospy
import actionlib
from ep_start.msg import MissionAction, MissionGoal
from std_msgs.msg import Empty


class MissionActionClient:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('mission_action_client')
        
        # 创建 Action 客户端
        self.client = actionlib.SimpleActionClient('mission_control', MissionAction)
        
        # 等待 Action 服务器启动
        rospy.loginfo("Waiting for mission_control server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to mission_control server")
        
        # 创建一个发布器用于手动触发任务（可选）
        self.start_pub = rospy.Publisher('/start_mission', Empty, queue_size=1)
        rospy.Subscriber('/start_mission', Empty, self.start_mission_callback)
        
        rospy.loginfo("Mission Action Client initialized")

    def start_mission_callback(self, msg):
        """当收到开始任务的命令时调用"""
        rospy.loginfo("Received start mission command")
        self.start_mission()

    def start_mission(self):
        """发送任务目标到 Action 服务器"""
        # 创建目标
        goal = MissionGoal()
        
        # 发送目标到 Action 服务器
        rospy.loginfo("Sending goal to mission controller...")
        self.client.send_goal(goal, 
                              done_cb=self.done_callback,
                              active_cb=self.active_callback,
                              feedback_cb=self.feedback_callback)
        
        # 等待结果（可选，也可以不等待）
        # self.client.wait_for_result()
        # result = self.client.get_result()
        # rospy.loginfo(f"Mission completed with success: {result.success}")

    def active_callback(self):
        """当目标变为活跃状态时调用"""
        rospy.loginfo("Mission goal is now active")

    def feedback_callback(self, feedback):
        """当收到反馈时调用"""
        rospy.loginfo(f"Current state: {feedback.current_state}, Progress: {feedback.progress:.1f}%")

    def done_callback(self, state, result):
        """当任务完成时调用"""
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Mission completed successfully! Success: {result.success}")
        else:
            rospy.logerr(f"Mission failed with state: {state}, Success: {result.success}")

    def run(self):
        """运行客户端"""
        rospy.spin()


if __name__ == '__main__':
    try:
        # 创建并运行 Action 客户端
        client = MissionActionClient()
        
        # 可选：立即启动任务
        # rospy.sleep(1)  # 等待一切初始化完成
        # client.start_mission()
        
        # 运行客户端
        client.run()
    except rospy.ROSInterruptException:
        pass
