#!/usr/bin/env python3

import rospy
import sys
import ast
from textual.app import App, ComposeResult
from textual.containers import Container, Vertical, Horizontal
from textual.widgets import Header, Footer, Button, ProgressBar, Static, Label, Input
from textual.screen import Screen
from ep_start.msg import MissionAction, MissionGoal, MissionFeedback
import actionlib


class MainScreen(Screen):
    """主屏幕 - 显示任务状态和控制按钮"""
    
    def __init__(self, app):
        super().__init__()
        self.app_ref = app

    def compose(self) -> ComposeResult:
        """创建用户界面"""
        yield Header("DJI EP 任务控制系统")
        yield Container(
            Label(f"任务名称: {self.app_ref.task_name}", id="task-name"),
            Static(f"当前状态: {self.app_ref.current_state}", id="current-state"),
            ProgressBar(total=100, show_eta=False, id="progress-bar"),
            Static(f"进度: {self.app_ref.progress:.1f}%", id="progress-text"),
            Horizontal(
                Button("▶ 开始任务", id="start-button", variant="success"),
                Button("⏹ 取消任务", id="cancel-button", variant="error"),
                id="button-container"
            ),
            Button("⚙ 参数设置", id="settings-button", variant="primary"),
            id="main-container"
        )
        yield Footer()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """处理按钮点击事件"""
        if event.button.id == "start-button":
            self.app_ref.start_mission()
        elif event.button.id == "cancel-button":
            self.app_ref.cancel_mission()
        elif event.button.id == "settings-button":
            self.app_ref.push_screen("settings")


class SettingsScreen(Screen):
    """设置屏幕 - 用于配置任务参数"""
    
    def __init__(self, app):
        super().__init__()
        self.app_ref = app

    def compose(self) -> ComposeResult:
        """创建设置界面"""
        yield Header("参数设置")
        yield Container(
            Label("参数设置", id="settings-title"),
            Vertical(
                Label("目标点1 [x, y, z]:"),
                Input(placeholder="[2.0, 0.0, 0.0]", id="point1-input", value=str(self.app_ref.point1)),
                Label("目标点2 [x, y, z]:"),
                Input(placeholder="[5.0, 0.0, 0.0]", id="point2-input", value=str(self.app_ref.point2)),
                Label("充电点 [x, y, z]:"),
                Input(placeholder="[0.0, 2.0, 0.0]", id="charge-point-input", value=str(self.app_ref.charge_point)),
                Label("回家点 [x, y, z]:"),
                Input(placeholder="[0.0, 0.0, 0.0]", id="home-point-input", value=str(self.app_ref.home_point)),
                Label("充电时间 (秒):"),
                Input(placeholder="30.0", id="charging-time-input", value=str(self.app_ref.charging_time)),
                id="settings-form"
            ),
            Horizontal(
                Button("保存", id="save-button", variant="success"),
                Button("取消", id="cancel-button", variant="error"),
                id="settings-buttons"
            ),
            id="settings-container"
        )
        yield Footer()

    def on_button_pressed(self, event: Button.Pressed) -> None:
        """处理按钮点击事件"""
        if event.button.id == "save-button":
            # 保存参数
            try:
                point1_input = self.query_one("#point1-input")
                point2_input = self.query_one("#point2-input")
                charge_point_input = self.query_one("#charge-point-input")
                home_point_input = self.query_one("#home-point-input")
                charging_time_input = self.query_one("#charging-time-input")
                
                self.app_ref.point1 = ast.literal_eval(point1_input.value)
                self.app_ref.point2 = ast.literal_eval(point2_input.value)
                self.app_ref.charge_point = ast.literal_eval(charge_point_input.value)
                self.app_ref.home_point = ast.literal_eval(home_point_input.value)
                self.app_ref.charging_time = float(charging_time_input.value)
                self.app_ref.save_parameters()
                self.app_ref.pop_screen()
            except Exception as e:
                self.app_ref.bell()
        elif event.button.id == "cancel-button":
            self.app_ref.pop_screen()


class MissionTUI(App):
    """任务进度显示的TUI界面"""
    
    CSS_PATH = "mission_tui.tcss"
    
    def __init__(self):
        super().__init__()
        self.current_state = "IDLE"
        self.progress = 0.0
        self.task_name = "DJI EP Mission"
        self.client = None
        self.connected = False
        
        # 任务参数
        self.point1 = [2.0, 0.0, 0.0]
        self.point2 = [5.0, 0.0, 0.0]
        self.charge_point = [0.0, 2.0, 0.0]
        self.home_point = [0.0, 0.0, 0.0]
        self.charging_time = 30.0

    def on_mount(self) -> None:
        """应用挂载时的初始化"""
        # 初始化 ROS 节点
        rospy.init_node('mission_tui', anonymous=True)
        
        # 创建 Action 客户端
        self.client = actionlib.SimpleActionClient('mission_control', MissionAction)
        
        # 更新连接状态
        self.connected = self.client.wait_for_server(rospy.Duration(5.0))
            
        # 设置定时器定期更新界面
        self.set_interval(0.5, self.update_display)
        
        # 安装屏幕
        self.install_screen(MainScreen(self), "main")
        self.install_screen(SettingsScreen(self), "settings")
        self.push_screen("main")

    def update_display(self) -> None:
        """更新显示内容"""
        # 只在主屏幕更新显示
        if isinstance(self.screen, MainScreen):
            try:
                current_state_widget = self.screen.query_one("#current-state", None)
                if current_state_widget:
                    current_state_widget.update(f"当前状态: {self.current_state}")
                
                progress_text_widget = self.screen.query_one("#progress-text", None)
                if progress_text_widget:
                    progress_text_widget.update(f"进度: {self.progress:.1f}%")
                
                progress_bar_widget = self.screen.query_one("#progress-bar", None)
                if progress_bar_widget:
                    progress_bar_widget.progress = int(self.progress)
                
                # 更新按钮状态
                start_button = self.screen.query_one("#start-button", None)
                cancel_button = self.screen.query_one("#cancel-button", None)
                
                if start_button and cancel_button:
                    if not self.connected:
                        start_button.disabled = True
                        cancel_button.disabled = True
                    else:
                        # 根据当前状态设置按钮是否可用
                        if self.current_state == "IDLE" or "FAILED" in self.current_state or self.current_state == "CANCELLED":
                            start_button.disabled = False
                            cancel_button.disabled = True
                        else:
                            start_button.disabled = True
                            cancel_button.disabled = False
            except Exception as e:
                pass  # 忽略查询异常

    def start_mission(self) -> None:
        """启动任务"""
        if self.client is None or not self.connected:
            self.current_state = "连接失败"
            return
            
        # 创建任务目标
        goal = MissionGoal()
        
        # 设置回调函数
        self.client.send_goal(
            goal,
            done_cb=self.done_callback,
            active_cb=self.active_callback,
            feedback_cb=self.feedback_callback
        )
        
        self.current_state = "STARTING"

    def cancel_mission(self) -> None:
        """取消任务"""
        if self.client:
            self.client.cancel_all_goals()
            self.current_state = "CANCELLED"

    def active_callback(self) -> None:
        """任务激活回调"""
        self.current_state = "ACTIVE"

    def feedback_callback(self, feedback: MissionFeedback) -> None:
        """任务反馈回调"""
        self.current_state = feedback.current_state
        self.progress = feedback.progress

    def done_callback(self, state, result) -> None:
        """任务完成回调"""
        if state == actionlib.GoalStatus.SUCCEEDED:
            self.current_state = "COMPLETED"
            self.progress = 100.0
        elif state == actionlib.GoalStatus.PREEMPTED:
            self.current_state = "CANCELLED"
        else:
            self.current_state = "FAILED"

    def save_parameters(self) -> None:
        """保存参数到ROS参数服务器"""
        try:
            rospy.set_param("/mission_controller/point1", str(self.point1))
            rospy.set_param("/mission_controller/point2", str(self.point2))
            rospy.set_param("/mission_controller/charge_point", str(self.charge_point))
            rospy.set_param("/mission_controller/home_point", str(self.home_point))
            rospy.set_param("/mission_controller/charging_time", self.charging_time)
        except Exception as e:
            print(f"保存参数时出错: {e}")


# 创建样式文件
css_content = """
Screen {
    background: #222233;
    color: #e0e0e0;
}

#main-container, #settings-container {
    layout: vertical;
    align: center middle;
    width: 100%;
    height: 100%;
    padding: 2;
}

#header {
    text-align: center;
    width: 100%;
    padding: 1 0;
    background: #1a1a2e;
    color: #ffffff;
    text-style: bold;
    margin-bottom: 2;
}

#task-name, #settings-title {
    text-align: center;
    width: 100%;
    margin-bottom: 1;
    background: #4a4a8a;
    color: #ffffff;
    padding: 1;
    text-style: bold;
    border: round #5d5d9e;
}

#current-state {
    text-align: center;
    width: 100%;
    margin-bottom: 1;
    background: #2d6a4f;
    color: #ffffff;
    padding: 1;
    border: round #40916c;
}

ProgressBar {
    width: 100%;
    height: 3;
    margin-bottom: 1;
    background: #33334d;
    border: round #5d5d9e;
}

Progress {
    background: #4caf50;
}

#progress-text {
    text-align: center;
    width: 100%;
    margin-bottom: 2;
    color: #e0e0e0;
    text-style: bold;
}

#button-container, #settings-buttons {
    width: 100%;
    height: auto;
    align: center middle;
    margin-top: 1;
    margin-bottom: 1;
}

Button {
    width: 18;
    margin-left: 2;
    margin-right: 2;
    height: 3;
    text-style: bold;
    border: round #5d5d9e;
}

#start-button {
    background: #2e7d32;
    color: #ffffff;
}

#cancel-button {
    background: #c62828;
    color: #ffffff;
}

#settings-button, #save-button {
    background: #1565c0;
    color: #ffffff;
}

Input {
    width: 100%;
    margin-bottom: 1;
    background: #33334d;
    color: #e0e0e0;
    border: round #5d5d9e;
}

#settings-form {
    width: 100%;
    margin-bottom: 2;
}

Label {
    width: 100%;
    color: #e0e0e0;
    margin-bottom: 1;
    text-style: bold;
}

Footer {
    background: #1a1a2e;
    color: #a0a0c0;
    text-style: italic;
}
"""

if __name__ == "__main__":
    # 先写入CSS文件
    css_path = "/home/ymc/git/me/Dji_EP_ws/src/ep_start/scripts/mission_tui.tcss"
    with open(css_path, "w") as f:
        f.write(css_content)
    
    # 运行应用
    app = MissionTUI()
    app.run()