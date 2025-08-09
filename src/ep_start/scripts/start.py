#!/usr/bin/env python3
import rospy
import math
import threading
from enum import Enum, auto
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Empty, Bool
from robomaster import robot, led

from gazebo_ros_link_attacher.srv import Attach, AttachRequest

class MissionState(Enum):
    """任务状态枚举"""
    IDLE = auto()                   # 空闲状态
    STARTING = auto()               # 任务开始
    MOVING_TO_POINT1 = auto()       # 移动到点1
    LAUNCHING_DRONE = auto()        # 无人机起飞
    WAITING_FOR_LAUNCH = auto()    # 等待无人机起飞
    MOVING_TO_POINT2 = auto()       # 移动到点2
    WAITING_FOR_DRONE = auto()      # 等待无人机降落
    MOVING_TO_CHARGE = auto()       # 移动到充电站
    CHARGING = auto()               # 等待充电
    RETURNING_HOME = auto()         # 返回出发点
    COMPLETED = auto()              # 任务结束
    FAILED = auto()                 # 状态失败

class MissionController:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('mission_controller', log_level=rospy.INFO)

        # 地图源尺寸
        self.map_shape_orignal = [4.6, 9]

        # 当前地图尺寸
        self.map_shape_prefix = [4.2, 4.2]

        # 任务参数
        point1_param = rospy.get_param("~point1", "[2.5, -0.9, 0.0]")  # [x:m, y:m, z:°]
        point2_param = rospy.get_param("~point2", "[2.57, 1.8, 0.0]")
        charge_point_param = rospy.get_param("~charge_point", "[2.0, -0.9, 0.0]")
        home_point_param = rospy.get_param("~home_point", "[8.0, 0.0, 0.0]")

        # 仿真相关
        self.sim_flag = True

        if self.sim_flag:
           
            # link attacher
            # 是否使用 attach 功能
            self.use_attach = rospy.get_param("~use_attach", True)

            # 模型/连结名称
            self.platform_model = rospy.get_param("~platform_model", "smartcar")
            self.platform_link = rospy.get_param("~platform_link", "base_footprint")
            self.drone_model = rospy.get_param("~drone_model", "p230_0")
            self.drone_link = rospy.get_param("~drone_link", "base_link")  # TODO: 确认实际名称

            # service 句柄占位
            self._attach_srv = None
            self._detach_srv = None

            # 初始化 service
            if self.use_attach:
                self._init_attach_services()

        # 解析参数为浮点数列表
        import ast
        self.point1 = [float(x) for x in ast.literal_eval(point1_param)] if isinstance(point1_param, str) else [float(x) for x in point1_param]
        self.point2 = [float(x) for x in ast.literal_eval(point2_param)] if isinstance(point2_param, str) else [float(x) for x in point2_param]
        self.charge_point = [float(x) for x in ast.literal_eval(charge_point_param)] if isinstance(charge_point_param, str) else [float(x) for x in charge_point_param]

        self.home_point = [8.0, 0.0, 0.0]

        self.charging_time = float(rospy.get_param("~charging_time", 5.0))  # 秒

        # 修正坐标参数 因仿真与实际y轴速度相反
        if self.sim_flag:
            self.point1[1] = -self.point1[1]
            self.point2[1] = -self.point2[1]
            self.charge_point[1] = -self.charge_point[1]
            self.home_point[1] = -self.home_point[1]

        # 状态跟踪
        self.current_state = MissionState.IDLE
        self.last_state_change = rospy.Time.now()
        self.state_timeout = rospy.Duration(60)  # 默认状态超时时间

        # 当前位置跟踪
        self.current_position = [0.0, 0.0, 0.0]

        # 发布话题
        self.drone_cmd_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.move_pub = rospy.Publisher('/move_distance', Float32MultiArray, queue_size=1)
        self.set_speed_pub = rospy.Publisher('/set_speed', Float32MultiArray, queue_size=1)  # 添加速度设置发布者

        # 订阅话题
        rospy.Subscriber('/drone/status', Bool, self.drone_status_cb)
        rospy.Subscriber('/move_completed', Empty, self.move_completed_cb)

        # 无人机状态
        self.drone_landed = False
        self.drone_takeoff = False

        # 小车状态
        self.move_completed = False

        # 状态机运行标志
        self.running = False

        # 多命令队列
        self.command_queue = []
        self.current_command_index = 0

        rospy.loginfo("Mission Controller Initialized")

        # 启动任务
        self.start_mission()


    # link attacher 
    def _init_attach_services(self):
        """创建 attach/detach 服务"""
        attach_name = "/link_attacher_node/attach"
        detach_name = "/link_attacher_node/detach"
        
        rospy.loginfo("Waiting for link attacher services...")
        attach_srv = rospy.ServiceProxy(attach_name, Attach)
        detach_srv = rospy.ServiceProxy(detach_name, Attach)
        attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
        detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        rospy.wait_for_service(attach_name)
        rospy.wait_for_service(detach_name)
        rospy.loginfo("Link attacher services are available.")

        self._attach_srv = rospy.ServiceProxy(attach_name, Attach)
        self._detach_srv = rospy.ServiceProxy(detach_name, Attach)
        rospy.loginfo("Attach services ready.")

    def attach_drone(self):
        """创建固定关节，把无人机锁在平台上"""
        if not self.use_attach:
            return
        if self._attach_srv is None:
            rospy.logwarn("Attach service not ready.")
            return
        req = AttachRequest()
        req.model_name_1 = self.platform_model
        req.link_name_1 = self.platform_link
        req.model_name_2 = self.drone_model
        req.link_name_2 = self.drone_link
        try:
            resp = self._attach_srv(req)
            if resp.ok:
                rospy.loginfo(f"Attached {req.model_name_2}/{req.link_name_2} to {req.model_name_1}/{req.link_name_1}")
            else:
                rospy.logwarn("Attach service returned ok=False")
        except rospy.ServiceException as e:
            rospy.logerr(f"Attach service call failed: {e}")

    def detach_drone(self):
        """解除固定关节"""
        if not self.use_attach:
            return
        if self._detach_srv is None:
            rospy.logwarn("Detach service not ready.")
            return
        req = AttachRequest()
        req.model_name_1 = self.platform_model
        req.link_name_1 = self.platform_link
        req.model_name_2 = self.drone_model
        req.link_name_2 = self.drone_link
        try:
            resp = self._detach_srv(req)
            if resp.ok:
                rospy.loginfo(f"Detached {req.model_name_2} from platform")
            else:
                rospy.logwarn("Detach service returned ok=False")
        except rospy.ServiceException as e:
            rospy.logerr(f"Detach service call failed: {e}")


    def start_mission(self):
        """启动任务"""
        if self.current_state != MissionState.IDLE:
            rospy.logwarn("Mission is already running or not in IDLE state")
            return False

        rospy.loginfo("Starting mission...")

        # 重置状态
        self.current_state = MissionState.STARTING
        self.drone_landed = False
        self.drone_takeoff = False
        self.move_completed = False
        self.running = True

        # 重置当前位置
        self.current_position = [0.0, 0.0, 0.0]

        # 启动状态机线程
        self.state_machine_thread = threading.Thread(target=self._run_state_machine)
        self.state_machine_thread.daemon = True
        self.state_machine_thread.start()

        return True

    def _run_state_machine(self):
        """运行状态机的主循环"""
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.running:
            # 处理当前状态
            self.handle_state()

            # 检查任务完成
            if self.current_state == MissionState.COMPLETED:
                rospy.loginfo("Mission completed successfully!")
                self.running = False
                return

            # 检查任务失败
            if self.current_state == MissionState.FAILED:
                rospy.logerr("Mission failed!")
                self.running = False
                return

            rate.sleep()

    def stop_mission(self):
        """停止任务"""
        rospy.logwarn("Mission stopped!")
        self.current_state = MissionState.FAILED

    def handle_state(self):
        """处理当前状态"""
        # 检查状态超时
        if (rospy.Time.now() - self.last_state_change) > self.state_timeout:
            rospy.logwarn(f"State {self.current_state.name} timed out!")

            # 如果超时，则直接进行下一个任务
            if self.current_state == MissionState.MOVING_TO_POINT1:
                rospy.logwarn("Start Failure! Moving to Point 1 timed out, transitioning to FAILED state")
                self.transition_state(MissionState.FAILED)

            elif self.current_state == MissionState.WAITING_FOR_DRONE:
                rospy.logwarn("Waiting for drone timed out, transitioning to MOVING_TO_CHARGE state")
                self.transition_state(MissionState.MOVING_TO_CHARGE)

            else:
                rospy.logwarn(f"Unhandled state timeout for {self.current_state.name}")
                self.transition_state(MissionState.FAILED)
            return

        # 发车
        if self.current_state == MissionState.STARTING:
            rospy.loginfo("Mission starting...")
            # 确保上车锁定
            if self.use_attach:
                self.attach_drone()

            self.transition_state(MissionState.MOVING_TO_POINT1)

        # 移动到点1
        elif self.current_state == MissionState.MOVING_TO_POINT1:
            if not self.move_in_progress:
                rospy.loginfo("Moving to Point 1({}, {}, {})...".format(self.point1[0], self.point1[1], self.point1[2]))
                self.command_queue = [self.point1]
                self.current_command_index = 0
                self.send_move_command(self.command_queue[self.current_command_index])
                self.state_timeout = rospy.Duration(30)  # 30秒超时

        # 飞机起飞状态
        elif self.current_state == MissionState.LAUNCHING_DRONE:
            rospy.sleep(2) # 稍等稳定
            # 起飞前先 detach
            if self.use_attach:
                rospy.loginfo("Detaching drone before launch...")
                self.detach_drone()
                rospy.sleep(2)  # 稍等物理稳定
            rospy.loginfo("Launching drone...")
            self.drone_cmd_pub.publish(Empty())
            rospy.sleep(2)  # 等待命令发送
            self.transition_state(MissionState.WAITING_FOR_LAUNCH)

        elif self.current_state == MissionState.WAITING_FOR_LAUNCH:
            rospy.loginfo("Waiting for drone to launch...")
            # 等待无人机起飞完成
            if self.drone_takeoff:
                rospy.loginfo("Drone launched successfully!")
                self.transition_state(MissionState.MOVING_TO_POINT2)
            # 设置状态超时
            self.state_timeout = rospy.Duration(30)  # 30秒超时

        # 移动到点2
        elif self.current_state == MissionState.MOVING_TO_POINT2:
            if not self.move_in_progress:
                rospy.loginfo("Moving to Point 2({}, {}, {})...".format(self.point2[0], self.point2[1], self.point2[2]))
                self.send_move_command(self.point2)
                self.state_timeout = rospy.Duration(30)

        # 等待无人机降落
        elif self.current_state == MissionState.WAITING_FOR_DRONE:
            rospy.loginfo("Waiting for drone to land...")

            # 此处等待无人机算力盒子发布话题
            if self.drone_landed:
                rospy.loginfo("Drone landed successfully!")
                self.transition_state(MissionState.MOVING_TO_CHARGE)

            # Debug
            self.state_timeout = rospy.Duration(300)  # 300S 任务执行延迟阈值

        # 移动到充电站
        elif self.current_state == MissionState.MOVING_TO_CHARGE:
            if not self.move_in_progress:
                rospy.loginfo("Moving to charging station({}, {}, {})...".format(self.charge_point[0], self.charge_point[1], self.charge_point[2]))
                self.command_queue = [self.charge_point, [0, 0, 180]]
                self.current_command_index = 0
                self.send_move_command(self.command_queue[self.current_command_index])
                self.state_timeout = rospy.Duration(30)

        # 等待充电完成
        elif self.current_state == MissionState.CHARGING:
            if not hasattr(self, 'charge_start_time'):
                rospy.loginfo(f"Charging for {self.charging_time} seconds...")
                self.charge_start_time = rospy.Time.now()

            elapsed = (rospy.Time.now() - self.charge_start_time).to_sec()
            if elapsed >= self.charging_time:
                self.transition_state(MissionState.RETURNING_HOME)

        # 比赛结束
        elif self.current_state == MissionState.RETURNING_HOME:
            if not self.move_in_progress:
                rospy.loginfo("Returning to home position...")
                # relative_move = self.calculate_relative_move(self.point_nomalize(self.home_point, self.map_shape_orignal, self.map_shape_prefix))
                self.command_queue = [ [2, 0, 0], [2, 0, 0], [2, 0, 0], [1, 0, 0] ]
                self.current_command_index = 0;
                self.send_move_command(self.command_queue[ self.current_command_index ])
                self.state_timeout = rospy.Duration(45)

        elif self.current_state == MissionState.FAILED:
            rospy.loginfo("Mission Failed! Back to home position...")
            # 移动小车至原来位置
            self.command_queue = [[0, 0, -self.current_position[2]], [-self.current_position[0], -self.current_position[1], 0.0]]
            self.send_move_command([self.current_position[0], self.current_position[1], 0.0])


    def point_nomalize(self, point, map_shape_orignal, map_shape_prefix):
        """将点坐标归一化到当前地图尺寸"""
        if map_shape_orignal[0] == map_shape_prefix[0] and map_shape_orignal[1] == map_shape_prefix[1]:
            return point

        normalized_x = point[0] * (map_shape_prefix[0] / map_shape_orignal[0])
        normalized_y = point[1] * (map_shape_prefix[1] / map_shape_orignal[1])
        return [normalized_x, normalized_y, point[2]]

    def calculate_relative_move(self, target_position):
        """计算从当前位置到目标位置的相对移动距离"""
        relative_move = [
            target_position[0] - self.current_position[0],
            target_position[1] - self.current_position[1],
            target_position[2] - self.current_position[2]
        ]

        # 更新当前位置为目标位置
        self.current_position = target_position[:]

        rospy.loginfo(f"Moving from {self.current_position} to {target_position}, relative move: {relative_move}")
        return relative_move

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

        self.current_position[0] += msg.data[0]
        self.current_position[1] += msg.data[1]
        self.current_position[2] = msg.data[2]

        self.move_pub.publish(msg)
        self.move_in_progress = True
        self.move_completed = False

    def set_speed(self, linear_scale=None, angular_scale=None, default_speed=None):
        """设置机器人速度参数"""
        # 发布速度设置消息
        msg = Float32MultiArray()
        msg.data = [
            linear_scale if linear_scale is not None else -1.0,
            angular_scale if angular_scale is not None else -1.0,
            default_speed if default_speed is not None else -1.0
        ]
        self.set_speed_pub.publish(msg)
        rospy.loginfo(f"Speed command sent: linear_scale={linear_scale}, angular_scale={angular_scale}, default_speed={default_speed}")

    def drone_status_cb(self, msg):
        """无人机状态回调"""
        self.drone_status = msg.data

        if self.drone_status:
            rospy.loginfo("Drone is flying.")
            self.drone_takeoff = True
        if self.drone_status is False:
            rospy.loginfo("Drone has landed.")
            self.drone_landed = True
        
        # 无人机成功降落后重新 attach（如果在等待降落或刚降落）
        if self.drone_status is False:
            rospy.loginfo("Drone has landed.")
            if self.use_attach:
                rospy.loginfo("Re-attaching drone to platform...")
                self.attach_drone()
        # 如果正在等待无人机降落并且无人机已着陆
        if self.current_state == MissionState.WAITING_FOR_DRONE and self.drone_landed:
            self.transition_state(MissionState.MOVING_TO_CHARGE)
            
    def move_completed_cb(self, msg):
        """移动完成回调"""
        self.move_completed = True
        self.move_in_progress = False
        
        # 检查是否有命令队列需要处理
        if self.command_queue and self.current_command_index < len(self.command_queue) - 1:
            # 执行队列中的下一个命令
            self.current_command_index += 1
            self.send_move_command(self.command_queue[self.current_command_index])
            return
        
        # 清空命令队列
        self.command_queue = []
        self.current_command_index = 0

        rospy.loginfo("Move completed, Current position: {}".format(self.current_position))

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
