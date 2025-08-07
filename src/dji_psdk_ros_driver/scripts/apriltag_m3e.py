#!/usr/bin/python3
# -*- coding: UTF-8 -*-
import rospy
import apriltag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tta_m3e_rtsp.msg import flightByVel
from tta_m3e_rtsp.srv import takeoffOrLanding, takeoffOrLandingRequest
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from tta_m3e_rtsp.cfg import pidConfig
from tta_m3e_rtsp.cfg import GlobalStateConfig


class AprilTagPID:
    def __init__(self):
        rospy.init_node('apriltag_location', anonymous=False)

        rospy.logwarn("==========================startup apriltag node============================")
        try:
            self.apriltag_detector = apriltag.Detector()
        except AttributeError:
            rospy.logerr("Failed to create apriltag detector. Check the apriltag library version.")

        self.draw_result_on_image = rospy.get_param('draw_on', default=True)

        self.pid_kp_x = [0.0, 0.0, 0.0]
        self.pid_ki_x = [0.0, 0.0, 0.0]
        self.pid_kd_x = [0.0, 0.0, 0.0]

        self.pid_kp_y = [0.0, 0.0, 0.0]
        self.pid_ki_y = [0.0, 0.0, 0.0]
        self.pid_kd_y = [0.0, 0.0, 0.0]

        # 初始化误差积分和上一次误差
        self.error_integral_x = 0.0
        self.error_integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0

        self.input_image_topic = rospy.get_param('input_image', default="image")
        self.output_image_topic = rospy.get_param('output_image', default="apriltag/image")
        self.control_velocity_topic = rospy.get_param("cmd_vel", default="joy_vel")

        self.takeoff_landing_service = rospy.ServiceProxy("takeoffOrLanding", takeoffOrLanding)

        self.control_velocity_publisher = rospy.Publisher(self.control_velocity_topic, flightByVel, queue_size=1)

        rospy.Subscriber(self.input_image_topic, Image, self.apriltag_bias_callback)

        self.output_image_publisher = rospy.Publisher(self.output_image_topic, Image, queue_size=1)

        self.rate = rospy.Rate(10)
        self.wait_duration = rospy.Duration(10)
        self.error_duration = rospy.Duration(1)

        self.global_system_state = ""

        self.pid_config_server = Server(pidConfig, callback=self.dynamic_reconfigure_pid_callback)
        self.global_state_client = Client("/cpu", timeout=10, config_callback=self.dynamic_reconfigure_state_callback)

    def dynamic_reconfigure_pid_callback(self, config, level):
        """
        动态参数配置回调函数，更新PID参数
        :param config: 包含新参数的配置对象
        """
        self.pid_kp_x = config["vx_kp"]
        self.pid_ki_x = config["vx_ki"]
        self.pid_kd_x = config["vx_kd"]

        self.pid_kp_y = config["vz_kp"]
        self.pid_ki_y = config["vz_ki"]
        self.pid_kd_y = config["vz_kd"]

        return config

    def dynamic_reconfigure_state_callback(self, config):
        """
        动态全局状态参数配置回调函数，更新全局状态
        :param config: 包含新状态的配置对象
        """
        self.global_system_state = config.global_state
        rospy.loginfo(f"dynamic param: {self.global_system_state}")

    def compute_pid_control(self, target_value, current_value, kp, ki, kd, axis):
        """
        计算PID控制输出
        :param target_value: 目标值
        :param current_value: 当前值
        :param kp: 比例系数
        :param ki: 积分系数
        :param kd: 微分系数
        :param axis: 控制轴（"x" 或 "y"）
        :return: 控制输出值
        """
        # 计算误差
        error = target_value - current_value

        if axis == "x":
            # 计算积分项
            self.error_integral_x += error

            # 计算微分项
            derivative = error - self.prev_error_x

            # 计算控制输出
            control_output = kp * error + ki * self.error_integral_x + kd * derivative

            # 更新上一次误差
            self.prev_error_x = error

            return control_output

        elif axis == "y":
            # 计算积分项
            self.error_integral_y += error

            # 计算微分项
            derivative = error - self.prev_error_y

            # 计算控制输出
            control_output = kp * error + ki * self.error_integral_y + kd * derivative

            # 更新上一次误差
            self.prev_error_y = error

            return control_output

        return 0

    def process_apriltag_detection(self, tag, cv_image, center_x, center_y, width_scale, height_scale, msg):
        """
        处理AprilTag检测结果，包括绘制标记、计算控制输出、发送控制消息等
        :param tag: AprilTag检测结果对象
        :param cv_image: 当前的OpenCV图像
        :param center_x: AprilTag中心点的x坐标
        :param center_y: AprilTag中心点的y坐标
        :param width_scale: AprilTag的宽度比例
        :param height_scale: AprilTag的高度比例
        :param msg: 接收到的图像消息
        """
        if width_scale > 45 and self.global_system_state == 25:
            rospy.logwarn("Prepare to send the landing request in 7 seconds. ")
            rospy.sleep(self.wait_duration)
            self.takeoff_landing_service.wait_for_service()
            request = takeoffOrLandingRequest()
            request.takeoffOrLanding = 2
            response = self.takeoff_landing_service.call(request)
            rospy.sleep(self.wait_duration)
            rospy.logwarn(f"The landing result of the aircraft: {response} , and the global status has changed. ")
            # params = {
            #     "global_state": 26
            # }
            # config = self.global_state_client.update_configuration(params)
            # rospy.loginfo(f"参数更新成功: {config}")
            self.global_system_state = 26

        if self.draw_result_on_image:
            cv2.polylines(cv_image, [tag.corners.astype(int)], True, (0, 255, 0), 2)
            cv2.circle(cv_image, (center_x, center_y), 3, (0, 255, 0), 1, -1)

        control_x = self.compute_pid_control(160.0, center_x, self.pid_kp_x, self.pid_ki_x, self.pid_kd_x, "x")
        control_y = self.compute_pid_control(85.0, center_y, self.pid_kp_y, self.pid_ki_y, self.pid_kd_y, "y")
        # print(f"send velocity:  x-> ({-control_x}, y-> {-control_y})")
        control_message = flightByVel()
        control_message.vel_n = control_y
        control_message.vel_e = -control_x
        control_message.fly_time = 0.1
        if (abs(160 - center_x) < 5) and (abs(85 - center_y) < 5) and self.global_system_state == 25:
            control_message.vel_d = -0.12
        elif (abs(160 - center_x) < 5) and (abs(85 - center_y) < 5) and self.global_system_state == 14:
            control_message.vel_d = 0.0
            rospy.logwarn("It is aligning with the AprilTag point. ")
            # params = {
            #         "global_state": 15
            # }
            # config = self.global_state_client.update_configuration(params)
            # self.global_system_state = 15
            # rospy.loginfo(f"参数更新成功: {config}")
        self.control_velocity_publisher.publish(control_message)


        if rospy.get_param("resend_img", default="False"):
            try:
                bridge = CvBridge()
                output_msg = bridge.cv2_to_imgmsg(cv_image, "bgr8")
                output_msg.header = msg.header

                self.output_image_publisher.publish(output_msg)
            except Exception as e:
                rospy.logerr("Error converting and publishing image: %s", e)

    def apriltag_bias_callback(self, msg):
        """
        处理图像消息，检测AprilTag并计算控制偏差
        :param msg: 接收到的图像消息
        """
        if self.global_system_state == 14 or self.global_system_state == 25:
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")  # 640 * 360
                print(f"Image scase [{cv_image.shape}]")

                cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
            except Exception as e:
                rospy.logerr("Error converting image: %s", e)
                return

            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            detection_results = self.apriltag_detector.detect(gray_image)

            try:
                for detected_tag in detection_results:
                    corners = detected_tag.corners
                    center_x = int((corners[0][0] + corners[1][0] + corners[2][0] + corners[3][0]) / 4)
                    center_y = int((corners[0][1] + corners[1][1] + corners[2][1] + corners[3][1]) / 4)
                    print(f"Center point coordinates: ({center_x}, {center_y})")
                    width_scale = (corners[1][0] - corners[0][0])
                    height_scale = (corners[1][1] - corners[0][1])
                    print(f"Apriltag width: {width_scale}, height:{height_scale}")

                    self.process_apriltag_detection(detected_tag, cv_image, center_x, center_y, width_scale, height_scale, msg)

            except Exception as e:
                rospy.logerr("Error processing AprilTag: %s", e)

        else:
            rospy.sleep(self.error_duration)
if __name__ == '__main__':
    apriltag_pid = AprilTagPID()
    rospy.spin()
