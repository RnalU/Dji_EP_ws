#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
二维码检测节点
提供二维码检测服务和实时检测功能
"""

import rospy
import cv2
import numpy as np
from pyzbar import pyzbar
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from vision_pkg.msg import QRCodeResult
from vision_pkg.srv import DetectQRCode, DetectQRCodeResponse


class QRCodeDetector:
    def __init__(self):
        """初始化二维码检测器"""
        rospy.init_node('qrcode_detector', anonymous=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 参数
        self.debug_mode = rospy.get_param('~debug_mode', True)
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.publish_result = rospy.get_param('~publish_result', True)
        
        # 中间参数
        self.start_detection_flag = False

        # 发布器
        if self.publish_result:
            self.result_pub = rospy.Publisher('qrcode_result', QRCodeResult, queue_size=1)
            self.debug_image_pub = rospy.Publisher('qrcode_debug_image', Image, queue_size=1)
        
        # 订阅器
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.start_qr_detection = rospy.Subscriber('start_qr_detection', Empty, self.start_qr_detection_callback)

        # 服务
        self.detect_service = rospy.Service('detect_qrcode', DetectQRCode, self.detect_service_callback)
        
        rospy.loginfo("QR码检测器已启动")
        rospy.loginfo(f"图像话题: {self.image_topic}")
        rospy.loginfo(f"调试模式: {self.debug_mode}")
        rospy.loginfo(f"发布结果: {self.publish_result}")

    def detect_qrcode(self, cv_image):
        """
        检测图像中的二维码
        
        Args:
            cv_image: OpenCV图像
            
        Returns:
            QRCodeResult: 检测结果
        """
        result = QRCodeResult()
        result.header = Header()
        result.header.stamp = rospy.Time.now()
        result.header.frame_id = "camera"
        result.detected = False
        result.confidence = 0.0
        
        try:
            # 转换为灰度图像
            if len(cv_image.shape) == 3:
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            else:
                gray = cv_image
            
            # 使用pyzbar检测二维码
            barcodes = pyzbar.decode(gray)
            
            if barcodes:
                # 获取第一个检测到的二维码
                barcode = barcodes[0]
                
                result.detected = True
                result.data = barcode.data.decode('utf-8')
                result.confidence = 1.0  # pyzbar没有置信度，设为1.0
                
                # 计算中心点
                points = barcode.polygon
                if len(points) == 4:
                    # 计算中心点
                    center_x = sum([p.x for p in points]) / 4
                    center_y = sum([p.y for p in points]) / 4
                    
                    result.position = Point()
                    result.position.x = center_x
                    result.position.y = center_y
                    result.position.z = 0.0
                    
                    # 四个角点
                    result.corners = []
                    for point in points:
                        corner = Point()
                        corner.x = float(point.x)
                        corner.y = float(point.y)
                        corner.z = 0.0
                        result.corners.append(corner)
                
                rospy.loginfo(f"检测到二维码: {result.data}")
                rospy.loginfo(f"位置: ({result.position.x:.1f}, {result.position.y:.1f})")
            
        except Exception as e:
            rospy.logerr(f"二维码检测错误: {str(e)}")
        
        return result

    def draw_qrcode_info(self, image, result):
        """
        在图像上绘制二维码检测信息
        
        Args:
            image: OpenCV图像
            result: QRCodeResult消息
            
        Returns:
            绘制了信息的图像
        """
        debug_image = image.copy()
        
        if result.detected:
            # 绘制边界框
            if len(result.corners) == 4:
                points = np.array([[int(corner.x), int(corner.y)] for corner in result.corners])
                cv2.polylines(debug_image, [points], True, (0, 255, 0), 2)
                
                # 绘制中心点
                center = (int(result.position.x), int(result.position.y))
                cv2.circle(debug_image, center, 5, (0, 0, 255), -1)
                
                # 显示二维码内容
                cv2.putText(debug_image, f"QR: {result.data}", 
                           (center[0] - 50, center[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(debug_image, f"QR: {result.data}", 
                           (center[0] - 50, center[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)
        
        # 显示检测状态
        status_text = "QR检测: 已检测到" if result.detected else "QR检测: 未检测到"
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(debug_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0) if result.detected else (0, 0, 255), 1)
        
        return debug_image

    def start_qr_detection_callback(self, msg):
        """开始二维码检测回调函数"""
        if not self.start_detection_flag:
            self.start_detection_flag = True
            rospy.loginfo("开始二维码检测")
        elif self.start_detection_flag:
            self.start_detection_flag = False
            rospy.loginfo("二维码检测关闭")


    def image_callback(self, msg):
        """图像话题回调函数"""
        # 未开始检测时直接跳过，节省算力
        if not self.start_detection_flag:
            # 发布调试图像
            if self.debug_mode:
                # 未开始检测，绘制提示
                debug_image = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(debug_image, "QR Detection: Not Started", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                    debug_msg.header = msg.header
                    self.debug_image_pub.publish(debug_msg)
                except CvBridgeError as e:
                    rospy.logerr(f"调试图像转换错误: {str(e)}")
            return

        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测二维码
            result = self.detect_qrcode(cv_image)
            
            # 发布结果
            if self.publish_result:
                self.result_pub.publish(result)
                
                # 发布调试图像
                if self.debug_mode:
                    debug_image = self.draw_qrcode_info(cv_image, result)
                    try:
                        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                        debug_msg.header = msg.header
                        self.debug_image_pub.publish(debug_msg)
                    except CvBridgeError as e:
                        rospy.logerr(f"调试图像转换错误: {str(e)}")
                        
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {str(e)}")

    def detect_service_callback(self, req):
        """二维码检测服务回调函数"""
        response = DetectQRCodeResponse()
        
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
            
            # 检测二维码
            result = self.detect_qrcode(cv_image)
            
            response.success = True
            response.result = result
            response.message = "检测完成"
            
            if result.detected:
                response.message += f", 检测到二维码: {result.data}"
            else:
                response.message += ", 未检测到二维码"
                
        except Exception as e:
            response.success = False
            response.message = f"检测失败: {str(e)}"
            rospy.logerr(f"服务调用错误: {str(e)}")
        
        return response

    def run(self):
        """运行节点"""
        rospy.spin()


if __name__ == '__main__':
    try:
        detector = QRCodeDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
