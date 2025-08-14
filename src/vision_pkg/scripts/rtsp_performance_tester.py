#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RTSP流性能测试脚本
用于测试和评估RTSP流的延迟、帧率等性能指标
"""

import rospy
import cv2
import time
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class RTSPPerformanceTester:
    """RTSP性能测试器"""
    
    def __init__(self):
        rospy.init_node('rtsp_performance_tester', anonymous=True)
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = None
        self.latencies = []
        self.max_samples = 100
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(
            '/vision/gimbal_camera/image_raw', 
            Image, 
            self.image_callback
        )
        
        rospy.loginfo("RTSP性能测试器已启动")
        rospy.loginfo("监听话题: /vision/gimbal_camera/image_raw")
        
        # 启动统计线程
        self.stats_thread = threading.Thread(target=self.stats_loop, daemon=True)
        self.stats_thread.start()
        
    def image_callback(self, msg):
        """图像消息回调"""
        current_time = time.time()
        
        # 计算延迟（从消息时间戳到当前时间）
        msg_time = msg.header.stamp.to_sec()
        if msg_time > 0:
            latency = current_time - msg_time
            self.latencies.append(latency)
            
            # 保持最近的样本
            if len(self.latencies) > self.max_samples:
                self.latencies.pop(0)
        
        self.frame_count += 1
        self.last_frame_time = current_time
        
    def stats_loop(self):
        """统计信息循环"""
        while not rospy.is_shutdown():
            time.sleep(5.0)  # 每5秒输出一次统计
            
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            if self.frame_count > 0:
                avg_fps = self.frame_count / elapsed
                
                avg_latency = 0
                min_latency = 0
                max_latency = 0
                
                if self.latencies:
                    avg_latency = sum(self.latencies) / len(self.latencies)
                    min_latency = min(self.latencies)
                    max_latency = max(self.latencies)
                
                print("\n" + "="*60)
                print(f"RTSP流性能统计 (运行时间: {elapsed:.1f}s)")
                print("="*60)
                print(f"总帧数: {self.frame_count}")
                print(f"平均FPS: {avg_fps:.2f}")
                print(f"平均延迟: {avg_latency*1000:.2f}ms")
                print(f"最小延迟: {min_latency*1000:.2f}ms")
                print(f"最大延迟: {max_latency*1000:.2f}ms")
                
                # 检查是否有帧丢失
                if self.last_frame_time:
                    time_since_last = current_time - self.last_frame_time
                    if time_since_last > 1.0:
                        print(f"⚠ 警告: {time_since_last:.1f}s内未收到新帧")
                
                print("="*60)
            else:
                print("等待接收图像数据...")
                
    def run(self):
        """运行测试器"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            print("\n测试结束")


def main():
    """主函数"""
    try:
        tester = RTSPPerformanceTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
