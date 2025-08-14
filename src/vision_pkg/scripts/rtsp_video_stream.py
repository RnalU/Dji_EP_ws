#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""
高性能RTSP视频流接收器
功能：
1. 低延迟RTSP流接收
2. 多线程处理架构
3. 自动重连机制
4. ROS Image话题发布
5. 帧率和延迟监控

作者: GitHub Copilot
日期: 2025-08-13
"""

import rospy
import cv2
import numpy as np
import threading
import time
import queue
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
import signal
import sys
import subprocess
import shutil


class OptimizedRTSPStreamer:
    """优化的RTSP流处理器"""
    
    def __init__(self):
        """初始化RTSP流处理器"""
        rospy.init_node('rtsp_streamer', anonymous=True)

        # 参数配置
        self.rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://192.168.160.253:8554/live')
        self.output_topic = rospy.get_param('~output_topic', '/vision/gimbal_camera/image_raw')
        self.fps_limit = rospy.get_param('~fps_limit', 30)
        self.buffer_size = rospy.get_param('~buffer_size', 1)
        self.reconnect_delay = rospy.get_param('~reconnect_delay', 2.0)
        self.use_gstreamer = rospy.get_param('~use_gstreamer', True)
        self.decode_mode = rospy.get_param('~decode_mode', 'auto').lower()
        self.latency_ms = rospy.get_param('~latency', 50)
        self.force_tcp = rospy.get_param('~force_tcp', True)

        # ROS 组件
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)

        # 线程控制
        self.frame_queue = queue.Queue(maxsize=self.buffer_size)
        self.running = True
        self.capture_thread = None
        self.publish_thread = None

        # 统计
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0
        self.total_latency = 0
        self.latency_samples = 0

        # 视频捕获对象
        self.cap = None

        rospy.loginfo("RTSP Streamer初始化完成")
        rospy.loginfo(f"RTSP URL: {self.rtsp_url}")
        rospy.loginfo(f"输出话题: {self.output_topic}")
        rospy.loginfo(f"使用GStreamer: {self.use_gstreamer}")
        rospy.loginfo(f"解码模式: {self.decode_mode}")

        self._log_opencv_gst_support()
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        rospy.loginfo("接收到退出信号，正在关闭...")
        self.stop()
        sys.exit(0)
        
    def _has_exec(self, name: str) -> bool:
        return shutil.which(name) is not None

    def _has_gst_element(self, element: str) -> bool:
        """通过 gst-inspect-1.0 检查元素是否存在"""
        if not self._has_exec('gst-inspect-1.0'):
            return False
        try:
            out = subprocess.run(['gst-inspect-1.0', element], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=2)
            return out.returncode == 0
        except Exception:
            return False

    def _log_opencv_gst_support(self):
        try:
            build_info = cv2.getBuildInformation()
            gst_supported = 'GStreamer' in build_info and 'YES' in [l.split(':')[1].strip() for l in build_info.split('\n') if l.startswith('GStreamer')][:1]
            rospy.loginfo(f"OpenCV 编译 GStreamer 支持: {gst_supported}")
        except Exception:
            rospy.logwarn("无法获取 OpenCV 构建信息")

    def _select_decoder_chain(self):
        """根据模式与可用插件选择解码器链 (不含 avdec 前后的空格)."""
        # 优先顺序: 用户指定 gpu -> 自动探测 NVDEC / VAAPI / CPU
        gpu_candidates = [
            ('nvh264dec', 'nvh264dec ! videoconvert'),  # NVIDIA dGPU (gst-plugins-bad 1.20+)
            ('nvv4l2decoder', 'nvv4l2decoder ! videoconvert'),  # Jetson
            ('vah264dec', 'vah264dec ! videoconvert'),  # 部分 VAAPI 命名
            ('vaapih264dec', 'vaapih264dec ! videoconvert'),  # VAAPI 常规
        ]

        cpu_chain = 'avdec_h264 ! videoconvert'

        # 强制 CPU
        if self.decode_mode == 'cpu':
            return 'CPU', cpu_chain

        # 强制 GPU (如果失败再回退 CPU)
        if self.decode_mode == 'gpu':
            for name, chain in gpu_candidates:
                if self._has_gst_element(name):
                    return name, chain
            rospy.logwarn("未发现可用 GPU 解码器, 回退 CPU")
            return 'CPU', cpu_chain

        # auto 模式
        for name, chain in gpu_candidates:
            if self._has_gst_element(name):
                return name, chain
        return 'CPU', cpu_chain

    def _create_gstreamer_pipeline(self):
        """创建多套 GStreamer 管道用于尝试"""
        decoder_name, decoder_chain = self._select_decoder_chain()
        proto = ' protocols=tcp' if self.force_tcp else ''
        latency = max(0, int(self.latency_ms))

        # 共性 appsink 片段
        appsink_part = 'appsink emit-signals=false sync=false max-buffers=1 drop=true enable-last-sample=false'
        # 注意：某些 OpenCV 版本不识别 emit-signals, 仍可忽略

        base_src = f"rtspsrc location={self.rtsp_url}{proto} latency={latency} drop-on-latency=true"

        # 常见问题: 过低 latency=0 可能退化；因此提供多级
        pipelines = [
            (f"低延迟({decoder_name})", f"{base_src} ! rtph264depay ! h264parse ! queue max-size-buffers=1 leaky=downstream ! {decoder_chain} ! video/x-raw,format=BGR ! {appsink_part}"),
            (f"基础({decoder_name})", f"{base_src} ! rtph264depay ! {decoder_chain} ! video/x-raw,format=BGR ! {appsink_part}"),
            (f"最小({decoder_name})", f"{base_src} ! rtph264depay ! {decoder_chain} ! {appsink_part}"),
        ]

        # 额外尝试 latency=0 极限模式
        zero_latency_src = f"rtspsrc location={self.rtsp_url}{proto} latency=0 buffer-mode=0"
        pipelines.append((f"极限0ms({decoder_name})", f"{zero_latency_src} ! rtph264depay ! h264parse ! {decoder_chain} ! video/x-raw,format=BGR ! {appsink_part}"))

        return pipelines
        
    def _create_opencv_backend(self):
        """创建OpenCV后端（备用方案）"""
        return self.rtsp_url
        
    def _connect_stream(self):
        """连接RTSP流"""
        max_retries = 5
        retry_count = 0
        
        while retry_count < max_retries and self.running:
            try:
                if self.use_gstreamer:
                    # 尝试多种GStreamer管道
                    pipelines = self._create_gstreamer_pipeline()
                    for i, (desc, pipeline) in enumerate(pipelines, start=1):
                        try:
                            rospy.loginfo(f"尝试GStreamer管道[{i}/{len(pipelines)}] {desc}: {pipeline}")
                            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                            if self.cap is None or not self.cap.isOpened():
                                rospy.logwarn(f"{desc} 初始化失败 (打不开 VideoCapture)")
                                continue
                            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                            # 读取多次, 给硬件解码预热
                            warm_ok = False
                            for test_attempt in range(6):
                                ret, frame = self.cap.read()
                                if ret and frame is not None and frame.size > 0:
                                    w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                                    h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                                    fps = self.cap.get(cv2.CAP_PROP_FPS)
                                    rospy.loginfo(f"连接成功[{desc}] 分辨率: {w}x{h} FPS(报告): {fps:.2f}")
                                    warm_ok = True
                                    return True
                                time.sleep(0.08)
                            if not warm_ok:
                                rospy.logwarn(f"{desc} 无法读取有效帧")
                                self.cap.release()
                        except Exception as e:
                            rospy.logwarn(f"{desc} 异常: {e}")
                            if self.cap:
                                self.cap.release()
                                self.cap = None
                            continue
                    rospy.logwarn("所有GStreamer管道都失败")
                    # rospy.logwarn("所有GStreamer管道都失败，尝试OpenCV后端")
                    # self.use_gstreamer = False
                    
                if not self.use_gstreamer:
                    # 使用OpenCV默认后端
                    rospy.loginfo(f"尝试使用OpenCV后端连接: {self.rtsp_url}")
                    
                    # 尝试不同的OpenCV后端
                    backends = [cv2.CAP_FFMPEG, cv2.CAP_GSTREAMER, cv2.CAP_ANY]
                    backend_names = ["FFMPEG", "GSTREAMER", "ANY"]
                    
                    for backend, name in zip(backends, backend_names):
                        try:
                            rospy.loginfo(f"尝试OpenCV {name}后端")
                            self.cap = cv2.VideoCapture(self.rtsp_url, backend)
                            
                            if self.cap is None or not self.cap.isOpened():
                                rospy.logwarn(f"OpenCV {name}后端初始化失败")
                                continue
                            
                            # 设置参数
                            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                            self.cap.set(cv2.CAP_PROP_FPS, self.fps_limit)
                            
                            # 测试读取
                            for test_attempt in range(5):  # 多尝试几次
                                ret, frame = self.cap.read()
                                if ret and frame is not None:
                                    rospy.loginfo(f"成功连接RTSP流 ({name}后端)，帧大小: {frame.shape}")
                                    return True
                                time.sleep(0.2)
                            
                            rospy.logwarn(f"OpenCV {name}后端无法读取帧")
                            self.cap.release()
                            
                        except Exception as e:
                            rospy.logwarn(f"OpenCV {name}后端异常: {e}")
                            if self.cap:
                                self.cap.release()
                                self.cap = None
                            continue
                
                raise Exception("所有连接方法都失败")
                    
            except Exception as e:
                retry_count += 1
                rospy.logwarn(f"连接失败 (尝试 {retry_count}/{max_retries}): {e}")
                
                if self.cap:
                    self.cap.release()
                    self.cap = None
                
                if retry_count < max_retries:
                    rospy.loginfo(f"等待 {self.reconnect_delay} 秒后重试...")
                    time.sleep(self.reconnect_delay)
        
        rospy.logerr("无法连接到RTSP流")
        return False
        
    def _capture_frames(self):
        """帧捕获线程"""
        while self.running:
            if not self._connect_stream():
                rospy.logerr("无法建立RTSP连接，退出捕获线程")
                break
                
            consecutive_failures = 0
            max_failures = 10
            
            while self.running and consecutive_failures < max_failures:
                try:
                    ret, frame = self.cap.read()
                    
                    if ret and frame is not None:
                        consecutive_failures = 0
                        current_time = time.time()
                        
                        # 只保留最新的帧，丢弃旧帧
                        try:
                            # 非阻塞方式清空队列
                            while not self.frame_queue.empty():
                                try:
                                    self.frame_queue.get_nowait()
                                except queue.Empty:
                                    break
                            
                            # 添加新帧
                            self.frame_queue.put((frame, current_time), block=False)
                            
                        except queue.Full:
                            # 队列满时跳过当前帧
                            pass
                            
                    else:
                        consecutive_failures += 1
                        rospy.logwarn(f"读取帧失败 ({consecutive_failures}/{max_failures})")
                        time.sleep(0.01)
                        
                except Exception as e:
                    consecutive_failures += 1
                    rospy.logwarn(f"捕获异常: {e} ({consecutive_failures}/{max_failures})")
                    time.sleep(0.01)
            
            # 连接丢失，尝试重连
            if self.running:
                rospy.logwarn("RTSP连接丢失，尝试重连...")
                if self.cap:
                    self.cap.release()
                    self.cap = None
                time.sleep(self.reconnect_delay)
        
        if self.cap:
            self.cap.release()
            rospy.loginfo("视频捕获线程已退出")
            
    def _publish_frames(self):
        """帧发布线程"""
        frame_interval = 1.0 / self.fps_limit
        last_publish_time = 0
        
        while self.running:
            try:
                # 等待新帧
                frame_data = self.frame_queue.get(timeout=1.0)
                frame, capture_time = frame_data
                
                current_time = time.time()
                
                # 控制发布帧率
                if current_time - last_publish_time < frame_interval:
                    continue
                
                # 转换为ROS Image消息
                try:
                    # 创建Header
                    header = Header()
                    header.stamp = rospy.Time.now()
                    header.frame_id = "gimbal_camera"
                    
                    # 转换图像
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image.header = header
                    
                    # 发布图像
                    self.image_pub.publish(ros_image)
                    
                    last_publish_time = current_time
                    self.frame_count += 1
                    
                    # 计算延迟
                    latency = current_time - capture_time
                    self.total_latency += latency
                    self.latency_samples += 1
                    
                    # 更新FPS统计
                    self._update_fps_stats()
                    
                except Exception as e:
                    rospy.logwarn(f"图像转换或发布失败: {e}")
                    
            except queue.Empty:
                # 超时，继续循环
                continue
            except Exception as e:
                rospy.logwarn(f"发布线程异常: {e}")
                
        rospy.loginfo("帧发布线程已退出")
        
    def _update_fps_stats(self):
        """更新FPS统计"""
        current_time = time.time()
        if current_time - self.last_fps_time >= 5.0:  # 每5秒输出一次统计
            elapsed_time = current_time - self.last_fps_time
            self.current_fps = self.frame_count / elapsed_time
            
            avg_latency = 0
            if self.latency_samples > 0:
                avg_latency = self.total_latency / self.latency_samples
            
            rospy.loginfo(f"性能统计 - FPS: {self.current_fps:.2f}, "
                         f"平均延迟: {avg_latency*1000:.2f}ms, "
                         f"发布帧数: {self.frame_count}")
            
            # 重置统计
            self.frame_count = 0
            self.last_fps_time = current_time
            self.total_latency = 0
            self.latency_samples = 0
            
    def start(self):
        """启动RTSP流处理"""
        rospy.loginfo("启动RTSP流处理器...")
        
        # 启动捕获线程
        self.capture_thread = threading.Thread(target=self._capture_frames, daemon=True)
        self.capture_thread.start()
        
        # 启动发布线程
        self.publish_thread = threading.Thread(target=self._publish_frames, daemon=True)
        self.publish_thread.start()
        
        rospy.loginfo("RTSP流处理器已启动")
        
        # 保持主线程运行
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
            
    def stop(self):
        """停止RTSP流处理"""
        rospy.loginfo("正在停止RTSP流处理器...")
        self.running = False
        
        # 等待线程结束
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
            
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
            
        # 释放资源
        if self.cap:
            self.cap.release()
            
        rospy.loginfo("RTSP流处理器已停止")


def main():
    """主函数"""
    try:
        streamer = OptimizedRTSPStreamer()
        streamer.start()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"RTSP流处理器启动失败: {e}")


if __name__ == '__main__':
    main()