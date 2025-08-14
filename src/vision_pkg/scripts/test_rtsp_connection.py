#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单的RTSP连接测试脚本
用于快速验证OpenCV能否连接到RTSP流
"""

import cv2
import time
import sys

def test_rtsp_connection(rtsp_url):
    """测试RTSP连接"""
    print(f"测试RTSP连接: {rtsp_url}")
    print("="*50)
    
    # 测试不同的OpenCV后端
    backends = [
        (cv2.CAP_FFMPEG, "FFMPEG"),
        (cv2.CAP_GSTREAMER, "GSTREAMER"), 
        (cv2.CAP_ANY, "ANY")
    ]
    
    for backend, name in backends:
        print(f"\n测试 OpenCV {name} 后端...")
        try:
            cap = cv2.VideoCapture(rtsp_url, backend)
            
            if not cap.isOpened():
                print(f"  ❌ {name}后端无法打开流")
                continue
            
            print(f"  ✅ {name}后端成功打开流")
            
            # 尝试读取几帧
            success_count = 0
            for i in range(10):
                ret, frame = cap.read()
                if ret and frame is not None:
                    success_count += 1
                    if i == 0:
                        print(f"  📐 帧尺寸: {frame.shape}")
                time.sleep(0.1)
            
            print(f"  📊 成功读取: {success_count}/10 帧")
            
            cap.release()
            
            if success_count > 0:
                print(f"  🎉 {name}后端工作正常！")
                return backend, name
            else:
                print(f"  ⚠️  {name}后端无法读取帧")
                
        except Exception as e:
            print(f"  ❌ {name}后端异常: {e}")
    
    print("\n❌ 所有后端都失败")
    return None, None

def test_gstreamer_pipeline(rtsp_url):
    """测试GStreamer管道"""
    print(f"\n测试GStreamer管道...")
    print("="*50)
    
    # 不同复杂度的管道
    pipelines = [
        # 简单管道
        f"rtspsrc location={rtsp_url} ! rtph264depay ! avdec_h264 ! videoconvert ! appsink",
        
        # 带参数的管道
        f"rtspsrc location={rtsp_url} latency=0 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink",
        
        # 完整优化管道
        f"rtspsrc location={rtsp_url} latency=0 buffer-mode=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! videoscale ! video/x-raw,format=BGR ! appsink emit-signals=true sync=false max-buffers=1 drop=true"
    ]
    
    for i, pipeline in enumerate(pipelines, 1):
        print(f"\n测试管道 {i}: {pipeline}")
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if not cap.isOpened():
                print(f"  ❌ 管道 {i} 无法打开")
                continue
            
            print(f"  ✅ 管道 {i} 成功打开")
            
            # 尝试读取几帧
            success_count = 0
            for j in range(5):
                ret, frame = cap.read()
                if ret and frame is not None:
                    success_count += 1
                    if j == 0:
                        print(f"  📐 帧尺寸: {frame.shape}")
                time.sleep(0.2)
            
            print(f"  📊 成功读取: {success_count}/5 帧")
            cap.release()
            
            if success_count > 0:
                print(f"  🎉 管道 {i} 工作正常！")
                return pipeline
            else:
                print(f"  ⚠️  管道 {i} 无法读取帧")
                
        except Exception as e:
            print(f"  ❌ 管道 {i} 异常: {e}")
    
    print("\n❌ 所有GStreamer管道都失败")
    return None

def main():
    if len(sys.argv) > 1:
        rtsp_url = sys.argv[1]
    else:
        rtsp_url = "rtsp://192.168.14.253:8554/live"
    
    print("RTSP连接测试工具")
    print("="*50)
    print(f"目标URL: {rtsp_url}")
    
    # 检查OpenCV版本和编译信息
    print(f"\nOpenCV版本: {cv2.__version__}")
    build_info = cv2.getBuildInformation()
    print("GStreamer支持:", "✅" if "GStreamer" in build_info else "❌")
    print("FFMPEG支持:", "✅" if "FFMPEG" in build_info else "❌")
    
    # 测试OpenCV后端
    backend, backend_name = test_rtsp_connection(rtsp_url)
    
    # 测试GStreamer管道
    working_pipeline = test_gstreamer_pipeline(rtsp_url)
    
    # 总结
    print("\n" + "="*50)
    print("测试总结:")
    if backend:
        print(f"✅ 推荐使用OpenCV {backend_name}后端")
    if working_pipeline:
        print(f"✅ 推荐使用GStreamer管道")
    
    if not backend and not working_pipeline:
        print("❌ 没有找到可工作的连接方法")
        print("建议检查:")
        print("1. RTSP服务器是否正常运行")
        print("2. 网络连接是否正常")
        print("3. OpenCV是否正确编译了GStreamer/FFMPEG支持")
    
    print("="*50)

if __name__ == "__main__":
    main()
