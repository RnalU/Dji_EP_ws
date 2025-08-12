#!/usr/bin/env python3

"""
测试二维码辅助定位降落功能
"""
import rospy
from flight_manager import FlightManager
from std_msgs.msg import Empty

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("qrcode_landoff_test")

    start_qr_pub = rospy.Publisher("/start_qr_detection", Empty, queue_size=1, latch=True)
    try:
        fm = FlightManager(uav_ns="uav1")

        # 飞机起飞，飞到指定位置
        fm.arm()
        fm.takeoff(1.0)
        fm.goto(1.2, 2.5, 1.0, 0)
        fm.hold(2)

        # 准备二维码降落
        fm.precision_qr_land()
        fm.disarm()
        fm.shutdown()
    except rospy.ROSInterruptException:
        pass
    finally:
        fm.shutdown()
