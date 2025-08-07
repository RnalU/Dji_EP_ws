import os
import sys
import time
import threading

# 添加当前目录到PATH
robomaster_path = "/home/ymc/git/me/Dji_EP_ws/src/driver_pkg/scripts"
if robomaster_path not in sys.path:
    sys.path.append(robomaster_path)

from robomaster import robot
from robomaster import config


if __name__ == '__main__':
    ep_robot = robot.Robot()
    # 指定机器人的 SN 号
    ep_robot.initialize(conn_type="sta")
    # ep_robot.initialize(conn_type="sta", sn="3JKDH5D00169PE")

    ep_version = ep_robot.get_version()
    SN = ep_robot.get_sn()
    print("Robot SN:", SN)
    # ep_led = ep_robot.led
    # ep_led.set_led(r=255, g=0, b=0, effect="on"
    
    print("Robot Version: {0}".format(ep_version))

    ep_robot.close()
