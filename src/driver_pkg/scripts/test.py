import time
from robomaster import robot, led
from robomaster import config


class EPRobot():
    def __init__(self, robot_conn_type="sta",robot_protocol_type="udp", robot_sn_id="3JKDH5D00169PE", robot_ip=""):
        # 机器人对象
        self.ep_robot = robot.Robot()

        # 连接设定
        self.conn_type = robot_conn_type
        self.protocol_type = robot_protocol_type
        self.sn_id = robot_sn_id
        self.init_complete = None

        # 模块对象
        self.ep_led = None
        self.ep_chassis = None
        self.ep_battery = None
        
    
    def connect(self):
        try:
            print(f"Params set: robot_conn_type={self.conn_type}\nrobot_protocol_type={self.protocol_type}\nConnecting to robot...")
            # 初始化机器人类 
            self.ep_robot.initialize(conn_type=self.conn_type)
            self.ep_version = self.ep_robot.get_version()
            self.ep_sn = self.ep_robot.get_sn()
            print("Robot connect success!\nconnect type: {}\nVersion: {}\nSN:{}\nprotocol:{}"
                                .format(self.conn_type, self.ep_version, self.ep_sn, self.protocol_type))
            self.init_complete = True

            self.ep_led = self.ep_robot.led
            self.ep_chassis = self.ep_robot.chassis
            self.ep_battery = self.ep_robot.battery

        except Exception as e:
            print("Robot connect failed!")
            self.init_complete = False
            return False    
    def run_robot_control_thread(self):
        if self.init_complete:
            print("Robot run")
            

    def close(self):
        if self.init_complete:
            print("Robot close")
            self.ep_robot.close()

    def robot_control(self):
        self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=0, b=255, effect=led.EFFECT_ON)
        # self.ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=2, y=0, z=0, xy_speed=1, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=90, xy_speed=0, z_speed=90).wait_for_completed()
        # self.ep_chassis.move(x=1.9, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=90, xy_speed=0, z_speed=90).wait_for_completed()
        # self.ep_chassis.move(x=1.6, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=-90, xy_speed=0, z_speed=90).wait_for_completed()
        # self.ep_chassis.move(x=1.3, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=-90, xy_speed=0, z_speed=90).wait_for_completed()
        # self.ep_chassis.move(x=2.7, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=90, xy_speed=0, z_speed=90).wait_for_completed()
        # self.ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        # self.ep_chassis.move(x=0, y=0, z=90, xy_speed=0, z_speed=90).wait_for_completed()

        self.ep_chassis.move(x=0.5, y=0, z=0, xy_speed=0.7, ).wait_for_completed()
        self.ep_chassis.move(x=1, y=-1, z=0, xy_speed=0.7, ).wait_for_completed()
        self.ep_chassis.move(x=-1, y=1, z=0, xy_speed=0.7, ).wait_for_completed()
        self.ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.7, ).wait_for_completed()

        self.ep_led.set_led(comp=led.COMP_ALL, r=0, g=255, b=0, effect=led.EFFECT_ON)
        
        

if __name__ == '__main__':
    ep_robot = EPRobot()
    ep_robot.connect()
    ep_robot.robot_control()
    ep_robot.close()


        