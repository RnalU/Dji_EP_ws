#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autonomous_flight_setup.py

无手柄自主起飞解决方案
解决普罗米修斯PX4仿真平台需要手柄才能切换OFFBOARD模式的问题

主要功能：
1. 绕过RC失控保护
2. 自动设置PX4参数以支持无RC自主飞行
3. 提供完整的自主起飞API
4. 监控飞行状态并提供安全保护

使用方法：
    from autonomous_flight_setup import AutonomousFlightController
    
    # 初始化控制器
    controller = AutonomousFlightController(uav_ns="uav1")
    
    # 设置仿真环境（绕过RC检查）
    controller.setup_simulation_environment()
    
    # 执行自主起飞
    success = controller.autonomous_takeoff(target_height=1.2)
    
    if success:
        print("自主起飞成功！")
        # 执行其他飞行任务...
        controller.autonomous_land()
"""

import rospy
import time
from typing import Optional, Dict, Any

from mavros_msgs.msg import State, RCIn, OverrideRCIn, ParamValue
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamGet
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import Joy

import sys
import os
# 同一目录下的flight_manager模块
from flight_manager import FlightManager


class AutonomousFlightController:
    """
    无手柄自主飞行控制器
    
    通过设置PX4参数和模拟RC输入来实现无手柄自主飞行
    """
    
    def __init__(self, uav_ns: str = "uav1"):
        self.uav_ns = uav_ns
        self.flight_manager = FlightManager(uav_ns=uav_ns)
        
        # RC override publisher（模拟手柄输入）
        self.rc_override_pub = rospy.Publisher(
            f"/{self.uav_ns}/mavros/rc/override", 
            OverrideRCIn, 
            queue_size=10
        )
        
        # 参数设置服务
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/param/set")
        rospy.wait_for_service(f"/{self.uav_ns}/mavros/param/get")
        self.param_set_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/param/set", ParamSet)
        self.param_get_srv = rospy.ServiceProxy(f"/{self.uav_ns}/mavros/param/get", ParamGet)
        
        rospy.loginfo(f"[AutonomousFlightController] 初始化完成，无人机命名空间: {self.uav_ns}")
    
    def setup_simulation_environment(self) -> bool:
        """
        设置仿真环境参数，绕过RC检查
        
        Returns:
            bool: 设置是否成功
        """
        rospy.loginfo("[AutonomousFlightController] 正在配置仿真环境参数...")
        
        # 关键参数配置
        params_to_set = {
            # 禁用RC失控保护
            "NAV_RCL_ACT": 0,        # RC失控时的动作：0=禁用
            "COM_RC_IN_MODE": 1,     # RC输入模式：1=仅用于模式切换，不用于控制
            "COM_RCL_EXCEPT": 4,     # RC失控异常处理：4=忽略在offboard模式下
            
            # ARM相关参数
            "COM_ARM_WO_GPS": 1,     # 允许无GPS解锁
            "CBRK_IO_SAFETY": 22027, # 禁用安全开关检查
            "COM_ARM_CHK_ESCS": 0,   # 禁用ESC检查
            
            # OFFBOARD模式参数
            "COM_OF_LOSS_T": 1.0,    # OFFBOARD信号丢失超时时间
            "COM_OBL_ACT": 0,        # OFFBOARD丢失动作：0=位置模式
            "COM_OBL_RC_ACT": 0,     # OFFBOARD模式下RC丢失动作：0=忽略
            
            # 位置控制参数
            "MPC_POS_MODE": 0,       # 位置控制模式
            "MC_AIRMODE": 0,         # 禁用空中模式（仿真中不需要）
            
            # 电池相关（仿真环境）
            "BAT_N_CELLS": 4,        # 电池节数
            "BAT_V_EMPTY": 3.2,      # 空电池电压
            "BAT_V_CHARGED": 4.2,    # 满电池电压
            
            # 安全相关
            "GF_ACTION": 0,          # 地面故障动作：0=无动作
            "LNDMC_FFALL_THR": 8.0,  # 自由落体检测阈值
        }
        
        success_count = 0
        total_params = len(params_to_set)
        
        for param_name, param_value in params_to_set.items():
            try:
                # 创建ParamValue对象
                value = ParamValue()
                if isinstance(param_value, float):
                    value.integer = 0
                    value.real = param_value
                else:
                    value.integer = int(param_value)
                    value.real = 0.0
                
                response = self.param_set_srv(param_name, value)
                if response.success:
                    rospy.loginfo(f"[参数设置] {param_name} = {param_value} ✓")
                    success_count += 1
                else:
                    rospy.logwarn(f"[参数设置] {param_name} = {param_value} ✗")
            except Exception as e:
                rospy.logerr(f"[参数设置] 设置 {param_name} 失败: {e}")
        
        rospy.loginfo(f"[AutonomousFlightController] 参数设置完成: {success_count}/{total_params}")
        
        # 模拟手柄输入
        self._simulate_rc_input()
        
        return success_count > total_params * 0.8  # 80%以上参数设置成功
    
    def _simulate_rc_input(self):
        """
        模拟手柄输入，主要是设置SWB开关到OFFBOARD位置
        """
        rospy.loginfo("[AutonomousFlightController] 模拟RC输入...")
        
        # 创建RC override消息
        rc_msg = OverrideRCIn()
        rc_msg.channels = [65535] * 18  # 初始化所有通道为无效值
        
        # 设置基本通道（油门、俯仰、滚转、偏航）到中位
        rc_msg.channels[0] = 1500  # 滚转
        rc_msg.channels[1] = 1500  # 俯仰  
        rc_msg.channels[2] = 1100  # 油门（低位，防止意外起飞）
        rc_msg.channels[3] = 1500  # 偏航
        
        # 模式切换开关 - 通常在通道5或6
        rc_msg.channels[4] = 1100  # SWA - 低位
        rc_msg.channels[5] = 2000  # SWB - 高位（OFFBOARD模式）
        rc_msg.channels[6] = 1500  # SWC - 中位
        rc_msg.channels[7] = 1500  # SWD - 中位
        
        # 发布RC override消息
        for _ in range(50):  # 发送50次确保被接收
            self.rc_override_pub.publish(rc_msg)
            rospy.sleep(0.02)
        
        rospy.loginfo("[AutonomousFlightController] RC输入模拟完成")
    
    def autonomous_takeoff(self, target_height: float = 1.2, timeout: float = 30.0) -> bool:
        """
        无手柄自主起飞
        
        Args:
            target_height: 目标起飞高度（米）
            timeout: 超时时间（秒）
            
        Returns:
            bool: 起飞是否成功
        """
        rospy.loginfo(f"[AutonomousFlightController] 开始自主起飞到高度 {target_height}m")
        
        try:
            # 1. 等待飞控连接
            rospy.loginfo("步骤1: 等待飞控连接...")
            if not self._wait_for_connection(timeout=10.0):
                rospy.logerr("飞控连接超时")
                return False
            
            # 2. 再次确保RC模拟
            self._simulate_rc_input()
            rospy.sleep(2.0)
            
            # 3. 解锁
            rospy.loginfo("步骤2: 解锁无人机...")
            if not self.flight_manager.arm():
                rospy.logerr("无人机解锁失败")
                return False
            
            # 4. 切换到OFFBOARD模式
            rospy.loginfo("步骤3: 切换到OFFBOARD模式...")
            if not self.flight_manager.enter_offboard():
                rospy.logerr("切换到OFFBOARD模式失败")
                return False
            
            # 5. 起飞
            rospy.loginfo("步骤4: 执行起飞...")
            if not self.flight_manager.takeoff(target_height, timeout=timeout):
                rospy.logerr("起飞失败")
                return False
            
            rospy.loginfo(f"[AutonomousFlightController] 自主起飞成功！当前高度: {self.flight_manager.current_position[2]:.2f}m")
            return True
            
        except Exception as e:
            rospy.logerr(f"[AutonomousFlightController] 自主起飞异常: {e}")
            return False
    
    def autonomous_land(self, timeout: float = 20.0) -> bool:
        """
        自主降落
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: 降落是否成功
        """
        rospy.loginfo("[AutonomousFlightController] 开始自主降落...")
        
        try:
            if not self.flight_manager.land(timeout=timeout):
                rospy.logerr("自主降落失败")
                return False
            
            # 降落后上锁
            rospy.loginfo("降落完成，上锁无人机...")
            self.flight_manager.disarm()
            
            rospy.loginfo("[AutonomousFlightController] 自主降落成功！")
            return True
            
        except Exception as e:
            rospy.logerr(f"[AutonomousFlightController] 自主降落异常: {e}")
            return False
    
    def _wait_for_connection(self, timeout: float = 10.0) -> bool:
        """
        等待与飞控的连接
        
        Args:
            timeout: 超时时间（秒）
            
        Returns:
            bool: 是否连接成功
        """
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
            
            # 检查MAVROS连接状态
            try:
                state = rospy.wait_for_message(f"/{self.uav_ns}/mavros/state", State, timeout=1.0)
                if state.connected:
                    rospy.loginfo("飞控连接成功")
                    return True
            except:
                pass
            
            rospy.sleep(0.1)
        
        return False
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取当前飞行状态
        
        Returns:
            Dict[str, Any]: 状态信息字典
        """
        return {
            "armed": self.flight_manager.is_armed,
            "mode": self.flight_manager.current_mode,
            "position": self.flight_manager.current_position,
            "yaw_deg": self.flight_manager.current_yaw_deg
        }
    
    def emergency_stop(self):
        """
        紧急停止
        """
        rospy.logwarn("[AutonomousFlightController] 执行紧急停止！")
        self.flight_manager.abort("Emergency stop requested")
        # 可以根据需要添加更多紧急处理逻辑
    
    def shutdown(self):
        """
        关闭控制器
        """
        rospy.loginfo("[AutonomousFlightController] 关闭控制器...")
        self.flight_manager.shutdown()


# ----------------------------------------------------------------------
# 测试和演示代码
# ----------------------------------------------------------------------
def autonomous_flight_demo():
    """
    自主飞行演示
    """
    rospy.init_node("autonomous_flight_demo", anonymous=True)
    
    # 创建控制器
    controller = AutonomousFlightController(uav_ns="uav1")
    
    try:
        # 设置仿真环境
        rospy.loginfo("=== 配置仿真环境 ===")
        if not controller.setup_simulation_environment():
            rospy.logerr("仿真环境配置失败")
            return
        
        rospy.loginfo("等待5秒让参数生效...")
        rospy.sleep(5.0)
        
        # 执行自主起飞
        rospy.loginfo("=== 开始自主起飞 ===")
        if not controller.autonomous_takeoff(target_height=1.5):
            rospy.logerr("自主起飞失败")
            return
        
        # 悬停一段时间
        rospy.loginfo("=== 悬停测试 ===")
        controller.flight_manager.hold(5.0)
        
        # 简单移动测试
        rospy.loginfo("=== 移动测试 ===")
        x, y, z = controller.flight_manager.current_position
        controller.flight_manager.goto(x + 1.0, y + 1.0, z, 0.0, timeout=15.0)
        
        controller.flight_manager.hold(3.0)
        
        # 返回起始位置
        controller.flight_manager.goto(x, y, z, 0.0, timeout=15.0)
        
        # 自主降落
        rospy.loginfo("=== 开始自主降落 ===")
        controller.autonomous_land()
        
        rospy.loginfo("=== 自主飞行演示完成 ===")
        
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
    except Exception as e:
        rospy.logerr(f"演示过程中发生异常: {e}")
    finally:
        controller.shutdown()


if __name__ == "__main__":
    autonomous_flight_demo()
