#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
autonomous_takeoff_demo.py

无手柄自主起飞演示脚本
使用修改后的flight_manager实现自主起飞

使用方法：
1. 先运行仿真环境
2. 运行参数配置脚本：./scripts/setup_autonomous_flight.bash
3. 运行此脚本：python3 scripts/autonomous_takeoff_demo.py
"""

import rospy
import sys
import os
import time

# 添加flight_manager模块路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)  # 同一目录下

try:
    from flight_manager import FlightManager
except ImportError as e:
    print(f"错误：无法导入FlightManager模块: {e}")
    print(f"请确保flight_manager.py文件存在于: {current_dir}")
    sys.exit(1)


def autonomous_flight_demo():
    """
    自主飞行演示
    """
    print("=" * 50)
    print("无手柄自主起飞演示")
    print("=" * 50)
    
    # 初始化ROS节点
    rospy.init_node("autonomous_takeoff_demo", anonymous=True)
    
    # 创建飞行管理器
    uav_ns = "uav1"
    print(f"初始化飞行管理器 - 无人机: {uav_ns}")
    fm = FlightManager(uav_ns=uav_ns, wait_forever=False)
    
    try:
        print("\n步骤1: 等待系统就绪...")
        rospy.sleep(3.0)
        
        print("步骤2: 解锁无人机...")
        if not fm.arm():
            print("❌ 解锁失败！")
            return False
        print("✅ 无人机已解锁")
        
        print("步骤3: 切换到OFFBOARD模式...")
        if not fm.enter_offboard():
            print("❌ 切换到OFFBOARD模式失败！")
            return False
        print("✅ 已切换到OFFBOARD模式")
        
        print("步骤4: 执行起飞...")
        target_height = 1.2
        if not fm.takeoff(target_height, timeout=15.0):
            print("❌ 起飞失败！")
            return False
        
        current_pos = fm.current_position
        print(f"✅ 起飞成功！当前位置: ({current_pos[0]:.2f}, {current_pos[1]:.2f}, {current_pos[2]:.2f})")
        
        print("步骤5: 悬停测试...")
        fm.hold(5.0)
        print("✅ 悬停完成")
        
        print("步骤6: 移动测试...")
        x, y, z = fm.current_position
        target_x, target_y = x + 1.0, y + 1.0
        print(f"移动到: ({target_x:.2f}, {target_y:.2f}, {z:.2f})")
        
        if fm.goto(target_x, target_y, z, 0.0, timeout=15.0):
            print("✅ 移动成功")
            fm.hold(3.0)
            
            # 返回起始位置
            print("返回起始位置...")
            fm.goto(x, y, z, 0.0, timeout=15.0)
        else:
            print("❌ 移动失败")
        
        print("步骤7: 执行降落...")
        if not fm.land(0.15,timeout=15.0):
            print("❌ 降落失败！")
            return False
        print("✅ 降落成功")
        
        print("步骤8: 上锁无人机...")
        fm.disarm()
        print("✅ 无人机已上锁")
        
        print("\n" + "=" * 50)
        print("🎉 自主飞行演示完成！")
        print("=" * 50)
        return True
        
    except KeyboardInterrupt:
        print("\n❌ 用户中断操作")
        return False
    except Exception as e:
        print(f"\n❌ 演示过程中发生异常: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        print("\n关闭飞行管理器...")
        fm.shutdown()


def check_prerequisites():
    """
    检查运行前提条件
    """
    print("检查运行前提条件...")
    
    # 检查ROS环境
    if not os.environ.get('ROS_MASTER_URI'):
        print("❌ ROS环境未设置，请先source ROS环境")
        return False
    
    # 检查MAVROS连接
    try:
        from mavros_msgs.msg import State
        state = rospy.wait_for_message("/uav1/mavros/state", State, timeout=5.0)
        if state.connected:
            print("✅ MAVROS连接正常")
        else:
            print("❌ MAVROS未连接到飞控，请检查仿真环境")
            return False
    except:
        print("❌ MAVROS未连接，请先启动仿真环境")
        print("   运行: cd /home/ymc/git/me/Dji_EP_ws && ./start_simulation.bash")
        return False
    
    return True


if __name__ == "__main__":
    # if not check_prerequisites():
        # print("\n请先解决上述问题，然后重新运行此脚本")
        # sys.exit(1)
    
    print("\n" + "=" * 60)
    print("重要提示：")
    print("1. 请确保已经运行了仿真环境")
    print("2. 请确保已经运行了参数配置脚本:")
    print("   ./scripts/setup_autonomous_flight.bash")
    print("3. 按 Ctrl+C 可以随时中断飞行")
    print("=" * 60)
    
    input("\n按回车键开始自主飞行演示...")
    
    success = autonomous_flight_demo()
    if success:
        print("\n✅ 演示成功完成！")
    else:
        print("\n❌ 演示失败，请检查错误信息")
