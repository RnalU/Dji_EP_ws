# 无手柄自主起飞解决方案

## 问题描述

在普罗米修斯PX4仿真平台中，默认情况下需要连接手柄并将SWB开关拨到相应位置才能切换到OFFBOARD模式进行自主飞行。这个解决方案可以让你在仿真环境中**完全脱离手柄**实现自主起飞。

## 解决原理

通过修改PX4的关键参数来绕过RC（遥控器）检查：

1. **禁用RC失控保护** - 让PX4在没有RC输入时仍然可以正常工作
2. **修改ARM检查** - 允许在没有GPS和安全开关的情况下解锁
3. **配置OFFBOARD模式** - 设置OFFBOARD模式下的RC失控处理策略
4. **优化安全参数** - 适配仿真环境的安全设置

## 使用方法

### 步骤1: 启动仿真环境

```bash
# 在你的工作空间根目录
cd /home/ymc/git/me/Dji_EP_ws

# 启动仿真环境
./start_simulation.bash
```

等待所有组件启动完成（约10-15秒）。

### 步骤2: 配置无手柄参数

在新的终端中运行：

```bash
# 进入工作空间
cd /home/ymc/git/me/Dji_EP_ws

# 运行参数配置脚本
./scripts/setup_autonomous_flight.bash
```

脚本会自动设置所有必要的PX4参数。

### 步骤3: 运行自主飞行演示

```bash
# 运行演示脚本
python3 scripts/autonomous_takeoff_demo.py
```

或者手动在Python中使用：

```python
#!/usr/bin/env python3
import rospy
import sys
import os

# 添加路径
sys.path.append('/home/ymc/git/me/Dji_EP_ws/src/drone-control-start/scripts')
from flight_manager import FlightManager

# 初始化
rospy.init_node("my_autonomous_flight")
fm = FlightManager(uav_ns="uav1")

# 自主起飞序列
print("开始自主起飞...")
fm.arm()                    # 解锁
fm.enter_offboard()         # 进入OFFBOARD模式  
fm.takeoff(1.2)             # 起飞到1.2米
print("起飞成功！")

# 执行飞行任务
fm.goto(1.0, 1.0, 1.2, 0.0) # 移动到(1,1,1.2)
fm.hold(3.0)                # 悬停3秒

# 降落
fm.land()                   # 降落
fm.disarm()                 # 上锁
fm.shutdown()               # 关闭
```

## 核心参数说明

以下是配置脚本修改的关键参数：

| 参数名 | 值 | 说明 |
|--------|----|----- |
| `NAV_RCL_ACT` | 0 | 禁用RC失控时的自动动作 |
| `COM_RC_IN_MODE` | 1 | RC仅用于模式切换，不用于控制 |
| `COM_RCL_EXCEPT` | 4 | 在OFFBOARD模式下忽略RC失控 |
| `COM_ARM_WO_GPS` | 1 | 允许无GPS解锁 |
| `CBRK_IO_SAFETY` | 22027 | 禁用安全开关检查 |
| `COM_OF_LOSS_T` | 1.0 | OFFBOARD信号丢失超时时间 |
| `COM_OBL_ACT` | 0 | OFFBOARD丢失时切换到位置模式 |

## 安全注意事项

⚠️ **重要警告**：此解决方案仅适用于仿真环境！

- ✅ **仿真环境**：完全安全，可以随意使用
- ❌ **真实无人机**：不要在真实硬件上使用这些参数设置
- 🔄 **参数恢复**：仿真重启后参数会自动恢复默认值

## 故障排除

### 问题1: MAVROS连接失败
```bash
# 检查MAVROS状态
rostopic echo /uav1/mavros/state

# 如果没有输出，重启仿真环境
```

### 问题2: 解锁失败
```bash
# 检查当前参数是否设置成功
rosservice call /uav1/mavros/param/get "param_id: 'COM_ARM_WO_GPS'"

# 重新运行参数配置脚本
./scripts/setup_autonomous_flight.bash
```

### 问题3: OFFBOARD模式切换失败
```bash
# 检查RC相关参数
rosservice call /uav1/mavros/param/get "param_id: 'COM_RCL_EXCEPT'"

# 确保值为4
```

### 问题4: 起飞后无法控制
检查`flight_manager.py`中的setpoint发布是否正常：

```bash
# 查看position setpoint
rostopic echo /uav1/mavros/setpoint_position/local
```

## 扩展使用

### 多无人机支持

修改脚本支持多无人机：

```bash
# 为uav2配置参数
./scripts/setup_autonomous_flight.bash uav2
```

```python
# Python中使用
fm2 = FlightManager(uav_ns="uav2")
```

### 自定义飞行任务

```python
# 复杂飞行路径
waypoints = [
    (0, 0, 1.5, 0),    # 起飞点
    (2, 0, 1.5, 90),   # 向前2米，转向90度
    (2, 2, 1.5, 180),  # 向右2米，转向180度
    (0, 2, 1.5, 270),  # 向后2米，转向270度
    (0, 0, 1.5, 0),    # 回到起飞点
]

for x, y, z, yaw in waypoints:
    fm.goto(x, y, z, yaw, timeout=20.0)
    fm.hold(2.0)  # 每个点悬停2秒
```

## 技术原理

### 为什么需要手柄？

PX4默认有以下安全检查：
1. **RC失控保护**：检测到RC信号丢失时执行保护动作
2. **模式切换安全**：某些模式切换需要RC确认
3. **ARM安全检查**：解锁时需要检查各种传感器状态

### 解决方案原理

通过修改参数，我们：
1. **绕过RC检查**：让PX4认为RC失控是正常状态
2. **简化ARM流程**：跳过不必要的传感器检查
3. **优化OFFBOARD**：配置OFFBOARD模式的异常处理

这样就可以在仿真环境中实现完全自主的飞行控制。

## 总结

使用这个解决方案，你可以：

✅ **完全脱离手柄**进行仿真飞行  
✅ **简化开发流程**，专注于算法开发  
✅ **保持原有API**，无需修改现有代码  
✅ **安全可靠**，仅影响仿真环境  

这个方案特别适合：
- 自主导航算法开发
- 机器学习训练
- 大规模仿真测试
- 无人机群体仿真

现在你可以完全专注于无人机的智能算法开发，而不用担心手柄的限制！
