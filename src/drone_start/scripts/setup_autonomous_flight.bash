#!/bin/bash

# PX4仿真无手柄自主起飞配置脚本
# 通过设置PX4参数来绕过RC检查，实现无手柄自主飞行

echo "PX4仿真无手柄自主起飞配置"
echo "=============================="

# 定义无人机命名空间
UAV_NS=${1:-uav1}
echo "配置无人机: $UAV_NS"

# 等待MAVROS连接
echo "等待MAVROS连接..."
timeout 30 bash -c "until rostopic echo /${UAV_NS}/mavros/state -n 1 | grep -q connected; do sleep 1; done"

if [ $? -eq 0 ]; then
    echo "MAVROS连接成功"
else
    echo "MAVROS连接超时"
    exit 1
fi

echo "开始设置PX4参数..."

# 定义参数设置函数
set_param() {
    local param_name=$1
    local param_value=$2
    echo "设置 $param_name = $param_value"
    rosservice call /${UAV_NS}/mavros/param/set "param_id: '$param_name'
value: {integer: $param_value, real: 0.0}" > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo "  ✓ $param_name 设置成功"
    else
        echo "  ✗ $param_name 设置失败"
    fi
}

set_param_float() {
    local param_name=$1
    local param_value=$2
    echo "设置 $param_name = $param_value"
    rosservice call /${UAV_NS}/mavros/param/set "param_id: '$param_name'
value: {integer: 0, real: $param_value}" > /dev/null 2>&1
    
    if [ $? -eq 0 ]; then
        echo "  ✓ $param_name 设置成功"
    else
        echo "  ✗ $param_name 设置失败"
    fi
}

# 关键参数设置 - 这些参数允许无RC自主飞行

echo "1. 禁用RC失控保护..."
set_param "NAV_RCL_ACT" 0        # RC失控时的动作：0=禁用
set_param "COM_RC_IN_MODE" 1     # RC输入模式：1=仅用于模式切换
set_param "COM_RCL_EXCEPT" 4     # RC失控异常处理：4=忽略在offboard模式下

echo "2. 设置ARM相关参数..."
set_param "COM_ARM_WO_GPS" 1     # 允许无GPS解锁
set_param "CBRK_IO_SAFETY" 22027 # 禁用安全开关检查
set_param "COM_ARM_CHK_ESCS" 0   # 禁用ESC检查

echo "3. 设置OFFBOARD模式参数..."
set_param_float "COM_OF_LOSS_T" 1.0    # OFFBOARD信号丢失超时时间
set_param "COM_OBL_ACT" 0        # OFFBOARD丢失动作：0=位置模式
set_param "COM_OBL_RC_ACT" 0     # OFFBOARD模式下RC丢失动作：0=忽略

echo "4. 设置位置控制参数..."
set_param "MPC_POS_MODE" 0       # 位置控制模式
set_param "MC_AIRMODE" 0         # 禁用空中模式（仿真中不需要）

echo "5. 设置安全相关参数..."
set_param "GF_ACTION" 0          # 地面故障动作：0=无动作
set_param_float "LNDMC_FFALL_THR" 8.0  # 自由落体检测阈值

echo "6. 设置电池相关参数（仿真环境）..."
set_param "BAT_N_CELLS" 4        # 电池节数
set_param_float "BAT_V_EMPTY" 3.2      # 空电池电压
set_param_float "BAT_V_CHARGED" 4.2    # 满电池电压

echo ""
echo "参数设置完成！"
echo ""
echo "现在你可以使用以下Python代码进行无手柄自主起飞："
echo ""
echo "# 在Python脚本中"
echo "from flight_manager import FlightManager"
echo ""
echo "# 创建飞行管理器"
echo "fm = FlightManager(uav_ns='$UAV_NS')"
echo ""
echo "# 自主起飞序列"
echo "print('开始自主起飞...')"
echo "fm.arm()                    # 解锁"
echo "fm.enter_offboard()         # 进入OFFBOARD模式"
echo "fm.takeoff(1.2)             # 起飞到1.2米"
echo "print('起飞成功！')"
echo ""
echo "# 执行飞行任务..."
echo "fm.goto(1.0, 1.0, 1.2, 0.0) # 移动到指定位置"
echo "fm.hold(3.0)                # 悬停3秒"
echo ""
echo "# 降落和上锁"
echo "fm.land()                   # 降落"
echo "fm.disarm()                 # 上锁"
echo "fm.shutdown()               # 关闭"

echo ""
echo "=============================="
echo "配置完成！无人机现在可以无手柄自主起飞。"
