#!/bin/bash

# 测试脚本，用于加载环境并启动单个launch文件
echo "开始测试环境设置和launch文件..."

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 加载环境
echo "加载环境变量..."
source "${SCRIPT_DIR}/setup_env.bash"

# 验证PX4路径是否正确添加到ROS_PACKAGE_PATH
echo "验证ROS_PACKAGE_PATH中的PX4路径..."
if [[ "$ROS_PACKAGE_PATH" == *"/home/ymc/pixhawk4/prometheus_px4"* ]]; then
    echo "✓ PX4路径已正确添加到ROS_PACKAGE_PATH"
else
    echo "✗ PX4路径未添加到ROS_PACKAGE_PATH！"
    echo "当前ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
    exit 1
fi

# 验证px4包是否可找到
echo "验证px4包可用性..."
if rospack find px4 >/dev/null 2>&1; then
    echo "✓ px4包已找到: $(rospack find px4)"
else
    echo "✗ px4包未找到！"
    exit 1
fi

# 尝试启动launch文件
echo "尝试启动launch文件..."
echo "按下Enter键继续启动launch文件，或按Ctrl+C取消..."
read

# 启动launch文件
roslaunch sim_pkg all_simulate_gazebo.launch
