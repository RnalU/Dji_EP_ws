# DJI EP Workspace Simulation Setup

这个文档说明如何在 DJI EP 工作空间中设置和运行基于 Prometheus 和 PX4 的无人机仿真环境。

## 📁 项目结构

```
Dji_EP_ws/
├── src/                          # ROS 包源码
│   └── sim_pkg/                  # 仿真包
├── devel/                        # 编译输出
├── build/                        # 编译文件
├── setup_env.bash               # 环境设置脚本 ⭐
├── start_simulation.bash        # 仿真启动脚本 ⭐
├── test_environment.bash        # 环境测试脚本 ⭐
├── workspace_config.env         # 配置文件模板
└── README_SIMULATION.md         # 本文档
```

## 🚀 快速开始

### 1. 环境测试
首先测试你的环境是否正确配置：

```bash
cd ~/git/me/Dji_EP_ws
./test_environment.bash
```

### 2. 启动仿真
如果环境测试通过，启动完整仿真：

```bash
./start_simulation.bash
```

## 🔧 详细配置

### 环境变量配置

脚本使用以下默认路径：
- **DJI EP 工作空间**: `~/git/me/Dji_EP_ws`
- **Prometheus 路径**: `~/pixhawk4/`
- **PX4 路径**: `~/pixhawk4/`
- **Prometheus MAVROS**: `~/prometheus_mavros/`

### 自定义路径配置

如果你的安装路径不同，可以：

#### 方法1: 修改配置文件
1. 复制配置模板：
   ```bash
   cp workspace_config.env.example workspace_config.env
   ```

2. 编辑 `workspace_config.env` 文件，修改路径：
   ```bash
   # 示例自定义配置
   USER_WORKSPACE="${HOME}/my_custom_workspace"
   PROMETHEUS_PATH="${HOME}/my_prometheus"
   PX4_PATH="${HOME}/my_px4"
   ```

#### 方法2: 设置环境变量
在运行脚本前设置环境变量：
```bash
export USER_WORKSPACE="${HOME}/my_workspace"
export PROMETHEUS_PATH="${HOME}/my_prometheus"
./start_simulation.bash
```

## 🏗️ 仿真组件

仿真系统包含以下组件，分别在不同的终端标签页中启动：

### 标签页 1: ROS Core
- 启动 ROS Master 节点
- 提供 ROS 通信基础设施

### 标签页 2: Gazebo 仿真 (sim_pkg)
- 启动 Gazebo 物理仿真环境
- 加载无人机模型和场景
- 启动 PX4 SITL (Software In The Loop)
- 启动 MAVROS 通信节点

### 标签页 3: 无人机控制 (prometheus_uav_control)
- 启动 Prometheus 无人机控制系统
- 提供高级飞行控制接口

### 标签页 4: 起飞降落演示 (prometheus_demo)
- 启动自动起飞降落演示程序
- 展示基本飞行任务

## 🔍 故障排除

### 常见问题

#### 1. "Resource not found: px4" 错误
**原因**: PX4 包路径未正确设置

**解决方案**:
```bash
# 测试 PX4 包是否可找到
source ./setup_env.bash
rospack find px4
```

如果仍然报错，检查：
- Prometheus 和 PX4 是否正确安装
- `workspace_config.env` 中的路径是否正确

#### 2. "prometheus_gazebo package not found" 错误
**原因**: Prometheus 包未正确安装或路径错误

**解决方案**:
1. 确认 Prometheus 已安装在 `${PROMETHEUS_PATH}/Prometheus/`
2. 确认该目录下有 `devel/setup.bash` 文件
3. 重新运行环境测试

#### 3. Gazebo 启动失败
**原因**: Gazebo 模型路径或插件路径错误

**解决方案**:
```bash
# 检查 Gazebo 环境变量
echo $GAZEBO_MODEL_PATH
echo $GAZEBO_PLUGIN_PATH
```

#### 4. 工作空间编译问题
如果 DJI EP 工作空间未正确编译：

```bash
cd ~/git/me/Dji_EP_ws
catkin_make
# 或者使用 catkin tools
catkin build
```

### 调试工具

#### 环境测试脚本
```bash
./test_environment.bash
```
这个脚本会检查：
- ROS 环境是否正确
- 所有必需的包是否可找到
- Gazebo 环境变量是否设置
- 工作空间结构是否完整

#### 手动环境加载
```bash
source ./setup_env.bash
# 然后手动测试各个组件
roslaunch sim_pkg all_simulate_gazebo.launch
```

## 📝 开发说明

### 添加新的仿真组件

1. 在 `src/` 目录下创建新的 ROS 包
2. 重新编译工作空间：`catkin_make`
3. 修改 `start_simulation.bash` 添加新的启动命令

### 修改仿真参数

仿真参数可以在以下位置修改：
- **无人机模型**: `src/sim_pkg/launch/sitl_px4_indoor.launch`
- **Gazebo 世界**: `src/sim_pkg/worlds/`
- **控制参数**: `src/sim_pkg/config/`

### 环境变量说明

| 变量名 | 说明 | 默认值 |
|--------|------|--------|
| `USER_WORKSPACE` | 用户工作空间根目录 | `${HOME}/git/me` |
| `DJI_EP_WS` | DJI EP 工作空间目录 | `${USER_WORKSPACE}/Dji_EP_ws` |
| `PROMETHEUS_PATH` | Prometheus 安装路径 | `${HOME}/pixhawk4` |
| `PX4_PATH` | PX4 安装路径 | `${HOME}/pixhawk4` |
| `PROMETHEUS_MAVROS_PATH` | Prometheus MAVROS 路径 | `${HOME}/prometheus_mavros` |

## 🆘 获取帮助

如果遇到问题：

1. **首先运行环境测试**: `./test_environment.bash`
2. **检查各个终端标签页的错误信息**
3. **确认所有依赖都已正确安装**
4. **检查路径配置是否正确**

## 📚 相关文档

- [Prometheus 无人机系统文档](https://github.com/amov-lab/Prometheus)
- [PX4 开发指南](https://docs.px4.io/)
- [ROS Noetic 教程](http://wiki.ros.org/noetic/Tutorials)
- [Gazebo 仿真教程](http://gazebosim.org/tutorials)

---

*最后更新: 2024年*