#!/bin/bash

# Environment setup script for DJI EP workspace with PX4 and Prometheus integration
# This script ensures proper loading order of ROS environments

echo "Setting up DJI EP workspace environment..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load configuration from config file if it exists
CONFIG_FILE="${SCRIPT_DIR}/workspace_config.env"
if [ -f "${CONFIG_FILE}" ]; then
    echo "Loading configuration from: ${CONFIG_FILE}"
    source "${CONFIG_FILE}"
else
    echo "No configuration file found, using default paths..."
fi

# Define workspace paths using environment variables for flexibility
# These can be overridden by the config file
export USER_HOME="${USER_HOME:-${HOME}}"
export USER_WORKSPACE="${USER_WORKSPACE:-${USER_HOME}/git/me}"
export DJI_EP_WS="${DJI_EP_WS:-${USER_WORKSPACE}/Dji_EP_ws}"
export PROMETHEUS_PATH="${PROMETHEUS_PATH:-${USER_HOME}/pixhawk4}"
export PX4_PATH="${PX4_PATH:-${USER_HOME}/pixhawk4}"

# Remove trailing slashes to avoid double slashes
PROMETHEUS_PATH="${PROMETHEUS_PATH%/}"
PX4_PATH="${PX4_PATH%/}"
export PROMETHEUS_MAVROS_PATH="${PROMETHEUS_MAVROS_PATH:-${USER_HOME}/prometheus_mavros}"

echo "Workspace paths:"
echo "  DJI EP Workspace: ${DJI_EP_WS}"
echo "  Prometheus Path: ${PROMETHEUS_PATH}"
echo "  PX4 Path: ${PX4_PATH}"
echo "  Prometheus MAVROS: ${PROMETHEUS_MAVROS_PATH}"
echo ""

# First, source the base ROS environment
echo "Loading base ROS Noetic environment..."
source /opt/ros/noetic/setup.bash

# PX4 and Prometheus environment setup
echo "Loading PX4 and Prometheus environments..."

# Source prometheus_mavros environment
if [ -f "${PROMETHEUS_MAVROS_PATH}/devel/setup.bash" ]; then
    echo "  - Loading Prometheus MAVROS from: ${PROMETHEUS_MAVROS_PATH}/devel/setup.bash"
    source "${PROMETHEUS_MAVROS_PATH}/devel/setup.bash"
else
    echo "  - WARNING: Prometheus MAVROS setup.bash not found at: ${PROMETHEUS_MAVROS_PATH}/devel/setup.bash"
fi

# Source Prometheus environment
if [ -f "${PROMETHEUS_PATH}/Prometheus/devel/setup.bash" ]; then
    echo "  - Loading Prometheus from: ${PROMETHEUS_PATH}/Prometheus/devel/setup.bash"
    source "${PROMETHEUS_PATH}/Prometheus/devel/setup.bash"
else
    echo "  - WARNING: Prometheus setup.bash not found at: ${PROMETHEUS_PATH}/Prometheus/devel/setup.bash"
fi

# Source PX4 Gazebo setup
if [ -f "${PX4_PATH}/prometheus_px4/Tools/setup_gazebo.bash" ]; then
    echo "  - Loading PX4 Gazebo setup from: ${PX4_PATH}/prometheus_px4/Tools/setup_gazebo.bash"
    source "${PX4_PATH}/prometheus_px4/Tools/setup_gazebo.bash" "${PX4_PATH}/prometheus_px4" "${PX4_PATH}/prometheus_px4/build/amovlab_sitl_default"
else
    echo "  - WARNING: PX4 Gazebo setup.bash not found at: ${PX4_PATH}/prometheus_px4/Tools/setup_gazebo.bash"
fi

# Add PX4 to ROS package path
echo "  - Adding PX4 packages to ROS_PACKAGE_PATH"
export ROS_PACKAGE_PATH="${PX4_PATH}/prometheus_px4:$ROS_PACKAGE_PATH"
export ROS_PACKAGE_PATH="${PX4_PATH}/prometheus_px4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"

# Set up Gazebo environment variables
echo "Setting up Gazebo environment variables..."
export GAZEBO_PLUGIN_PATH="$GAZEBO_PLUGIN_PATH:${PROMETHEUS_PATH}/Prometheus/devel/lib"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${PROMETHEUS_PATH}/Prometheus/Simulator/gazebo_simulator/gazebo_models/uav_models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${PROMETHEUS_PATH}/Prometheus/Simulator/gazebo_simulator/gazebo_models/ugv_models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${PROMETHEUS_PATH}/Prometheus/Simulator/gazebo_simulator/gazebo_models/sensor_objects"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${PROMETHEUS_PATH}/Prometheus/Simulator/gazebo_simulator/gazebo_models/scene_models"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH:${PROMETHEUS_PATH}/Prometheus/Simulator/gazebo_simulator/gazebo_models/texture"

# Finally, source the DJI EP workspace (this should be last to maintain access to all packages)
if [ -f "${DJI_EP_WS}/devel/setup.bash" ]; then
    echo "Loading DJI EP workspace from: ${DJI_EP_WS}/devel/setup.bash"
    source "${DJI_EP_WS}/devel/setup.bash"
else
    echo "WARNING: DJI EP workspace setup.bash not found at: ${DJI_EP_WS}/devel/setup.bash"
    echo "Make sure you have built your workspace with: catkin_make or catkin build"
fi

# Re-add PX4 paths after sourcing workspace (they might get overridden)
echo "  - Re-adding PX4 packages to ROS_PACKAGE_PATH (post-workspace)"
export ROS_PACKAGE_PATH="${PX4_PATH}/prometheus_px4:${PX4_PATH}/prometheus_px4/Tools/sitl_gazebo:$ROS_PACKAGE_PATH"

echo ""
echo "Environment setup complete!"
echo "Available workspaces:"
echo "  - Base ROS Noetic: /opt/ros/noetic"
echo "  - Prometheus MAVROS: ${PROMETHEUS_MAVROS_PATH}/devel"
echo "  - Prometheus: ${PROMETHEUS_PATH}/Prometheus/devel"
echo "  - DJI EP Workspace: ${DJI_EP_WS}/devel"
echo ""
echo "PX4 package path: ${PX4_PATH}/prometheus_px4"
echo ""

# Verify critical packages are available
echo "Verifying package availability:"
echo "  - Checking PX4 package paths..."
echo "    PX4 main path: ${PX4_PATH}/prometheus_px4"
echo "    PX4 in ROS_PACKAGE_PATH: $(echo $ROS_PACKAGE_PATH | grep -o "${PX4_PATH}/prometheus_px4" || echo "NOT FOUND")"

if rospack find px4 >/dev/null 2>&1; then
    echo "  ✓ px4 package found at: $(rospack find px4)"
else
    echo "  ✗ px4 package NOT found"
    echo "    - Manually checking if package.xml exists: $(ls -la ${PX4_PATH}/prometheus_px4/package.xml 2>/dev/null || echo "NOT FOUND")"
fi

if rospack find prometheus_gazebo >/dev/null 2>&1; then
    echo "  ✓ prometheus_gazebo package found at: $(rospack find prometheus_gazebo)"
else
    echo "  ✗ prometheus_gazebo package NOT found"
fi

if rospack find sim_pkg >/dev/null 2>&1; then
    echo "  ✓ sim_pkg package found at: $(rospack find sim_pkg)"
else
    echo "  ✗ sim_pkg package NOT found"
fi

echo ""
echo "ROS_PACKAGE_PATH includes all necessary paths"
echo "You can now run: ./start_simulation.bash"
