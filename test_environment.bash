#!/bin/bash

# DJI EP Workspace Environment Test Script
# This script tests if all required packages and environments are properly configured

echo "DJI EP Environment Test"
echo "======================"
echo ""

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source the environment setup
echo "1. Loading environment setup..."
if [ -f "${SCRIPT_DIR}/setup_env.bash" ]; then
    source "${SCRIPT_DIR}/setup_env.bash"
    echo "   ✓ Environment setup completed"
else
    echo "   ✗ setup_env.bash not found in ${SCRIPT_DIR}"
    exit 1
fi

echo ""
echo "2. Testing ROS environment..."

# Test ROS core
if command -v roscore >/dev/null 2>&1; then
    echo "   ✓ ROS commands available"
else
    echo "   ✗ ROS commands not found"
    echo "     Make sure ROS is properly installed and sourced"
fi

# Test ROS_PACKAGE_PATH
if [ -n "$ROS_PACKAGE_PATH" ]; then
    echo "   ✓ ROS_PACKAGE_PATH is set"
    echo "     Path: $ROS_PACKAGE_PATH"
else
    echo "   ✗ ROS_PACKAGE_PATH is not set"
fi

echo ""
echo "3. Testing critical packages..."

# Test packages
PACKAGES=("px4" "prometheus_gazebo" "prometheus_uav_control" "prometheus_demo" "sim_pkg" "mavros")
FAILED_PACKAGES=()

for package in "${PACKAGES[@]}"; do
    if rospack find "$package" >/dev/null 2>&1; then
        PACKAGE_PATH=$(rospack find "$package")
        echo "   ✓ $package found at: $PACKAGE_PATH"
    else
        echo "   ✗ $package NOT found"
        FAILED_PACKAGES+=("$package")
    fi
done

echo ""
echo "4. Testing Gazebo environment..."

# Test Gazebo environment variables
if [ -n "$GAZEBO_MODEL_PATH" ]; then
    echo "   ✓ GAZEBO_MODEL_PATH is set"
    echo "     Path: $GAZEBO_MODEL_PATH"
else
    echo "   ✗ GAZEBO_MODEL_PATH is not set"
fi

if [ -n "$GAZEBO_PLUGIN_PATH" ]; then
    echo "   ✓ GAZEBO_PLUGIN_PATH is set"
    echo "     Path: $GAZEBO_PLUGIN_PATH"
else
    echo "   ✗ GAZEBO_PLUGIN_PATH is not set"
fi

# Test Gazebo command
if command -v gazebo >/dev/null 2>&1; then
    echo "   ✓ Gazebo command available"
else
    echo "   ✗ Gazebo command not found"
fi

echo ""
echo "5. Testing workspace structure..."

# Test workspace structure
REQUIRED_DIRS=("${DJI_EP_WS}/src" "${DJI_EP_WS}/devel" "${DJI_EP_WS}/build")
for dir in "${REQUIRED_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        echo "   ✓ Directory exists: $dir"
    else
        echo "   ✗ Directory missing: $dir"
    fi
done

# Test launch files
LAUNCH_FILES=("${DJI_EP_WS}/src/sim_pkg/launch/all_simulate_gazebo.launch")
for file in "${LAUNCH_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "   ✓ Launch file exists: $file"
    else
        echo "   ✗ Launch file missing: $file"
    fi
done

echo ""
echo "6. Testing external dependencies..."

# Test external paths
EXTERNAL_PATHS=("${PROMETHEUS_PATH}/Prometheus" "${PX4_PATH}/prometheus_px4" "${PROMETHEUS_MAVROS_PATH}")
for path in "${EXTERNAL_PATHS[@]}"; do
    if [ -d "$path" ]; then
        echo "   ✓ External path exists: $path"
    else
        echo "   ✗ External path missing: $path"
    fi
done

echo ""
echo "========================"
echo "Test Summary"
echo "========================"

if [ ${#FAILED_PACKAGES[@]} -eq 0 ]; then
    echo "✓ All critical packages found!"
else
    echo "✗ Missing packages: ${FAILED_PACKAGES[*]}"
    echo ""
    echo "To fix missing packages:"
    echo "1. Check if Prometheus and PX4 are properly installed"
    echo "2. Verify paths in workspace_config.env (if using custom config)"
    echo "3. Make sure all workspaces are built (catkin_make or catkin build)"
    echo "4. Check if all required repositories are cloned"
fi

echo ""
echo "Environment Variables Summary:"
echo "  DJI_EP_WS: ${DJI_EP_WS}"
echo "  PROMETHEUS_PATH: ${PROMETHEUS_PATH}"
echo "  PX4_PATH: ${PX4_PATH}"
echo "  PROMETHEUS_MAVROS_PATH: ${PROMETHEUS_MAVROS_PATH}"

echo ""
echo "To start simulation, run: ./start_simulation.bash"
echo "For more details, check individual package documentation"
