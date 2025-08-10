#!/bin/bash

# DJI EP Workspace Simulation Launcher
# This script launches the complete simulation environment with proper environment setup
# Uses environment variables for flexible path management

# Define workspace paths using environment variables
export USER_HOME="${HOME}"
export USER_WORKSPACE="${USER_HOME}/git/me"
export DJI_EP_WS="${USER_WORKSPACE}/Dji_EP_ws"

echo "DJI EP Simulation Launcher"
echo "=========================="
echo "DJI EP Workspace: ${DJI_EP_WS}"
echo "Setup script: ${DJI_EP_WS}/setup_env.bash"
echo ""

# Check if setup script exists
if [ ! -f "${DJI_EP_WS}/setup_env.bash" ]; then
    echo "ERROR: Setup script not found at: ${DJI_EP_WS}/setup_env.bash"
    echo "Please make sure the setup_env.bash script exists in your workspace."
    exit 1
fi

echo "Starting simulation with the following components:"
echo "1. ROS Core"
echo "2. Gazebo Simulation (sim_pkg)"
echo "3. UAV Control (prometheus_uav_control)"
echo "4. Takeoff/Land Demo (prometheus_demo)"
echo ""
echo "Starting in 3 seconds..."
sleep 3

gnome-terminal --window -e 'bash -c "
    echo \"Starting ROS Core...\";
    roscore;
    exec bash
"' \
--tab -e "bash -c \"
    echo 'Loading environment and starting Gazebo simulation...';
    source ${DJI_EP_WS}/setup_env.bash;
    sleep 1;
    echo 'Launching sim_pkg all_simulate_gazebo.launch...';
    roslaunch sim_pkg all_simulate_gazebo.launch;
    exec bash
\"" \
--tab -e "bash -c \"
    echo 'Loading environment and starting Gimbal simulation...';
    source ${DJI_EP_WS}/setup_env.bash;
    sleep 1;
    echo 'Launching sim_pkg gimbal_initialize.launch...';
    roslaunch sim_pkg gimbal_initialize.launch;
    exec bash
\"" \
--tab -e "bash -c \"
    echo 'Loading environment and starting UAV control...';
    source ${DJI_EP_WS}/setup_env.bash;
    sleep 2;
    echo 'Launching sim_pkg uav_control_main_indoor.launch...';
    roslaunch sim_pkg uav_control_main_indoor.launch;
    exec bash
\"" \
--tab -e "bash -c \"
    source ${DJI_EP_WS}/setup_env.bash;
    sleep 2;
    bash ${DJI_EP_WS}/src/drone_start/scripts/setup_autonomous_flight.bash;
    exec bash
\"" 

echo "Simulation started! Check the opened terminal tabs for status."
echo "If you encounter any issues, check the individual terminal tabs for error messages."
