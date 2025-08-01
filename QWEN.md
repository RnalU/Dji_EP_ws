# Qwen Code Customization

This file contains custom instructions and context for Qwen Code to improve our interactions.

## Project Overview

This is a ROS workspace for controlling a DJI RoboMaster EP robot. The workspace contains three main packages:

1. `driver_pkg`: Contains the core driver code for communicating with the robot.
2. `ep_start`: Contains startup scripts and configurations.
3. `robot_contral`: Contains control logic for the robot.

## Key Directories

- `src/`: Source code for all packages
- `build/`: Build artifacts
- `devel/`: Development environment setup
- `logs/`: Log files

## Common Tasks

- Building the workspace: `catkin build`
- Sourcing the environment: `source devel/setup.bash`
- Running a launch file: `roslaunch <package_name> <launch_file>`