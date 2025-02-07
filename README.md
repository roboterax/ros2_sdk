# STAR1 Example Package

This repository contains example ROS 2 packages for STAR1 robot control, developed by RobotEra.

## Overview

This repository includes the following packages:

- `star1_example`: A metapackage containing example packages for STAR1 robot
- `simple_trajectory_server`: A trajectory server implementation for basic motion control
- `xbot_common_interfaces`: Common interface definitions used by STAR1 robot
- `rl_controller`: A real-time reinforcement learning (RL) controller implemented as a ROS 2 package.

### Common Interfaces
The `xbot_common_interfaces` package defines:
- Custom messages for robot state and control
- Service definitions for robot operations
- Action definitions for motion control

### Simple Trajectory Server
The simple trajectory server provides basic trajectory execution capabilities:
- Supports different trajectory types (zero position, sinusoidal wave)
- Configurable joint control parameters
- Real-time execution feedback
- ROS 2 action server interface



### RL Controller
This package uses a TorchScript neural network policy to generate precise motor commands for robotic platforms, processing multiple sensor inputs including IMU data, motor feedback, and remote control commands.


## Usage

1. Build the packages:
    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```
2. Launch control sdk
    ```bash
    source install/setup.bash

    ros2 service call /dynamic_launch xbot_common_interfaces/srv/DynamicLaunch \
    "app_name: '' 
    sync_control: false 
    launch_mode: 'pd'" ## replace `pd` with `no_hand_pd` if no hand on robot
    ```
3. Switch robot to ready state
    ```bash
    source your_ws/install/setup.bash
    ros2 service call /ready_service std_srvs/srv/Trigger {}   
    ```
4. Run demo 
    - Simple trajectory
        1. **Launch trajectory server**
        ```bash
        source install/setup.bash
        ros2 launch simple_trajectory_server simple_trajectory_server.launch.py
        ```
        2. **Execute zero trajectory**
            ```bash
            source install/setup.bash
            ros2 action send_goal /simple_trajectory_action xbot_common_interfaces/action/SimpleTrajectory "traj_type: 0 duration: 4.0"  
            ```
        3. **Execute sin wave trajectory**
            ```bash
            source install/setup.bash
            ros2 action send_goal /simple_trajectory_action xbot_common_interfaces/action/SimpleTrajectory "traj_type: 1 duration: 10.0"  
            ```
    - RL controller
        1. Switch robot to active state
            ```bash
            source your_ws/install/setup.bash
            ros2 service call /activate_service std_srvs/srv/Trigger {}   
            ```
        2. Launch the controller:
            ```bash
            source install/setup.bash
            ros2 launch rl_controller rl_controller.launch.py
            ```
        3. Operate radiolink
## Support & Contact

For technical support or business inquiries:
- Email: support@robotera.com
- Create an issue in this repository

## About RobotEra

RobotEra is dedicated to advancing humanoid robotics technology.We specialize in developing robust and versatile humanoid robots for real-world applications. 

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details. 