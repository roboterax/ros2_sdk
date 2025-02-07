# Simple Trajectory Server

This package provides a simple trajectory server for executing predefined trajectories within a ROS2 system. It is designed to offer a straightforward interface for trajectory control and feedback.

## Features

- Executes simple trajectories for robot joints or actuators
- Provides real-time feedback during execution
- Configurable trajectory parameters and durations

## ROS2 Interface
#### Subscribed Topics
- `/joint_states` (sensor_msgs/msg/JointState)
- `/imu_broadcaster/imu` (sensor_msgs/msg/Imu)

#### Published Topic
- `/hybrid_body_controller/commands` (xbot_common_interfaces/msg/HybridJointCommand)

#### Action
- `/simple_trajectory_action` (xbot_common_interfaces/action/SimpleTrajectory)

## Usage

- First of all ,insure that robot is in `ready` state

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