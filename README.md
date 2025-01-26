# STAR1 Example Package

This repository contains example ROS 2 packages for STAR1 robot control, developed by RobotEra.

## Overview

This repository includes the following packages:

- `star1_example`: A metapackage containing example packages for STAR1 robot
- `simple_trajectory_server`: A trajectory server implementation for basic motion control
- `xbot_common_interfaces`: Common interface definitions used by STAR1 robot

### Simple Trajectory Server

The simple trajectory server provides basic trajectory execution capabilities:
- Supports different trajectory types (zero position, sinusoidal wave)
- Configurable joint control parameters
- Real-time execution feedback
- ROS 2 action server interface

### Common Interfaces

The `xbot_common_interfaces` package defines:
- Custom messages for robot state and control
- Service definitions for robot operations
- Action definitions for motion control

## Usage

1. Build the packages:

```bash
colcon build --
```

## Support & Contact

For technical support or business inquiries:
- Email: support@robotera.com
- Create an issue in this repository

## About RobotEra

RobotEra is dedicated to advancing humanoid robotics technology.We specialize in developing robust and versatile humanoid robots for real-world applications. 

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details. 