# RL Controller

A real-time reinforcement learning (RL) controller implemented as a ROS 2 package. This system uses a TorchScript neural network policy to generate precise motor commands for robotic platforms, processing multiple sensor inputs including IMU data, motor feedback, and remote control commands.

## Features

- High-frequency real-time inference (100 Hz) with dedicated threading
- Temporal context through 15-frame observation stacking
- Smooth transition handling between control states
- Comprehensive safety systems including remote kill switch
- Full ROS 2 middleware integration
- Real-time performance optimizations
- Configurable policy loading via YAML

## System Requirements

- ROS 2 (Foxy/Humble or later)
- libtorch (PyTorch C++ library)
- xbot_common_interfaces
- C++17 or later

## Installation

1. Download and unzip the repository (https://github.com/roboterax/star1_example):

2. Build the package:
   ```bash
   cd star1_example
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```


## Architecture

### Control Flow

The controller operates in four distinct phases:

1. **Initialization** (100 steps; 1s)
   - Collection of initial motor positions
   - System calibration

2. **Startup** (500 steps; 5s)
   - Smooth interpolation from initial to default positions
   - Safety checks and sensor validation

3. **Stand-still** (900 steps; 9s)
   - Maintenance of default poses
   - System stability verification

4. **RL Control**
   - Full policy deployment
   - Real-time inference and control

### Topic Interface

#### Subscribed Topics
- `/imu_feedback` (xbot_common_interfaces/msg/Imu)
- `/loco/remoteControl/radio` (xbot_common_interfaces/msg/ChannelsMsg)
- `/motor_feedback` (std_msgs/msg/Float64MultiArray)

#### Published Topics
- `/rl_controller/policy_inference` (std_msgs/msg/Float64MultiArray)
  - Array format: [positions[31], velocities[31], kp[31], kd[31], torques[31]]

### Remote Control Mapping

| Channel | Function         | Info                                                      |
|---------|------------------|-----------------------------------------------------------|
| 0       | Turning rate     | Uses the right stick for adjusting angular motion         |
| 2       | Forward/Backward | Controls forward & reverse speed using the left stick     |
| 3       | Lateral movement | Enables side-to-side motion via the left stick            |
| 8       | Movement enable  | Engages/disengages motion; moving down on RC locks controls |
| 9       | Safety switch    | Activates an emergency stop when moved down               |

## Configuration

### Policy Requirements

Your custom policy model must meet these specifications if you use the default observation and action space:

- Input tensor shape: `[1, 705]` (47 observations × 15 history frames)
- Output tensor shape: `[1, 12]` (control actions for leg joints)
- Format: TorchScript

### Customization

To modify the system architecture:

1. Observation size adjustment:
   - Modify `N_SINGLE_POLICY_OBS` in `rl_controller.hpp`

2. History length changes:
   - Update `N_HIST_LEN` in `rl_controller.hpp`

3. Joint control expansion:
   - Adjust `N_CONTROLLED_JOINTS` and `N_UNCONTROLLED_JOINTS` in `rl_controller.hpp`
   - Example for including waist and arm control:
     ```cpp
     constexpr int N_CONTROLLED_JOINTS = N_LEG_JOINTS + N_WAIST_JOINTS + N_ARM_JOINTS;
     constexpr int N_UNCONTROLLED_JOINTS = N_NECK_JOINTS;
     ```
   - Update training code to match new observation size and output size

### Loading Custom Policies

1. Place your TorchScript model in the `model` directory
2. Update the configuration:
   ```yaml
   # config/policy.yaml
   rl_controller_node:
     ros__parameters:
       policy_file: "your_policy.pt"
   ```

## Usage

Launch the controller:
```bash
# cd /rl_controller
source install/setup.bash
ros2 launch rl_controller rl_controller.launch.py
```

## Common Issues

1. Policy Loading Failures
   - Verify policy file path in `config/policy.yaml`
   - Ensure TorchScript model matches expected input/output dimensions
   - Check model compatibility with current libtorch version

2. Performance Issues
   - Adjust thread count via `at::set_num_threads()` based on system capabilities
   - Monitor sensor data rates using `ros2 topic hz`
   - Check CPU usage with `top` or `htop`
   - Consider disabling debug logging in production

3. Control Instability
   - Validate sensor data quality and update rates
   - Check PD gain settings in configuration
   - Ensure your policy model is well-tuned for the task


## Contact

For support or questions, please contact support@robotera.com