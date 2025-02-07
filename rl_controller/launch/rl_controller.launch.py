# loco_controller_launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory("rl_controller")
    policy_config = os.path.join(package_dir, "config", "policy.yaml")
    return LaunchDescription([
        Node(
            package='rl_controller',  # 替换为你的包名
            executable='rl_controller_node',  # 替换为你编译后的可执行文件名
            output='screen',
            parameters=[policy_config],
            # prefix='xterm -e gdb --ex run --args'
        ),
    ])
