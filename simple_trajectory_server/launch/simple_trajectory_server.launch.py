import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="simple_trajectory_server",
            executable="simple_trajectory_server",
            output="both",
            parameters=[PathJoinSubstitution(
                [FindPackageShare("simple_trajectory_server"), "config/config.yaml",]
            )],
            emulate_tty=True
        )
    ])
