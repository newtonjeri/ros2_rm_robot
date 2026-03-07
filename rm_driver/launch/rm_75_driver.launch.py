import launch
import os
import yaml
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Declare launch arguments for namespace and config file
    arm_ns_arg = DeclareLaunchArgument(
        'arm_namespace', default_value='',
        description='Namespace for the arm (e.g., left_arm or right_arm)')

    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value='rm_75_config.yaml',
        description='Config YAML file name (e.g., rm_75_left_config.yaml)')

    arm_config = PathJoinSubstitution([
        FindPackageShare('rm_driver'), 'config', LaunchConfiguration('config_file')
    ])

    return LaunchDescription([
        arm_ns_arg,
        config_file_arg,

        Node(
            package="rm_driver",
            executable="rm_driver",
            namespace=LaunchConfiguration('arm_namespace'),
            parameters=[arm_config],
            output='screen'
        )
    ])