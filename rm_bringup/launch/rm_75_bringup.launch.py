import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():

    # ==================== Launch Arguments ====================
    hand_feedback_mode_arg = DeclareLaunchArgument(
        'hand_feedback_mode', default_value='open_loop',
        description='Hand feedback mode: open_loop (commanded values) or udp (from udp_hand_status).')

    # ==================== Left Arm ====================
    left_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_driver'), 'launch', 'rm_75_driver.launch.py')),
        launch_arguments={
            'arm_namespace': 'left_arm',
            'config_file': 'rm_75_left_config.yaml',
        }.items()
    )

    left_arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_control'), 'launch', 'rm_75_control.launch.py')),
        launch_arguments={
            'arm_namespace': 'left_arm',
            'action_name': '/left_arm_controller/follow_joint_trajectory',
            'pole_action_name': '/left_pole_controller/follow_joint_trajectory',
            'hand_action_name': '/left_hand_controller/follow_joint_trajectory',
            'hand_feedback_mode': LaunchConfiguration('hand_feedback_mode'),
        }.items()
    )

    # ==================== Right Arm ====================
    right_arm_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_driver'), 'launch', 'rm_75_driver.launch.py')),
        launch_arguments={
            'arm_namespace': 'right_arm',
            'config_file': 'rm_75_right_config.yaml',
        }.items()
    )

    right_arm_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rm_control'), 'launch', 'rm_75_control.launch.py')),
        launch_arguments={
            'arm_namespace': 'right_arm',
            'action_name': '/right_arm_controller/follow_joint_trajectory',
            'pole_action_name': '/right_pole_controller/follow_joint_trajectory',
            'hand_action_name': '/right_hand_controller/follow_joint_trajectory',
            'hand_feedback_mode': LaunchConfiguration('hand_feedback_mode'),
        }.items()
    )

    # ==================== Description (shared) ====================
    # rm_75_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('rm_description'), 'launch', 'rm_75_display.launch.py')),
    # )

    # ==================== MoveIt (shared) ====================
    # rm_75_moveit_config = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('rm_75_config'), 'launch', 'real_moveit_demo.launch.py')),
    # )

    return LaunchDescription([
        hand_feedback_mode_arg,
        left_arm_driver,
        left_arm_control,
        right_arm_driver,
        right_arm_control,
        # Joint state merger: merges both arm topics into /joint_states
        Node(
            package='rm_driver',
            executable='joint_state_merger.py',
            name='joint_state_merger',
            parameters=[{
                'right_arm_topic': '/right_arm/arm_joint_states',
                'left_arm_topic': '/left_arm/arm_joint_states',
                'merged_topic': '/joint_states',
            }],
            output='screen',
        ),
        # rm_75_description,
        # rm_75_moveit_config,
    ])