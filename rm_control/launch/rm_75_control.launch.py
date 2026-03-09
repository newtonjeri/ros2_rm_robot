from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    arm_ns_arg = DeclareLaunchArgument(
        'arm_namespace', default_value='',
        description='Namespace for the arm (e.g., left_arm or right_arm)')

    action_name_arg = DeclareLaunchArgument(
        'action_name', default_value='/left_arm_controller/follow_joint_trajectory',
        description='Action server name for follow joint trajectory (use absolute path e.g. /left_arm_controller/follow_joint_trajectory)')

    pole_action_name_arg = DeclareLaunchArgument(
        'pole_action_name', default_value='',
        description='Action server name for pole/lift controller (e.g. /left_pole_controller/follow_joint_trajectory). Empty disables pole support.')

    hand_action_name_arg = DeclareLaunchArgument(
        'hand_action_name', default_value='',
        description='Action server name for hand controller (e.g. /left_hand_controller/follow_joint_trajectory). Empty disables hand support.')

    hand_feedback_mode_arg = DeclareLaunchArgument(
        'hand_feedback_mode', default_value='open_loop',
        description='Hand feedback mode: open_loop (commanded values) or udp (from udp_hand_status topic).')

    ld = LaunchDescription()
    ld.add_action(arm_ns_arg)
    ld.add_action(action_name_arg)
    ld.add_action(pole_action_name_arg)
    ld.add_action(hand_action_name_arg)
    ld.add_action(hand_feedback_mode_arg)

    control_node = Node(
        package='rm_control',
        executable='rm_control',
        namespace=LaunchConfiguration('arm_namespace'),
        parameters=[
            {'follow': True},
            {'arm_type': 75},
            {'action_name': LaunchConfiguration('action_name')},
            {'pole_action_name': LaunchConfiguration('pole_action_name')},
            {'hand_action_name': LaunchConfiguration('hand_action_name')},
            {'hand_feedback_mode': LaunchConfiguration('hand_feedback_mode')}
        ],
        output='screen',
    )

    ld.add_action(control_node)
    return ld

