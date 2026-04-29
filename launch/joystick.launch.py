# joystick.launch.py
# fr3_teleop: One-command bring-up for PS5 joystick teleop. Includes
# teleop.launch.py (servo_node + robot_state_publisher), starts joy_node,
# and runs joystick_teleop. move_group is started conditionally for the
# home-pose button.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    teleop_share = get_package_share_directory('fr3_teleop')

    with_move_group_arg = DeclareLaunchArgument(
        'with_move_group',
        default_value='true',
        description='Launch move_group for the home-pose button (Circle).',
    )
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev_id',
        default_value='0',
        description='Joystick device id (e.g. 0 → /dev/input/js0).',
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_share, 'launch', 'teleop.launch.py')
        ),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_share, 'launch', 'move_group.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('with_move_group')),
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_dev_id'),
            'autorepeat_rate': 20.0,
        }],
        output='screen',
    )

    joystick_teleop = Node(
        package='fr3_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
    )

    return LaunchDescription([
        with_move_group_arg,
        joy_dev_arg,
        teleop_launch,
        move_group_launch,
        joy_node,
        joystick_teleop,
    ])
