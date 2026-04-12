# fr3_teleop/launch/teleop.launch.py
# Launches MoveIt Servo for FR3 keyboard teleoperation on the inference laptop.
# Expects franka_ros2 + fr3_arm_controller running on the RT PC.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # FR3 robot description (URDF)
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file,
         ' hand:=true',
         ' robot_ip:=192.168.51.20',
         ' use_fake_hardware:=true',
         ' fake_sensor_commands:=false']
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_config, value_type=str)
    }

    # FR3 semantic description (SRDF)
    franka_semantic_xacro_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf', 'fr3_arm.srdf.xacro'
    )
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=true']
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_config, value_type=str)
    }

    # Kinematics
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    # Servo parameters
    servo_yaml = load_yaml('fr3_teleop', 'config/servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml['moveit_servo']}

    # Servo node (standalone, not composable — runs on inference laptop,
    # communicates with fr3_arm_controller on RT PC via ROS2)
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        output='screen',
    )

    # Robot state publisher (for TF tree on inference laptop)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        servo_node,
    ])
