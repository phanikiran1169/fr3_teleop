# move_group.launch.py
# fr3_teleop: Inference-PC move_group for the FR3, used by joystick_teleop
# to plan and execute the home-pose move.
#
# Adapted from franka_ros2's franka_fr3_moveit_config/launch/moveit.launch.py.
# Only move_group itself runs on the inference PC; ros2_control_node,
# controller spawners, robot_state_publisher, joint_state_publisher, the
# gripper, and rviz run on the RT PC via franka_bringup.
#
# URDF is loaded with ros2_control:=false so the inference PC's URDF does
# not declare command/state interfaces (the RT PC owns the hardware
# interface).

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_name, file_path):
    path = os.path.join(get_package_share_directory(package_name), file_path)
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    franka_xacro = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro',
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command([
                FindExecutable(name='xacro'), ' ', franka_xacro,
                ' hand:=true', ' arm_id:=fr3', ' ros2_control:=false',
            ]),
            value_type=str,
        )
    }

    franka_srdf_xacro = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'srdf', 'fr3_arm.srdf.xacro',
    )
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            Command([FindExecutable(name='xacro'), ' ',
                     franka_srdf_xacro, ' hand:=true']),
            value_type=str,
        )
    }

    kinematics_yaml = _load_yaml(
        'franka_fr3_moveit_config', 'config/kinematics.yaml')

    ompl_planning = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning['move_group'].update(
        _load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml'))

    moveit_simple_controllers = _load_yaml(
        'franka_fr3_moveit_config', 'config/fr3_controllers.yaml')
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor,
        ],
    )

    return LaunchDescription([move_group_node])
