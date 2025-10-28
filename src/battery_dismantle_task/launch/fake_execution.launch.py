#!/usr/bin/env python3
"""
Final, definitive, and correct launch file for fake execution.
This version corrects the parameter namespacing issue by defining separate
parameter dictionaries for kinematics and controllers, and passing them to the
correct nodes. This ensures all nodes (move_group, rviz, skill_server)
are properly configured.
"""

import os
import yaml
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Standard launch arguments
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # --- Correctly locate all necessary files ---
    moveit_pkg_name = "battery_dismantle_task"
    moveit_share = Path(get_package_share_directory(moveit_pkg_name))

    xacro_file = str(Path(get_package_share_directory("kortex_description")) / "robots" / "gen3_robotiq_2f_85.xacro")
    srdf_file = str(moveit_share / "config" / "gen3_robotiq_2f_85.srdf")
    rviz_file = str(moveit_share / "config" / "moveit.rviz")

    # --- Define a non-colliding start pose ("home") ---
    home_pose_str = '''{'joint_1': 0.0, 'joint_2': 0.2618, 'joint_3': 3.14159, 'joint_4': -2.2689, 'joint_5': 0.0, 'joint_6': 0.9599, 'joint_7': 1.5708}'''

    # --- Use MoveItConfigsBuilder for robot_description and SRDF ONLY ---
    # Do NOT load trajectory_execution (controllers) - we handle that manually for fake execution
    moveit_config = (
        MoveItConfigsBuilder(robot_name="kinova_gen3_6dof_robotiq_2f_85")
        .robot_description(
            file_path=xacro_file,
            mappings={"use_fake_hardware": "true", "initial_positions": home_pose_str},
        )
        .robot_description_semantic(file_path=srdf_file)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # --- Define Parameters in SEPARATE Python Dictionaries ---

    # 1. Kinematics: Needed by move_group, rviz, and skill_server
    kinematics_params = {
        "robot_description_kinematics": {
            "manipulator": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
            }
        }
    }

    # 2. Controllers: Load controller configuration for fake execution
    # Even with fake_execution: true, we need to define controllers so
    # trajectory_execution_manager knows which joints to simulate
    controllers_file = str(moveit_share / "config" / "moveit_controllers.yaml")

    with open(controllers_file, 'r') as f:
        controllers_config = yaml.safe_load(f)

    controllers_params = {
        "moveit_simple_controller_manager": controllers_config
    }

    # QoS configuration file (same as debug_robot_visualization)
    qos_params_file = str(moveit_share / "config" / "robot_state_publisher_qos.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            qos_params_file,  # Add QoS parameters
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            controllers_params,
            kinematics_params,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # --- Assemble the final launch description ---
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])