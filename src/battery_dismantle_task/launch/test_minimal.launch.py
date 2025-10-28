#!/usr/bin/env python3
"""
Minimal test launch file to isolate the empty tuple issue
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Standard launch argument
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")

    # Get paths
    moveit_pkg_name = "battery_dismantle_task"
    moveit_share = Path(get_package_share_directory(moveit_pkg_name))

    xacro_file = str(Path(get_package_share_directory("kortex_description")) / "robots" / "gen3_robotiq_2f_85.xacro")
    srdf_file = str(moveit_share / "config" / "gen3_robotiq_2f_85.srdf")

    # Define home pose
    home_pose_str = '''{'joint_1': 0.0, 'joint_2': 0.2618, 'joint_3': 3.14159, 'joint_4': -2.2689, 'joint_5': 0.0, 'joint_6': 0.9599, 'joint_7': 1.5708}'''

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder(robot_name="gen3_robotiq_2f_85", package_name=moveit_pkg_name)
        .robot_description(
            file_path=xacro_file,
            mappings={"use_fake_hardware": "true", "initial_positions": home_pose_str},
        )
        .robot_description_semantic(file_path=srdf_file)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # QoS configuration file
    qos_params_file = str(moveit_share / "config" / "robot_state_publisher_qos.yaml")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            qos_params_file,
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"source_list": []},
        ],
    )

    return LaunchDescription([
        use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
