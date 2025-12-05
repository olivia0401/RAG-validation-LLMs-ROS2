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
import launch
import launch.conditions
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

    # Use kortex's official SRDF (has complete gripper configuration)
    kortex_moveit_pkg = "kinova_gen3_7dof_robotiq_2f_85_moveit_config"
    kortex_moveit_share = Path(get_package_share_directory(kortex_moveit_pkg))

    xacro_file = str(Path(get_package_share_directory("kortex_description")) / "robots" / "gen3_robotiq_2f_85.xacro")
    srdf_file = str(kortex_moveit_share / "config" / "gen3_robotiq_2f_85.srdf")  # Use kortex SRDF
    rviz_file = str(moveit_share / "config" / "moveit.rviz")

    # --- Define a non-colliding start pose ("home") ---
    home_pose_str = '''{'joint_1': 0.0, 'joint_2': 0.2618, 'joint_3': 3.14159, 'joint_4': -2.2689, 'joint_5': 0.0, 'joint_6': 0.9599, 'joint_7': 1.5708}'''

    # --- Use MoveItConfigsBuilder for robot_description and SRDF ONLY ---
    # Do NOT load trajectory_execution (controllers) - we handle that manually for fake execution
    # Do NOT load planning_pipelines - we manually override with modified ompl_planning.yaml
    moveit_config = (
        MoveItConfigsBuilder(robot_name="kinova_gen3_6dof_robotiq_2f_85")
        .robot_description(
            file_path=xacro_file,
            mappings={"use_fake_hardware": "true", "initial_positions": home_pose_str},
        )
        .robot_description_semantic(file_path=srdf_file)
        .to_moveit_configs()
    )

    # Manually load the MODIFIED ompl_planning.yaml (FixStartStateCollision removed)
    ompl_planning_file = str(kortex_moveit_share / "config" / "ompl_planning.yaml")
    with open(ompl_planning_file, 'r') as f:
        ompl_planning_config = yaml.safe_load(f)

    planning_pipelines_params = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_planning_config,
    }

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
            planning_pipelines_params,  # Use our manually loaded ompl_planning.yaml
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
            planning_pipelines_params,  # Use manually loaded planning pipelines
        ],
    )

    # --- Launch argument for skill_server ---
    start_skill_server_arg = DeclareLaunchArgument(
        "start_skill_server",
        default_value="true",
        description="Whether to start the skill server node"
    )
    start_skill_server = LaunchConfiguration("start_skill_server")

    # --- Skill Server Node (with delay to ensure move_group is ready) ---
    skill_server_node = TimerAction(
        period=3.0,  # Wait 3 seconds for move_group to initialize
        actions=[
            Node(
                package="battery_dismantle_task",
                executable="skill_server_node",
                name="skill_server",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    planning_pipelines_params,  # Use manually loaded planning pipelines
                    kinematics_params,
                    {
                        "waypoints_path": str(moveit_share / "config" / "waypoints.json"),
                        "use_sim_time": False,
                    },
                ],
                condition=launch.conditions.IfCondition(start_skill_server),
            )
        ],
    )

    # --- Publish Initial Joint States (required for fake execution) ---
    publish_initial_joint_states_node = Node(
        package="battery_dismantle_task",
        executable="publish_initial_joint_states.py",
        name="publish_initial_joint_states",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # --- Visual State Manager (manages collision objects and attach/detach) ---
    visual_state_manager_node = TimerAction(
        period=3.0,  # Wait for move_group
        actions=[
            Node(
                package="battery_dismantle_task",
                executable="visual_state_manager.py",
                name="visual_state_manager",
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ],
    )

    # --- Assemble the final launch description ---
    return LaunchDescription([
        use_sim_time,
        start_skill_server_arg,
        robot_state_publisher_node,
        publish_initial_joint_states_node,
        move_group_node,
        rviz_node,
        skill_server_node,
        visual_state_manager_node,
    ])