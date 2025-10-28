#!/usr/bin/env python3
"""
纯 RViz + MoveIt 演示 - 不需要 ros2_control
只用于可视化和 LLM 控制测试
"""

from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 配置路径
    kortex_share = Path(get_package_share_directory("kortex_description"))
    xacro_file_path = str(kortex_share / "robots" / "gen3_robotiq_2f_85.xacro")

    moveit_pkg_name = "battery_dismantle_task"
    moveit_share = Path(get_package_share_directory(moveit_pkg_name))
    srdf_file_path = str(moveit_share / "config" / "gen3_robotiq_2f_85.srdf")
    rviz_config_path = str(moveit_share / "config" / "moveit.rviz")

    # MoveIt 配置 - 使用 fake_execution (不需要 ros2_control)
    moveit_config = (
        MoveItConfigsBuilder(robot_name="gen3_robotiq_2f_85", package_name=moveit_pkg_name)
        .robot_description(
            file_path=xacro_file_path,
            mappings={
                "dof": "7",
                "gripper": "robotiq_2f_85",
                "use_fake_hardware": "true",  # 这个参数会被忽略
                "sim_ignition": "false",
                "use_isaac_bridge": "false",
            },
        )
        .robot_description_semantic(file_path=srdf_file_path)
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        # 关键：使用 MoveIt 的 fake controllers
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description],
    )

    # move_group - 使用 fake_execution
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {
                "capabilities": "move_group/ExecuteTaskSolutionCapability",
                # 关键：启用 fake execution
                "trajectory_execution.execution_duration_monitoring": False,
                "allow_trajectory_execution": True,
            },
        ],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": False},
        ],
    )

    # 场景发布节点
    publish_scene_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="battery_dismantle_task",
                executable="publish_scene.py",
                output="screen",
            )
        ],
    )

    # 技能服务器节点
    skill_server_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="battery_dismantle_task",
                executable="skill_server",
                output="screen",
                parameters=[
                    {"waypoints_path": str(moveit_share / "config" / "waypoints.json")},
                ]
            )
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
        move_group_node,
        rviz_node,
        publish_scene_node,
        skill_server_node,
    ])
