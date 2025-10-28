
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Get the package share directory
    moveit_pkg_name = "battery_dismantle_task"
    moveit_share = Path(get_package_share_directory(moveit_pkg_name))

    # Get the xacro file path
    xacro_file = str(Path(get_package_share_directory("kortex_description")) / "robots" / "gen3_robotiq_2f_85.xacro")

    # Use MoveItConfigsBuilder to get the robot description
    moveit_config = (
        MoveItConfigsBuilder(robot_name="gen3_robotiq_2f_85", package_name=moveit_pkg_name)
        .robot_description(file_path=xacro_file)
        .to_moveit_configs()
    )

    # RViz configuration file
    rviz_file = str(moveit_share / "config" / "moveit.rviz")

    # QoS parameter file
    qos_params_file = str(moveit_share / "config" / "robot_state_publisher_qos.yaml")

    # Create the robot_state_publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            qos_params_file,
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        arguments=["--ros-args", "-p", "qos_overrides:='{/joint_states: {reliability: best_effort}}'"],
    )

    # Create the rviz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Assemble the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
