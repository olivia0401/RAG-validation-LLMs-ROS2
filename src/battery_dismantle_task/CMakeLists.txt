cmake_minimum_required(VERSION 3.16)
project(battery_dismantle_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dismantle_node src/dismantle_node.cpp src/DismantleTask.cpp)

target_include_directories(dismantle_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(dismantle_node
  rclcpp
  moveit_ros_planning_interface
  moveit_task_constructor_core
  ament_index_cpp
  nlohmann_json
  tf2_geometry_msgs
  std_msgs
)

# New simple skill server
add_executable(skill_server src/skill_server_node.cpp)

target_include_directories(skill_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(skill_server
  rclcpp
  moveit_ros_planning_interface
  nlohmann_json
  std_msgs
)

install(TARGETS dismantle_node skill_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}
  PATTERN "*.yaml"
  PATTERN "*.json"
  PATTERN "*.rviz"
  PATTERN "*.srdf")

install(PROGRAMS
  scripts/publish_scene.py
  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(battery_dismantle_task)

ament_package()