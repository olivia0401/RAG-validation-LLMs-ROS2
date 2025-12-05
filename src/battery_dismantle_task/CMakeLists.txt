cmake_minimum_required(VERSION 3.16)
project(battery_dismantle_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)

# 编译 skill_server_node（唯一的C++节点）
add_executable(skill_server_node src/skill_server_node.cpp)

ament_target_dependencies(skill_server_node
  rclcpp
  moveit_ros_planning_interface
  nlohmann_json
  std_msgs
)

# 安装C++节点
install(TARGETS skill_server_node
  DESTINATION lib/${PROJECT_NAME})

# 安装launch文件和config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 安装Python脚本（scripts目录）
install(PROGRAMS
  scripts/publish_scene.py
  scripts/publish_initial_joint_states.py
  scripts/visual_state_manager.py
  DESTINATION lib/${PROJECT_NAME})

# 安装Python包（llm_agent目录）
install(DIRECTORY llm_agent/
  DESTINATION lib/${PROJECT_NAME}/llm_agent
  FILES_MATCHING PATTERN "*.py")

ament_package()
