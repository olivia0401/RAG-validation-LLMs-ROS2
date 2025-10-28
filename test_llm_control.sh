#!/bin/bash
# LLM控制机器人测试脚本 - 完整流程

echo "=========================================="
echo "🤖 LLM控制机器人 - 完整测试"
echo "=========================================="
echo ""
echo "此脚本将测试LLM通过ROS2控制机器人手臂在RViz中运动"
echo ""
echo "测试流程："
echo "  1. 启动机器人系统（fake execution模式）"
echo "  2. 启动skill_server接收LLM命令"
echo "  3. 在RViz中观察机器人运动"
echo ""
echo "修复内容："
echo "  ✅ 移除了joint_state_publisher_gui（解决冲突）"
echo "  ✅ 使用MoveIt fake execution模拟关节状态"
echo "  ✅ skill_server通过/llm_commands接收命令"
echo ""
echo "=========================================="
echo ""

# 检查是否在正确的目录
if [ ! -d "/home/olivia/llms-ros2" ]; then
    echo "❌ 错误: /home/olivia/llms-ros2 目录不存在"
    exit 1
fi

cd /home/olivia/llms-ros2

echo "步骤 1: Source工作空间..."
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "✅ 工作空间已source"
echo ""

echo "步骤 2: 启动机器人系统（fake execution模式）..."
echo ""
echo "启动后您将看到："
echo "  - RViz窗口（可视化机器人）"
echo "  - move_group节点（运动规划）"
echo "  - skill_server节点（等待LLM命令）"
echo ""
echo "⚠️  请等待所有节点启动完成（约5-10秒）"
echo "⚠️  当看到 '[skill_server]: ✅ Skill Server Ready!' 时表示系统就绪"
echo ""
echo "=========================================="
echo "测试命令（在另一个终端运行）："
echo "=========================================="
echo ""
echo "# 方式1: 使用ROS2命令测试单个技能"
echo "cd /home/olivia/llms-ros2"
echo "source install/setup.bash"
echo ""
echo "# 测试moveTo技能"
echo "ros2 topic pub --once /llm_commands std_msgs/msg/String \\"
echo "  '{\"data\": \"{\\\"skill\\\": \\\"moveTo\\\", \\\"target\\\": \\\"HOME\\\"}\"}'"
echo ""
echo "# 查看反馈"
echo "ros2 topic echo /llm_feedback"
echo ""
echo "----------------------------------------"
echo ""
echo "# 方式2: 使用LLM Agent（推荐）"
echo "cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent"
echo "python3 main.py \"拆解电池顶盖螺栓\" --demo"
echo ""
echo "=========================================="
echo ""
echo "按 Enter 启动系统..."
read

echo "🚀 启动中..."
echo ""

# 启动fake_execution.launch.py
ros2 launch battery_dismantle_task fake_execution.launch.py
