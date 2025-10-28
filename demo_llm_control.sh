#!/bin/bash
# LLM 控制机器人拆卸电池 - 完整演示

echo "=========================================="
echo "🤖 LLM Robot Control Demo"
echo "=========================================="
echo ""
echo "此演示展示 LLM 如何规划并执行电池拆卸任务"
echo ""
echo "架构："
echo "  LLM → Planner → Validator → Executor → Skill Server → Robot"
echo ""
echo "按 Enter 开始演示..."
read

cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent

echo ""
echo "正在启动 LLM Agent..."
echo "任务: 拆解电池顶盖螺栓"
echo ""

python3 main.py "拆解电池顶盖螺栓" --demo --mock

echo ""
echo "=========================================="
echo "✅ 演示完成！"
echo "=========================================="
echo ""
echo "说明："
echo "- 使用 --demo 参数：使用预定义的任务计划"
echo "- 使用 --mock 参数：模拟机器人执行"
echo ""
echo "真实机器人控制："
echo "  python3 main.py '拆解电池' (需要 ROS2 + MoveIt 正确配置)"
echo ""
