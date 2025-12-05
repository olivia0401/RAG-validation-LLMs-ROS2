#!/bin/bash
# 测试RViz中的机械臂运动

cd /home/olivia/llms-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "======================================"
echo "🎯 测试RViz机械臂运动"
echo "======================================"
echo ""
echo "步骤："
echo "1. 确保RViz已经打开（应该能看到机器人模型）"
echo "2. 在RViz中查看'RobotModel'显示"
echo "3. 运行LLM任务时，观察机械臂是否移动"
echo ""
echo "======================================"
echo ""

# 监控joint_states (后台)
echo "📊 开始监控joint_states (5秒)..."
timeout 5 ros2 topic echo /joint_states --field position | while read line; do
    echo "  Joint positions: $line"
done &

# 等待1秒让监控启动
sleep 1

# 执行任务
echo ""
echo "🚀 执行grasp/release任务..."
cd /home/olivia/llms-ros2/src/battery_dismantle_task/llm_agent
python3 main.py --demo

echo ""
echo "======================================"
echo "✅ 测试完成！"
echo ""
echo "检查："
echo "- 上面是否显示了不同的joint positions?"
echo "- RViz中机械臂是否移动了?"
echo ""
echo "如果RViz中没有看到运动，请检查："
echo "  1. RViz的 'RobotModel' display是否启用"
echo "  2. Fixed Frame 是否设置为 'base_link' 或 'world'"
echo "======================================"
