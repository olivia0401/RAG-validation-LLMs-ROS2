#!/bin/bash

echo "========================================="
echo "完整系统测试脚本"
echo "========================================="
echo ""

cd /home/olivia/llms-ros2

# 1. 启动fake execution系统
echo "步骤 1: 启动fake execution系统（后台运行）..."
source install/setup.bash
ros2 launch battery_dismantle_task fake_execution.launch.py > /tmp/system_test.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

# 2. 等待系统完全启动
echo "步骤 2: 等待系统启动（10秒）..."
sleep 10

# 3. 检查所有节点是否运行
echo "步骤 3: 检查ROS2节点状态..."
source install/setup.bash
ros2 node list

# 4. 运行LLM agent测试
echo ""
echo "步骤 4: 运行LLM Agent测试..."
cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent
timeout 30 python3 main.py 2>&1 | tee /tmp/llm_test.log

# 5. 显示结果
echo ""
echo "========================================="
echo "测试完成！"
echo "========================================="
echo ""
echo "查看完整日志:"
echo "  - Launch log: /tmp/system_test.log"
echo "  - LLM log: /tmp/llm_test.log"
echo ""

# 清理
echo "清理进程..."
kill $LAUNCH_PID 2>/dev/null
pkill -f "ros2 launch battery_dismantle_task" 2>/dev/null

echo "完成！"
