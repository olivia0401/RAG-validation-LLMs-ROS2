#!/bin/bash
# RViz显示诊断脚本

echo "=========================================="
echo "RViz显示完整诊断"
echo "=========================================="

source /home/olivia/llms-ros2/install/setup.bash

echo ""
echo "1. 检查运行的ROS2节点"
echo "=========================================="
ros2 node list | grep -E "(robot_state|rviz)"
NODE_COUNT=$(ros2 node list | grep robot_state_publisher | wc -l)
echo ""
echo "robot_state_publisher 数量: $NODE_COUNT"
if [ $NODE_COUNT -gt 1 ]; then
    echo "⚠️  警告：发现多个robot_state_publisher！"
else
    echo "✅ robot_state_publisher 数量正常"
fi

echo ""
echo "2. 检查TF frames"
echo "=========================================="
ros2 run tf2_ros tf2_echo world base_link 2>&1 | head -15 &
sleep 2
pkill -9 tf2_echo

echo ""
echo "3. 检查joint_states话题"
echo "=========================================="
echo "关节名称："
ros2 topic echo /joint_states --once 2>&1 | grep -A 20 "name:"

echo ""
echo "4. 检查RViz配置文件"
echo "=========================================="
echo "Scene Robot 设置："
grep -A 6 "Scene Robot:" /home/olivia/llms-ros2/src/battery_dismantle_task/config/moveit.rviz

echo ""
echo "Planned Path 设置："
grep -A 10 "Planned Path:" /home/olivia/llms-ros2/src/battery_dismantle_task/config/moveit.rviz

echo ""
echo "5. 检查Display数量"
echo "=========================================="
DISPLAY_COUNT=$(grep -c "Class: moveit_rviz_plugin/MotionPlanning" /home/olivia/llms-ros2/src/battery_dismantle_task/config/moveit.rviz)
echo "MotionPlanning Display 数量: $DISPLAY_COUNT"
if [ $DISPLAY_COUNT -gt 1 ]; then
    echo "⚠️  警告：发现多个MotionPlanning Display！"
else
    echo "✅ MotionPlanning Display 数量正常"
fi

echo ""
echo "=========================================="
echo "诊断完成"
echo "=========================================="
