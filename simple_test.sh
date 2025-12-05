#!/bin/bash

echo "=== 测试1: 移动到HOME位置 ==="
ros2 topic pub --once /llm_commands std_msgs/msg/String 'data: "{\"schema\":\"llm_cmd/v1\",\"command_id\":\"test1\",\"skill\":\"moveTo\",\"target\":\"HOME\"}"'
sleep 3

echo ""
echo "=== 测试2: 打开夹爪 ==="
ros2 topic pub --once /llm_commands std_msgs/msg/String 'data: "{\"schema\":\"llm_cmd/v1\",\"command_id\":\"test2\",\"skill\":\"openGripper\"}"'
sleep 3

echo ""
echo "=== 测试3: 关闭夹爪 ==="
ros2 topic pub --once /llm_commands std_msgs/msg/String 'data: "{\"schema\":\"llm_cmd/v1\",\"command_id\":\"test3\",\"skill\":\"closeGripper\"}"'
sleep 3

echo ""
echo "=== 测试完成 ==="
