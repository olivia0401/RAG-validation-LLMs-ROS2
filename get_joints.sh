#!/bin/bash
# 快速读取当前机械臂关节角度

echo "==================================="
echo "Current Joint States"
echo "==================================="

ros2 topic echo /joint_states --once | grep -A 50 "name:" | head -60

echo ""
echo "==================================="
echo "提取关节角度数组（复制到waypoints.json）："
echo "==================================="

# 提取并格式化关节角度
ros2 topic echo /joint_states --once 2>/dev/null | python3 -c "
import sys
import yaml

data = sys.stdin.read()
try:
    msg = yaml.safe_load(data)

    # 找到joint_1到joint_7的位置
    joints = {}
    for i, name in enumerate(msg['name']):
        if 'joint_' in name and name.startswith('joint_'):
            joints[name] = round(msg['position'][i], 4)

    # 按顺序输出
    joint_array = [joints.get(f'joint_{i}', 0.0) for i in range(1, 8)]

    print('Joint angles for waypoints.json:')
    print(joint_array)
    print()
    print('Formatted:')
    print('[')
    for i, val in enumerate(joint_array):
        print(f'    {val},  # joint_{i+1}')
    print(']')

except Exception as e:
    print(f'Error: {e}')
"
