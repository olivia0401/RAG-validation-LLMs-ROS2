#!/bin/bash
# 运行LLM Executor的快捷脚本

cd /home/olivia/llms-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 检查是否提供了JSON文件参数
if [ $# -eq 0 ]; then
    echo "使用默认的示例JSON文件..."
    JSON_FILE="src/battery_dismantle_task/config/llm_output_example.json"
else
    JSON_FILE="$1"
fi

echo "运行 executor 使用文件: $JSON_FILE"
python3 src/LLMs/executor_patch.py "$JSON_FILE"
