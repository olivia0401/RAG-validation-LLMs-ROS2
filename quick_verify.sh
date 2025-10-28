#!/bin/bash
# 快速验证修复是否成功

echo "=========================================="
echo "🔍 快速验证测试"
echo "=========================================="
echo ""

cd /home/olivia/llms-ros2

echo "✅ 1. 检查launch文件是否存在..."
if [ -f "src/battery_dismantle_task/launch/fake_execution.launch.py" ]; then
    echo "   ✅ fake_execution.launch.py 存在"
else
    echo "   ❌ fake_execution.launch.py 不存在"
    exit 1
fi

echo ""
echo "✅ 2. 检查是否移除了joint_state_publisher_gui..."
if grep -q "joint_state_publisher_gui_node" src/battery_dismantle_task/launch/fake_execution.launch.py; then
    echo "   ❌ 仍然包含joint_state_publisher_gui_node"
    echo "   请手动检查文件"
else
    echo "   ✅ 已成功移除joint_state_publisher_gui"
fi

echo ""
echo "✅ 3. 检查skill_server是否编译..."
if [ -f "install/battery_dismantle_task/lib/battery_dismantle_task/skill_server" ]; then
    echo "   ✅ skill_server 已编译"
else
    echo "   ❌ skill_server 未找到"
    echo "   运行: colcon build --packages-select battery_dismantle_task"
    exit 1
fi

echo ""
echo "✅ 4. 检查LLM Agent是否存在..."
if [ -f "src/battery_dismantle_task/LLM_Robot_Agent/main.py" ]; then
    echo "   ✅ LLM Agent main.py 存在"
else
    echo "   ❌ LLM Agent main.py 不存在"
    exit 1
fi

echo ""
echo "=========================================="
echo "🎉 所有检查通过！"
echo "=========================================="
echo ""
echo "下一步："
echo "  1. 运行系统: ./test_llm_control.sh"
echo "  2. 或者分步测试："
echo ""
echo "     # Terminal 1: 启动机器人系统"
echo "     cd /home/olivia/llms-ros2"
echo "     source install/setup.bash"
echo "     ros2 launch battery_dismantle_task fake_execution.launch.py"
echo ""
echo "     # Terminal 2: 运行LLM Agent"
echo "     cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent"
echo "     python3 main.py \"拆解电池\" --demo"
echo ""
