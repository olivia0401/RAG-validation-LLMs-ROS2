#!/bin/bash
# Quick test script for fake execution launch

cd /home/olivia/llms-ros2
source install/setup.bash

echo "========================================================================"
echo "🚀 Testing fake_execution.launch.py"
echo "========================================================================"
echo ""
echo "This will:"
echo "  1. Launch move_group with fake execution"
echo "  2. Launch RViz for visualization"
echo "  3. Launch skill_server (delayed 5 seconds)"
echo ""
echo "Look for these SUCCESS indicators:"
echo "  ✅ move_group starts without controller errors"
echo "  ✅ skill_server shows: '✅ Skill Server Ready! Listening on /llm_commands'"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================================================"
echo ""

ros2 launch battery_dismantle_task fake_execution.launch.py
