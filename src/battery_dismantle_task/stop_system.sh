#!/bin/bash
################################################################################
# Battery Dismantle Task - 系统停止脚本
# 优雅地停止所有组件
################################################################################

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}=================================${NC}"
echo -e "${YELLOW}🛑 停止Battery Dismantle Task${NC}"
echo -e "${YELLOW}=================================${NC}"
echo ""

# 停止ROS2节点
echo -e "${YELLOW}⏳ 停止ROS2节点...${NC}"
pkill -f "ros2 launch battery_dismantle_task" || true
pkill -f "skill_server_node" || true
pkill -f "rviz2" || true
pkill -f "move_group" || true
sleep 2
echo -e "${GREEN}✅ ROS2节点已停止${NC}"

# 停止Web UI
echo -e "${YELLOW}⏳ 停止Web UI...${NC}"
pkill -f "web_ui.py" || true
pkill -f "gradio" || true
sleep 1
echo -e "${GREEN}✅ Web UI已停止${NC}"

# 停止Ollama（可选）
echo ""
read -p "是否停止Ollama服务？(y/N): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo -e "${YELLOW}⏳ 停止Ollama...${NC}"
    pkill -x ollama || true
    sleep 1
    echo -e "${GREEN}✅ Ollama已停止${NC}"
else
    echo -e "${YELLOW}⏭️  保持Ollama运行${NC}"
fi

echo ""
echo -e "${GREEN}=================================${NC}"
echo -e "${GREEN}✅ 系统已停止${NC}"
echo -e "${GREEN}=================================${NC}"
