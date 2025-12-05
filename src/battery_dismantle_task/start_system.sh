#!/bin/bash
################################################################################
# Battery Dismantle Task - 完整系统启动脚本
# 自动启动所有必需的组件：Ollama + ROS2 + Web UI
################################################################################

set -e  # 遇到错误立即退出

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="/home/olivia/llms-ros2"
LLM_AGENT_DIR="$SCRIPT_DIR/llm_agent"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=================================${NC}"
echo -e "${BLUE}🤖 Battery Dismantle Task${NC}"
echo -e "${BLUE}   Complete System Startup${NC}"
echo -e "${BLUE}=================================${NC}"
echo ""

################################################################################
# 步骤1: 检查并启动Ollama
################################################################################
echo -e "${YELLOW}[1/4] 检查Ollama服务...${NC}"

if ! command -v ollama &> /dev/null; then
    echo -e "${RED}❌ Ollama未安装！${NC}"
    echo "请运行: curl -fsSL https://ollama.com/install.sh | sh"
    exit 1
fi

if ! pgrep -x "ollama" > /dev/null; then
    echo -e "${YELLOW}⏳ 启动Ollama服务...${NC}"
    ollama serve > /tmp/ollama.log 2>&1 &
    OLLAMA_PID=$!
    sleep 3

    if pgrep -x "ollama" > /dev/null; then
        echo -e "${GREEN}✅ Ollama服务已启动 (PID: $OLLAMA_PID)${NC}"
    else
        echo -e "${RED}❌ Ollama启动失败！请查看日志: /tmp/ollama.log${NC}"
        exit 1
    fi
else
    echo -e "${GREEN}✅ Ollama服务已在运行${NC}"
fi

# 检查模型是否存在
echo -e "${YELLOW}⏳ 检查LLM模型...${NC}"
if ! ollama list | grep -q "gemma2:9b"; then
    echo -e "${YELLOW}📥 模型gemma2:9b未找到，开始下载...${NC}"
    echo -e "${YELLOW}   (这可能需要5-10分钟，取决于网络速度)${NC}"
    ollama pull gemma2:9b
    echo -e "${GREEN}✅ 模型下载完成${NC}"
else
    echo -e "${GREEN}✅ 模型gemma2:9b已安装${NC}"
fi

echo ""

################################################################################
# 步骤2: 编译ROS2工作空间（如果需要）
################################################################################
echo -e "${YELLOW}[2/4] 检查ROS2工作空间...${NC}"

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash

if [ ! -d "install/battery_dismantle_task" ]; then
    echo -e "${YELLOW}⏳ 首次运行，正在编译工作空间...${NC}"
    colcon build --packages-select battery_dismantle_task --symlink-install
    echo -e "${GREEN}✅ 编译完成${NC}"
else
    echo -e "${GREEN}✅ 工作空间已编译${NC}"
fi

source install/setup.bash
echo ""

################################################################################
# 步骤3: 启动ROS2仿真系统
################################################################################
echo -e "${YELLOW}[3/4] 启动ROS2仿真系统...${NC}"
echo -e "${BLUE}   - MoveIt move_group${NC}"
echo -e "${BLUE}   - RViz可视化${NC}"
echo -e "${BLUE}   - Skill Server${NC}"
echo -e "${BLUE}   - Visual State Manager${NC}"
echo ""

# 在新终端中启动ROS2系统
gnome-terminal --tab --title="ROS2 System" -- bash -c "
    cd $WORKSPACE_DIR
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch battery_dismantle_task fake_execution.launch.py start_skill_server:=true
    exec bash
" &

echo -e "${GREEN}✅ ROS2系统正在启动...${NC}"
echo -e "${YELLOW}⏳ 等待15秒让系统完全启动...${NC}"
sleep 15

echo ""

################################################################################
# 步骤4: 启动Web UI
################################################################################
echo -e "${YELLOW}[4/4] 启动Web UI...${NC}"

# 在新终端中启动Web UI
gnome-terminal --tab --title="Web UI" -- bash -c "
    cd $LLM_AGENT_DIR
    python3 web_ui.py
    exec bash
" &

echo -e "${GREEN}✅ Web UI正在启动...${NC}"
echo -e "${YELLOW}⏳ 等待5秒让Web UI启动...${NC}"
sleep 5

echo ""

################################################################################
# 启动完成
################################################################################
echo -e "${GREEN}=================================${NC}"
echo -e "${GREEN}🎉 系统启动完成！${NC}"
echo -e "${GREEN}=================================${NC}"
echo ""
echo -e "${BLUE}📍 访问方式：${NC}"
echo -e "   Web UI:  ${GREEN}http://localhost:7860${NC}"
echo -e "   Ollama:  ${GREEN}http://localhost:11434${NC}"
echo ""
echo -e "${BLUE}🛠️  使用步骤：${NC}"
echo -e "   1. 打开浏览器访问: ${GREEN}http://localhost:7860${NC}"
echo -e "   2. 点击 ${YELLOW}'Initialize System'${NC} 按钮"
echo -e "   3. 输入自然语言指令，例如："
echo -e "      - ${YELLOW}Go to home position${NC}"
echo -e "      - ${YELLOW}Open the gripper${NC}"
echo -e "      - ${YELLOW}Disassemble the battery${NC}"
echo -e "   4. 观察RViz窗口中的机械臂动作"
echo ""
echo -e "${BLUE}🔍 进程信息：${NC}"
echo -e "   Ollama PID: $(pgrep -x ollama || echo 'N/A')"
echo -e "   查看日志: /tmp/ollama.log"
echo ""
echo -e "${BLUE}🛑 停止系统：${NC}"
echo -e "   运行: ${YELLOW}$SCRIPT_DIR/stop_system.sh${NC}"
echo ""
