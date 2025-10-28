# 🚀 LLM控制机器人 - 启动指南

## ✅ 修复完成！

已经修复了 `fake_execution.launch.py` 中的冲突问题，现在机器人可以在RViz中正常运动了。

## 🎯 启动步骤

### Terminal 1: 启动机器人系统

```bash
cd /home/olivia/llms-ros2
source install/setup.bash
ros2 launch battery_dismantle_task fake_execution.launch.py
```

**等待启动完成** - 当您看到以下信息时表示系统就绪：
```
[skill_server]: ✅ Skill Server Ready! Listening on /llm_commands
```

**您将看到：**
- ✅ RViz窗口打开，显示Kinova Gen3机器人手臂
- ✅ move_group节点运行（运动规划）
- ✅ skill_server节点等待接收命令
- ✅ 电池场景被加载到规划场景中

---

### Terminal 2: 运行LLM Agent

打开**新的终端**，运行：

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent
python3 main.py "拆解电池顶盖螺栓" --demo
```

**您将看到：**

```
============================================================
🤖 LLM Robot Agent
============================================================
📝 Task: 拆解电池顶盖螺栓
🧠 LLM: Demo Plan
🤖 Executor: ROS2
============================================================

============================================================
1️⃣  PLANNING PHASE
============================================================
📋 Using demo plan...

✅ Generated Plan:
  Step 1: grasp({'target': 'TopCoverBolts'})
  Step 2: release({'target': 'TopCoverBolts'})

============================================================
2️⃣  VALIDATION PHASE
============================================================
✅ Plan validation passed

============================================================
3️⃣  EXECUTION PHASE
============================================================

🚀 Executing plan (2 steps)...

📍 Step 1/2: grasp(target=TopCoverBolts)
   📤 Published command to skill_server
   📥 Waiting for feedback...
   ✅ Success (观察RViz中机器人运动！)

📍 Step 2/2: release(target=TopCoverBolts)
   📤 Published command to skill_server
   📥 Waiting for feedback...
   ✅ Success (观察RViz中机器人运动！)

============================================================
🎉 MISSION SUCCESS
============================================================
```

**在RViz中观察：**
- 🤖 机器人手臂移动到抓取位置
- ✋ 夹爪关闭（grasp）
- ✋ 夹爪打开（release）
- 🔄 所有动作平滑执行

---

## 🧪 其他测试选项

### 选项1: 手动测试单个技能

在Terminal 3中运行：

```bash
cd /home/olivia/llms-ros2
source install/setup.bash

# 测试moveTo技能
ros2 topic pub --once /llm_commands std_msgs/msg/String \
  '{"data": "{\"skill\": \"moveTo\", \"target\": \"HOME\"}"}'

# 查看反馈
ros2 topic echo /llm_feedback
```

### 选项2: Mock模式（不需要ROS2）

如果您只想测试LLM规划逻辑，不需要机器人运动：

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent
python3 main.py "拆解电池顶盖螺栓" --demo --mock
```

---

## 📊 可用的技能

Skill Server支持以下技能：

1. **moveTo(target)** - 移动到预定义姿态
   - 参数: `target` = "HOME", "APPROACH_BOLT", "PLACE_BOLT" 等
   - 示例: `{"skill": "moveTo", "target": "HOME"}`

2. **grasp(target)** - 抓取物体
   - 参数: `target` = 物体名称
   - 动作: 移动到目标位置 → 关闭夹爪 → attach物体
   - 示例: `{"skill": "grasp", "target": "TopCoverBolts"}`

3. **release(target)** - 释放物体
   - 参数: `target` = 物体名称
   - 动作: detach物体 → 打开夹爪 → 移动到HOME
   - 示例: `{"skill": "release", "target": "TopCoverBolts"}`

---

## ❓ 故障排除

### 问题1: RViz没有打开
- 检查是否在WSL2中运行，需要X11转发
- 设置: `export DISPLAY=:0`

### 问题2: skill_server未启动
- 检查Terminal 1的输出
- 确保没有错误信息
- 等待5-10秒让所有节点初始化

### 问题3: 机器人不动
- 检查 `/joint_states` 话题是否有数据: `ros2 topic echo /joint_states`
- 检查move_group是否运行: `ros2 node list | grep move_group`

### 问题4: LLM Agent连接失败
- 确保Terminal 1的系统已经启动
- 检查 `/llm_commands` 话题是否存在: `ros2 topic list | grep llm`

---

## 🎓 下一步

1. ✅ **测试Demo模式** - 验证系统工作正常
2. 🧠 **配置真实LLM** - 移除 `--demo` 参数，配置API key
3. 📝 **自定义任务** - 修改waypoints.json添加新的动作
4. 🔧 **扩展技能** - 在skill_server_node.cpp中添加新技能

---

## 📁 重要文件

- **主程序**: `src/battery_dismantle_task/LLM_Robot_Agent/main.py`
- **启动文件**: `src/battery_dismantle_task/launch/fake_execution.launch.py`
- **Skill Server**: `src/battery_dismantle_task/src/skill_server_node.cpp`
- **配置文件**: `src/battery_dismantle_task/config/waypoints.json`

---

## 🎉 开始测试吧！

按照上面的步骤启动两个终端，观察LLM如何控制机器人拆解电池！
