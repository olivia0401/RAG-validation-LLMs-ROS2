# LLM 控制机器人完整演示

## 演示 1: Mock 模式（立即可用）

这个演示展示完整的 LLM → 规划 → 验证 → 执行流程，使用模拟执行。

### 运行步骤：

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent
python3 main.py "拆解电池顶盖螺栓" --demo --mock
```

### 你会看到：

```
============================================================
🤖 LLM Robot Agent
============================================================
📝 Task: 拆解电池顶盖螺栓
🧠 LLM: Demo Plan
🤖 Executor: Mock
============================================================

============================================================
1️⃣  PLANNING PHASE
============================================================
📋 Using demo plan...

✅ Generated Plan:
  Step 1: grasp({'target': 'TopCoverBolts'})
  Step 2: release({'target': 'TopCoverBolts'})

💾 Plan saved to: .../outputs/plan.json

============================================================
2️⃣  VALIDATION PHASE
============================================================
✅ Plan validation passed

============================================================
3️⃣  EXECUTION PHASE
============================================================

🚀 Executing plan (2 steps)...

📍 Step 1/2: grasp(target=TopCoverBolts)
   🎭 Mock execution
   ✅ Success

📍 Step 2/2: release(target=TopCoverBolts)
   🎭 Mock execution
   ✅ Success

📊 Execution Summary:
   Total: 2
   Executed: 2
   Failed: 0

============================================================
🎉 MISSION SUCCESS
============================================================
```

**说明**：
- ✅ 展示完整的任务规划流程
- ✅ 展示计划验证
- ✅ 展示技能序列执行
- ✅ 展示成功/失败反馈
- ⚠️  但是没有真实的机器人运动（Mock 模式）

---

## 演示 2: ROS2 通信测试（部分可用）

这个演示展示 LLM Agent 与 Skill Server 的 ROS2 通信。

### Terminal 1: 启动 Skill Server

注意：虽然控制器有问题，但 skill_server 可以启动并接收命令。

```bash
cd /home/olivia/llms-ros2
source install/setup.bash
ros2 launch battery_dismantle_task simple_demo.launch.py
```

等待看到：
```
[skill_server-9] [INFO] [skill_server]: ✅ Skill Server Ready! Listening on /llm_commands
```

### Terminal 2: 手动发送测试命令

```bash
cd /home/olivia/llms-ros2
source install/setup.bash

# 测试 moveTo 命令
ros2 topic pub --once /llm_commands std_msgs/msg/String \
  '{"data": "{\"skill\": \"moveTo\", \"target\": \"HOME\"}"}'
```

### Terminal 3: 监听反馈

```bash
cd /home/olivia/llms-ros2
source install/setup.bash
ros2 topic echo /llm_feedback
```

你会看到：
```json
{
  "status": "failure",
  "message": "Skill 'moveTo' failed",
  "timestamp": 1761480617698046107
}
```

**说明**：
- ✅ Skill Server 成功接收命令
- ✅ 发布 JSON 格式反馈
- ❌ 执行失败（因为控制器问题导致机器人状态初始化失败）

---

## 演示 3: 完整的 LLM Agent + ROS2（通信正常）

这个演示展示 LLM Agent 通过 ROS2 与 Skill Server 通信的完整流程。

### Terminal 1: 启动系统（如果还没启动）

```bash
cd /home/olivia/llms-ros2
source install/setup.bash
ros2 launch battery_dismantle_task simple_demo.launch.py
```

### Terminal 2: 运行 LLM Agent（ROS2 模式）

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/LLM_Robot_Agent
python3 main.py "测试任务" --demo
```

你会看到：
```
✅ ROS2 executor initialized
============================================================
🤖 LLM Robot Agent
============================================================
📝 Task: 测试任务
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
   📤 Published command: {"skill": "grasp", "target": "TopCoverBolts"}
   📥 Feedback: {"message":"Skill 'grasp' failed","status":"failure",...}
   ❌ Failed
```

**说明**：
- ✅ LLM Agent → Skill Server 通信完全正常
- ✅ 命令发送成功
- ✅ 反馈接收和解析成功
- ❌ 技能执行失败（因为机器人初始状态问题）

---

## 问题诊断

### 当前问题根源

**症状**：所有 moveTo/grasp/release 技能都失败

**原因**：
```
[move_group] Start state appears to be in collision with respect to group manipulator
[move_group] Start state appears to be in collision with respect to group gripper
```

**根本原因**：
- ros2_control 的控制器（`joint_state_broadcaster`, `joint_trajectory_controller`）加载失败
- 导致机器人的关节状态无法正确发布
- MoveIt 无法获取正确的机器人状态
- 规划器认为起始状态处于碰撞中

### 为什么控制器加载失败？

查看日志：
```
[ERROR] Could not configure controller with name 'joint_trajectory_controller'
because no controller with this name exists
```

这意味着：
1. URDF 中声明了控制器（通过 xacro）
2. 但 ros2_controllers.yaml 中的配置没有被正确加载
3. 或者配置格式不匹配

---

## 解决方案选项

### 选项 A: 使用 MoveIt Fake Execution（推荐）

**优势**：
- 不需要 ros2_control
- 直接在 MoveIt 内部模拟
- RViz 可以看到机器人运动
- 完全满足你的需求（只要 RViz 可视化）

**需要做的**：
1. 修改 `moveit_controllers.yaml` 使用 `moveit_fake_controller_manager`
2. 或者创建新的 launch 文件完全跳过 ros2_control
3. 设置 `fake_execution: true`

### 选项 B: 修复 ros2_control 配置

**需要做的**：
1. 参考 Windows 成功版本的配置
2. 或者参考 kortex_ros2 官方示例
3. 调试控制器加载流程
4. 可能需要 1-2 小时

### 选项 C: 使用 Mock 模式展示（最快）

**优势**：
- 立即可用
- 展示完整流程
- 验证架构

**限制**：
- 看不到 RViz 中的机器人运动

---

## 我的建议

既然你说"只需要 RViz 和 MoveIt 控制就好了"，那我们应该：

1. **立即**：运行 Mock 模式演示，验证整个流程（演示 1）
2. **然后**：我帮你修改配置，使用 MoveIt Fake Execution
3. **最后**：在 RViz 中看到机器人执行 LLM 的命令

你想先看哪个演示？或者我直接帮你配置 MoveIt Fake Execution？
