# 🤖 如何切换到真实Kinova机器人

## 当前状态：Fake模式 ✅
- ✅ 所有功能已验证
- ✅ 平滑动画正常工作（5秒机械臂，3秒夹爪）
- ✅ RViz可视化完美
- ✅ 任务规划和执行100%成功

---

## 切换到真实硬件的步骤

### 前提条件：
1. ✅ Kinova Gen3 7-DOF 机械臂已连接电源并开机
2. ✅ Robotiq 2F-85 夹爪已安装在机械臂末端
3. ✅ 机器人通过网线连接到电脑（通常是192.168.1.10）
4. ✅ 清空工作区，确保机器人周围无障碍物

---

### 方法1：使用Kortex驱动（推荐）

#### 步骤1：测试与真实机器人的连接

```bash
cd /home/olivia/llms-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 测试连接（替换为你的机器人IP）
ros2 launch kortex_bringup kortex_robot.launch.py robot_ip:=192.168.1.10
```

如果成功，你应该看到：
```
[INFO] [kortex_driver]: Successfully connected to robot at 192.168.1.10
```

按 `Ctrl+C` 停止测试。

#### 步骤2：创建真实硬件launch文件

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/launch
cp fake_execution.launch.py real_execution.launch.py
```

编辑 `real_execution.launch.py`，修改第48行：
```python
# 从
mappings={"use_fake_hardware": "true", ...}

# 改为
mappings={"use_fake_hardware": "false", "robot_ip": "192.168.1.10", ...}
```

#### 步骤3：启动真实系统

```bash
cd /home/olivia/llms-ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# 启动真实机器人系统
ros2 launch battery_dismantle_task real_execution.launch.py start_skill_server:=true
```

#### 步骤4：测试真实执行

在另一个终端：
```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/llm_agent

# ⚠️ 警告：机器人会真实移动！
# 确保周围无障碍物！
python3 main.py --demo
```

---

### 方法2：仅使用MoveIt (不通过Kortex驱动)

这种方法需要你自己实现trajectory_execution接口。

**不推荐** - Kortex驱动已经提供了完整的支持。

---

## ⚠️ 安全注意事项

### 在运行真实机器人之前：

1. **🔴 急停按钮** - 确保急停按钮在手边
2. **🟡 工作空间** - 确保机器人工作区域清空
3. **🟢 起始位置** - 让机器人先到达HOME位置
4. **🔵 慢速测试** - 第一次执行时可以降低速度：
   ```python
   # 在waypoints.json中修改：
   "defaults": {
       "velocity_scaling": 0.3,  # 降到30%速度
       "accel_scaling": 0.3
   }
   ```

5. **⚪ 监控日志** - 观察skill_server日志，确保没有错误

---

## 故障排除

### 问题1：连接不到机器人

**解决方案：**
```bash
# 检查网络连接
ping 192.168.1.10

# 检查机器人状态指示灯
# - 绿色：正常
# - 红色：错误
# - 黄色：警告
```

### 问题2：机器人进入错误状态

**解决方案：**
1. 按急停按钮
2. 重启机器人电源
3. 释放急停，等待机器人初始化
4. 重新运行launch文件

### 问题3：轨迹执行失败

**检查：**
- 关节限位
- 碰撞检测设置
- 速度/加速度限制
- 工作空间边界

---

## 调试建议

### 逐步测试：

1. **只启动机器人驱动**（不启动skill_server）
   ```bash
   ros2 launch kortex_bringup kortex_robot.launch.py robot_ip:=192.168.1.10
   ```

2. **测试单个关节运动**
   ```bash
   ros2 topic pub /joint_trajectory_controller/joint_trajectory \
       trajectory_msgs/msg/JointTrajectory ...
   ```

3. **测试MoveIt规划**
   - 在RViz中手动拖动末端执行器
   - 点击"Plan"按钮
   - 检查是否生成有效轨迹

4. **最后启动完整系统**
   ```bash
   ros2 launch battery_dismantle_task real_execution.launch.py
   ```

---

## 性能对比

| 特性 | Fake模式 | 真实模式 |
|------|----------|---------|
| 安全性 | ✅ 完全安全 | ⚠️ 需要监督 |
| 开发速度 | ✅ 快速迭代 | ❌ 需要硬件 |
| 碰撞检测 | ✅ 仿真碰撞 | ✅ 真实碰撞 |
| 力控制 | ❌ 不支持 | ✅ 支持 |
| 视觉反馈 | ❌ 仅RViz | ✅ 真实物体 |
| 执行速度 | ⚡ 即时动画 | 🐢 实际物理限制 |

---

## 下一步计划

一旦真实硬件测试成功：

1. **校准** - 微调waypoints位置以匹配真实环境
2. **力控制** - 添加接触力检测
3. **视觉** - 集成相机进行物体检测
4. **错误恢复** - 实现抓取失败重试
5. **生产部署** - 创建稳定的production launch文件

---

## 🎯 总结

**Fake模式适合：**
- ✅ 算法开发和测试
- ✅ 界面和可视化开发
- ✅ 任务规划逻辑验证

**真实模式适合：**
- ✅ 最终验证
- ✅ 性能测试
- ✅ 实际生产任务

**目前系统状态：** 已经在Fake模式下完全验证，可以安全地切换到真实硬件！🚀
