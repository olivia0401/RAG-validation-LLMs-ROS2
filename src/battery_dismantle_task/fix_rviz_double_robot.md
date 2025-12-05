# 修复RViz显示两个手臂的问题

## 问题原因

RViz的MoveIt Motion Planning插件默认显示两个机器人模型：
1. **Scene Robot** (实心) - 当前实际位置
2. **Planned Path** (半透明) - 规划的轨迹动画

这是**正常的MoveIt功能**，不是bug。

---

## 解决方案

### 方法1：在RViz中手动关闭（推荐）

1. 打开RViz窗口
2. 在左侧Displays面板中，找到 **MotionPlanning**
3. 展开 **MotionPlanning** → **Planned Path**
4. **取消勾选** `Show Robot Visual`

**结果**：只显示一个实心手臂（当前状态）

---

### 方法2：只显示轨迹动画

1. 展开 **MotionPlanning** → **Scene Robot**
2. **取消勾选** `Show Robot Visual`
3. 保持 **Planned Path** → `Show Robot Visual` 为勾选

**结果**：只显示半透明的轨迹动画

---

### 方法3：同时显示但降低透明度（默认）

保持两个都显示，通过调整Alpha值区分：
- **Scene Robot** → `Robot Alpha: 1.0` (完全不透明)
- **Planned Path** → `Robot Alpha: 0.4` (半透明)

**好处**：
- 实心手臂 = 当前位置
- 半透明手臂 = 即将执行的动作预览

---

## 如果还是显示两个实心手臂

检查是否有重复的Display：

1. 查看Displays面板
2. 检查是否有**两个RobotModel** display
3. 或者有**两个MotionPlanning** display
4. 删除多余的

---

## 快速验证

运行以下命令检查是否有重复的robot_state_publisher：

```bash
ros2 node list | grep robot_state_publisher
```

**正常输出**：只有一行
```
/robot_state_publisher
```

**异常输出**：多行（需要清理旧进程）

---

## 重置RViz配置

如果以上方法都不行，删除RViz配置重新生成：

```bash
cd /home/olivia/llms-ros2/src/battery_dismantle_task/config
mv moveit.rviz moveit.rviz.backup
# 重新启动系统会生成默认配置
```
