#!/usr/bin/env python3
"""
实时监控joint_states，验证动画是否在工作
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)
        self.last_position = None
        self.count = 0

    def callback(self, msg):
        if len(msg.position) < 8:
            return

        arm_pos = msg.position[0]  # joint_1
        gripper_pos = msg.position[7]  # gripper

        # 检测变化
        if self.last_position is not None:
            arm_change = abs(arm_pos - self.last_position[0])
            grip_change = abs(gripper_pos - self.last_position[1])

            if arm_change > 0.001 or grip_change > 0.001:
                print(f"🟢 [{self.count}] 运动检测! arm={arm_pos:.3f} (Δ{arm_change:.3f}), gripper={gripper_pos:.3f} (Δ{grip_change:.3f})")
            else:
                print(f"⚪ [{self.count}] 静止: arm={arm_pos:.3f}, gripper={gripper_pos:.3f}")
        else:
            print(f"📍 [{self.count}] 初始: arm={arm_pos:.3f}, gripper={gripper_pos:.3f}")

        self.last_position = (arm_pos, gripper_pos)
        self.count += 1

def main():
    rclpy.init()
    monitor = JointMonitor()

    print("=" * 60)
    print("🔍 监控joint_states变化...")
    print("=" * 60)
    print("现在在另一个终端运行:")
    print("  python3 /home/olivia/llms-ros2/src/battery_dismantle_task/llm_agent/main.py --demo")
    print("=" * 60)

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n监控结束")

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
