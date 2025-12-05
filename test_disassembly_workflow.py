#!/usr/bin/env python3
"""
完整电池拆卸流程测试脚本
测试顺序：
1. 移动到HOME位置
2. 打开夹爪
3. 抓取TopCoverBolts（顶盖螺栓）
4. 放置到托盘
5. 抓取BatteryBox_0（电池主体）
6. 放置到回收箱
7. 返回HOME
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class DisassemblyTester(Node):
    def __init__(self):
        super().__init__('disassembly_tester')
        self.publisher = self.create_publisher(String, '/llm_commands', 10)
        self.feedback_sub = self.create_subscription(
            String, '/llm_feedback', self.feedback_callback, 10
        )
        self.last_feedback = None
        self.command_counter = 0

    def feedback_callback(self, msg):
        """接收反馈"""
        try:
            feedback = json.loads(msg.data)
            self.last_feedback = feedback
            status = feedback.get('status', 'unknown')
            message = feedback.get('message', '')
            cmd_id = feedback.get('command_id', '')

            if status == 'success':
                self.get_logger().info(f"✅ [{cmd_id}] {message}")
            elif status == 'progress':
                self.get_logger().info(f"🔄 [{cmd_id}] {message}")
            elif status == 'failure':
                self.get_logger().error(f"❌ [{cmd_id}] {message}")
            elif status == 'rejected':
                self.get_logger().error(f"🚫 [{cmd_id}] {message}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"无法解析反馈: {msg.data}")

    def send_command(self, skill, params=None, target=None, context=None, wait_time=5.0):
        """发送命令并等待反馈"""
        self.command_counter += 1
        cmd_id = f"test_{self.command_counter}"

        cmd = {
            "schema": "llm_cmd/v1",
            "command_id": cmd_id,
            "skill": skill
        }

        if params:
            cmd["params"] = params
        if target:
            cmd["target"] = target
        if context:
            cmd["context"] = context

        msg = String()
        msg.data = json.dumps(cmd)

        self.get_logger().info(f"📤 发送命令: {skill} (ID: {cmd_id})")
        self.last_feedback = None
        self.publisher.publish(msg)

        # 等待反馈
        start = time.time()
        while time.time() - start < wait_time:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_feedback and self.last_feedback.get('command_id') == cmd_id:
                status = self.last_feedback.get('status')
                if status in ['success', 'failure', 'rejected']:
                    return status == 'success'

        self.get_logger().warn(f"⏱️ 命令超时: {skill}")
        return False

    def run_workflow(self):
        """执行完整拆卸流程"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("🚀 开始电池拆卸流程测试")
        self.get_logger().info("=" * 60)

        steps = [
            ("Step 1: 移动到HOME位置", "moveTo", {"target": "HOME"}),
            ("Step 2: 打开夹爪", "openGripper", {}),
            ("Step 3: 选择目标 - TopCoverBolts", "selectObject", {"params": {"object_id": "TopCoverBolts"}}),
            ("Step 4: 抓取TopCoverBolts", "grasp", {"target": "TopCoverBolts"}),
            ("Step 5: 放置TopCoverBolts到托盘", "release", {"target": "TopCoverBolts"}),
            ("Step 6: 移动到HOME", "moveTo", {"target": "HOME"}),
            ("Step 7: 选择目标 - BatteryBox_0", "selectObject", {"params": {"object_id": "BatteryBox_0"}}),
            ("Step 8: 抓取BatteryBox_0", "grasp", {"target": "BatteryBox_0"}),
            ("Step 9: 放置BatteryBox_0到回收箱", "release", {"target": "BatteryBox_0"}),
            ("Step 10: 返回HOME", "moveTo", {"target": "HOME"}),
        ]

        success_count = 0
        for i, (description, skill, kwargs) in enumerate(steps, 1):
            self.get_logger().info(f"\n{'─' * 60}")
            self.get_logger().info(f"{description}")
            self.get_logger().info(f"{'─' * 60}")

            success = self.send_command(skill, **kwargs, wait_time=15.0)

            if success:
                success_count += 1
                self.get_logger().info(f"✅ Step {i} 成功")
            else:
                self.get_logger().error(f"❌ Step {i} 失败")
                response = input(f"\n是否继续下一步? (y/n): ")
                if response.lower() != 'y':
                    self.get_logger().warn("⚠️ 用户终止测试")
                    break

            # 步骤间短暂延迟
            time.sleep(0.5)

        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info(f"🏁 测试完成: {success_count}/{len(steps)} 步骤成功")
        self.get_logger().info("=" * 60)

        return success_count == len(steps)


def main():
    rclpy.init()
    tester = DisassemblyTester()

    # 等待系统稳定
    print("\n⏳ 等待3秒让系统稳定...")
    time.sleep(3)

    try:
        success = tester.run_workflow()
        if success:
            print("\n🎉 完整拆卸流程测试成功！")
        else:
            print("\n⚠️ 拆卸流程部分失败")
    except KeyboardInterrupt:
        print("\n⚠️ 测试被用户中断")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
