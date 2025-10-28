#!/usr/bin/env python3
"""
完整电池拆解序列测试
测试完整的拆解流程：
1. 拆卸顶盖螺栓 (TopCoverBolts)
2. 移除电池主体 (BatteryBox_0)
3. 返回HOME位置
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class DisassemblyTester(Node):
    def __init__(self):
        super().__init__('disassembly_tester')

        # Publisher for commands
        self.cmd_pub = self.create_publisher(String, '/llm_commands', 10)

        # Subscriber for feedback
        self.feedback_sub = self.create_subscription(
            String,
            '/llm_feedback',
            self.feedback_callback,
            10
        )

        self.latest_feedback = None
        self.get_logger().info('🤖 完整拆解测试初始化完成')
        time.sleep(2)  # 等待连接建立

    def feedback_callback(self, msg):
        """接收skill server的反馈"""
        self.latest_feedback = json.loads(msg.data)
        status = self.latest_feedback.get('status', 'unknown')
        message = self.latest_feedback.get('message', '')

        if status == 'success':
            self.get_logger().info(f'✅ {message}')
        elif status == 'failure':
            self.get_logger().error(f'❌ {message}')
        else:
            self.get_logger().warn(f'⚠️  {message}')

    def send_command(self, skill, target=None, timeout=10.0):
        """发送命令并等待反馈"""
        self.latest_feedback = None

        # 构建命令
        cmd = {"skill": skill}
        if target:
            cmd["target"] = target

        # 发布命令
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)

        skill_desc = f"{skill}({target})" if target else skill
        self.get_logger().info(f'📤 发送命令: {skill_desc}')

        # 等待反馈
        start_time = time.time()
        while self.latest_feedback is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_feedback is None:
            self.get_logger().error(f'⏱️  等待反馈超时: {skill_desc}')
            return False

        return self.latest_feedback.get('status') == 'success'

    def run_complete_disassembly(self):
        """运行完整拆解序列"""
        print("\n" + "="*60)
        print("🔋 开始完整电池拆解测试")
        print("="*60 + "\n")

        sequence = [
            {
                "step": 1,
                "name": "回到HOME位置",
                "skill": "move_to",
                "target": "HOME"
            },
            {
                "step": 2,
                "name": "拆卸顶盖螺栓",
                "skill": "grasp",
                "target": "TopCoverBolts"
            },
            {
                "step": 3,
                "name": "放置顶盖螺栓",
                "skill": "release",
                "target": "TopCoverBolts"
            },
            {
                "step": 4,
                "name": "回到HOME位置",
                "skill": "move_to",
                "target": "HOME"
            },
            {
                "step": 5,
                "name": "抓取电池主体",
                "skill": "grasp",
                "target": "BatteryBox_0"
            },
            {
                "step": 6,
                "name": "放置电池主体",
                "skill": "release",
                "target": "BatteryBox_0"
            },
            {
                "step": 7,
                "name": "最终回到HOME位置",
                "skill": "move_to",
                "target": "HOME"
            }
        ]

        results = []

        for task in sequence:
            step = task["step"]
            name = task["name"]
            skill = task["skill"]
            target = task.get("target")

            print(f"\n📍 步骤 {step}/7: {name}")
            print("-" * 60)

            success = self.send_command(skill, target, timeout=15.0)
            results.append({
                "step": step,
                "name": name,
                "success": success
            })

            if not success:
                print(f"\n❌ 步骤 {step} 失败，终止拆解序列")
                break

            time.sleep(1)  # 短暂停顿

        # 打印总结
        print("\n" + "="*60)
        print("📊 拆解序列测试总结")
        print("="*60)

        total = len(results)
        successful = sum(1 for r in results if r["success"])

        for result in results:
            status = "✅" if result["success"] else "❌"
            print(f"{status} 步骤 {result['step']}: {result['name']}")

        print(f"\n总计: {successful}/{total} 步骤成功")

        if successful == total:
            print("\n🎉 完整拆解序列测试成功！\n")
            return True
        else:
            print(f"\n⚠️  拆解序列部分失败 ({successful}/{total})\n")
            return False

def main():
    rclpy.init()
    tester = DisassemblyTester()

    try:
        success = tester.run_complete_disassembly()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断测试")
        return 1
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    import sys
    sys.exit(main())
