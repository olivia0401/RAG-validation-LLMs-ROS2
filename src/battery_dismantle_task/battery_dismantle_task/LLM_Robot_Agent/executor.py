"""
执行器 - ROS2 or Mock
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, List


class Executor:
    """任务执行器 - 通过ROS2话题与机器人通信"""

    def __init__(self, use_ros: bool = True):
        """
        初始化执行器

        Args:
            use_ros: 是否使用真实ROS2(False则mock)
        """
        self.use_ros = use_ros
        self.node = None
        self.command_pub = None
        self.feedback_sub = None
        self.last_feedback = None

        if use_ros:
            self._init_ros()

    def _init_ros(self):
        """初始化ROS2节点"""
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('llm_executor')
        self.command_pub = self.node.create_publisher(String, '/llm_commands', 10)
        self.feedback_sub = self.node.create_subscription(
            String,
            '/llm_feedback',
            self._feedback_callback,
            10
        )

        print("✅ ROS2 executor initialized")

        # Wait for topic connections to establish
        print("⏳ Waiting for skill_server connection...")
        time.sleep(3.0)

        # Spin once to process any pending callbacks
        rclpy.spin_once(self.node, timeout_sec=0.1)
        print("✅ Ready to send commands")

    def _feedback_callback(self, msg):
        """接收反馈"""
        self.last_feedback = msg.data
        print(f"   📥 Feedback: {msg.data}")

    def execute(self, plan: Dict, timeout: float = 120.0) -> Dict:
        """
        执行计划

        Args:
            plan: 计划字典
            timeout: 每个动作的超时时间(秒)

        Returns:
            执行结果 {"success": bool, "executed": int, "failed": int, "log": [...]}
        """
        actions = plan['plan']
        results = {
            "success": True,
            "executed": 0,
            "failed": 0,
            "log": []
        }

        print(f"\n🚀 Executing plan ({len(actions)} steps)...")

        for i, action in enumerate(actions, 1):
            skill = action['name']
            target = action['params']['target']

            print(f"\n📍 Step {i}/{len(actions)}: {skill}(target={target})")

            if self.use_ros:
                success = self._execute_ros(action, timeout)
            else:
                success = self._execute_mock(action)

            # 记录结果
            log_entry = {
                "step": i,
                "action": action,
                "success": success,
                "timestamp": time.time()
            }
            results['log'].append(log_entry)

            if success:
                results['executed'] += 1
                print(f"   ✅ Success")
            else:
                results['failed'] += 1
                results['success'] = False
                print(f"   ❌ Failed")
                # 是否继续?
                break

            time.sleep(0.5)  # 短暂延迟

        print(f"\n📊 Execution Summary:")
        print(f"   Total: {len(actions)}")
        print(f"   Executed: {results['executed']}")
        print(f"   Failed: {results['failed']}")

        return results

    def _execute_ros(self, action: Dict, timeout: float) -> bool:
        """通过ROS2执行"""
        # 构造命令
        command = {
            "skill": action['name'],
            "target": action['params']['target']
        }

        # 发布命令
        msg = String()
        msg.data = json.dumps(command)
        self.command_pub.publish(msg)
        print(f"   📤 Published command: {msg.data}")

        # 等待反馈
        self.last_feedback = None
        start_time = time.time()

        while self.last_feedback is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)

            if time.time() - start_time > timeout:
                print(f"   ⏱️  Timeout after {timeout}s")
                return False

        # 解析JSON反馈: {"status": "success", "message": "...", "timestamp": ...}
        try:
            feedback = json.loads(self.last_feedback)
            return feedback.get("status") == "success"
        except (json.JSONDecodeError, KeyError):
            # 兼容简单字符串反馈
            return self.last_feedback == "success"

    def _execute_mock(self, action: Dict) -> bool:
        """Mock执行 (用于测试)"""
        time.sleep(0.2)  # 模拟执行时间
        print(f"   🎭 Mock execution")
        return True  # 总是成功

    def shutdown(self):
        """关闭执行器"""
        if self.use_ros and self.node:
            self.node.destroy_node()
            # 注意：不调用rclpy.shutdown()，因为其他ROS2节点可能还在使用
            # rclpy.shutdown()


# 测试
def test():
    # Mock测试
    executor = Executor(use_ros=False)

    plan = {
        "plan": [
            {"step": 1, "name": "moveTo", "params": {"target": "HOME"}},
            {"step": 2, "name": "grasp", "params": {"target": "TopCoverBolts"}}
        ]
    }

    results = executor.execute(plan)
    print(f"\nResults: {results['success']}")


if __name__ == "__main__":
    test()
