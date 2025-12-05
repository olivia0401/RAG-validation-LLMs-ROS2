#!/usr/bin/env python3
"""
一键入口 - LLM → Plan → Validate → Execute → Replan

用法:
    python3 main.py "disassemble the battery"
    python3 main.py --demo  # 使用demo计划
    python3 main.py --mock  # Mock执行(不需要ROS2)
"""
import sys
import asyncio
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict

from planner import Planner
from validator import Validator
from executor import Executor


class LLMRobotAgent:
    """LLM机器人Agent - 完整流程"""

    def __init__(self, use_llm: bool = True, use_ros: bool = True):
        """
        初始化Agent

        Args:
            use_llm: 是否使用真实LLM
            use_ros: 是否使用真实ROS2
        """
        self.planner = Planner()
        self.validator = Validator()
        self.executor = Executor(use_ros=use_ros)
        self.use_llm = use_llm

        # 输出目录
        self.output_dir = Path(__file__).parent / "outputs"
        self.output_dir.mkdir(exist_ok=True)

    async def run(self, task: str, max_retries: int = 2) -> bool:
        """
        运行完整流程

        Args:
            task: 任务描述
            max_retries: 失败重试次数

        Returns:
            是否成功
        """
        print("=" * 60)
        print("🤖 LLM Robot Agent")
        print("=" * 60)
        print(f"📝 Task: {task}")
        print(f"🧠 LLM: {'Enabled' if self.use_llm else 'Demo Plan'}")
        print(f"🤖 Executor: {'ROS2' if self.executor.use_ros else 'Mock'}")
        print("=" * 60)

        attempt = 0
        while attempt <= max_retries:
            if attempt > 0:
                print(f"\n🔄 Retry attempt {attempt}/{max_retries}...")

            # 1️⃣ 规划
            print("\n" + "="*60)
            print("1️⃣  PLANNING PHASE")
            print("="*60)

            plan = await self.planner.plan(task, use_llm=self.use_llm)
            self.planner.print_plan(plan)

            # 保存计划
            self._save_plan(plan)

            # 2️⃣ 验证
            print("\n" + "="*60)
            print("2️⃣  VALIDATION PHASE")
            print("="*60)

            is_valid, errors = self.validator.validate_plan(plan)

            if not is_valid:
                print("❌ Plan validation failed:")
                for error in errors:
                    print(f"   - {error}")

                if attempt >= max_retries:
                    print("\n❌ Max retries reached. Aborting.")
                    return False

                attempt += 1
                continue

            print("✅ Plan validation passed")

            # 3️⃣ 执行
            print("\n" + "="*60)
            print("3️⃣  EXECUTION PHASE")
            print("="*60)

            results = self.executor.execute(plan)

            # 保存日志
            self._save_log(results)

            if results['success']:
                print("\n" + "="*60)
                print("🎉 MISSION SUCCESS")
                print("="*60)
                return True
            else:
                print("\n" + "="*60)
                print("⚠️  EXECUTION FAILED")
                print("="*60)

                if attempt >= max_retries:
                    print("❌ Max retries reached. Aborting.")
                    return False

                # TODO: 实现智能重规划
                print("🔄 Would replan here (not implemented yet)")
                attempt += 1

        return False

    def _save_plan(self, plan: Dict):
        """保存计划到文件"""
        filepath = self.output_dir / "plan.json"
        with open(filepath, 'w') as f:
            json.dump(plan, f, indent=2)
        print(f"\n💾 Plan saved to: {filepath}")

    def _save_log(self, results: Dict):
        """保存执行日志"""
        filepath = self.output_dir / "run_log.txt"

        with open(filepath, 'a') as f:
            f.write(f"\n{'='*60}\n")
            f.write(f"Execution at {datetime.now()}\n")
            f.write(f"{'='*60}\n")
            f.write(f"Success: {results['success']}\n")
            f.write(f"Executed: {results['executed']}\n")
            f.write(f"Failed: {results['failed']}\n")
            f.write(f"\nDetailed Log:\n")

            for entry in results['log']:
                f.write(f"  Step {entry['step']}: {entry['action']} - ")
                f.write(f"{'SUCCESS' if entry['success'] else 'FAILED'}\n")

        print(f"📝 Log saved to: {filepath}")

    def shutdown(self):
        """关闭Agent"""
        self.executor.shutdown()


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='LLM Robot Agent')
    parser.add_argument('task', nargs='?', default="disassemble the battery",
                       help='Task description')
    parser.add_argument('--demo', action='store_true',
                       help='Use demo plan instead of LLM')
    parser.add_argument('--mock', action='store_true',
                       help='Mock execution (no ROS2 required)')

    args = parser.parse_args()

    # 创建Agent
    agent = LLMRobotAgent(
        use_llm=not args.demo,
        use_ros=not args.mock
    )

    try:
        # 运行
        success = await agent.run(args.task)

        # 退出
        agent.shutdown()

        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        agent.shutdown()
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        agent.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
