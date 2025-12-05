#!/usr/bin/env python3
"""
测试UI输入指令与实际执行的匹配性
模拟Web UI的完整流程：自然语言 → LLM → Validator → Executor
"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'llm_agent'))

import asyncio
from planner import Planner
from validator import Validator
from executor import Executor
import json
from datetime import datetime


class UICommandMatchingTest:
    """测试UI命令匹配的完整流程"""

    def __init__(self):
        print("=" * 70)
        print("🧪 UI命令匹配性测试")
        print("=" * 70)
        print()

        # 初始化组件
        print("📦 初始化组件...")
        self.planner = Planner(backend="ollama")
        self.validator = Validator()
        self.executor = Executor(use_ros=True)
        print("✅ 所有组件初始化完成\n")

        # 记录测试结果
        self.test_results = []

    async def test_command(self, test_name: str, user_input: str, expected_skills: list = None):
        """
        测试单个命令的完整流程

        Args:
            test_name: 测试名称
            user_input: 用户输入的自然语言
            expected_skills: 期望LLM生成的技能列表（用于验证）
        """
        print("=" * 70)
        print(f"测试: {test_name}")
        print("=" * 70)

        result = {
            "test_name": test_name,
            "user_input": user_input,
            "timestamp": datetime.now().isoformat(),
            "steps": []
        }

        # 步骤1: 用户输入
        print(f"📝 用户输入: \"{user_input}\"\n")
        result["steps"].append({
            "stage": "user_input",
            "data": user_input
        })

        # 步骤2: LLM生成计划
        print("🧠 LLM生成计划...")
        try:
            plan = await self.planner.plan(user_input, use_llm=True)
            print(f"✅ 生成计划包含 {len(plan['plan'])} 步\n")

            # 打印详细计划
            print("📋 LLM生成的计划:")
            for step in plan['plan']:
                skill = step['name']
                target = step['params'].get('target', 'N/A')
                print(f"  步骤 {step['step']}: {skill}(target={target})")
            print()

            result["steps"].append({
                "stage": "llm_plan",
                "data": plan,
                "skill_sequence": [s['name'] for s in plan['plan']]
            })

            # 验证期望技能
            if expected_skills:
                actual_skills = [s['name'] for s in plan['plan']]
                if actual_skills == expected_skills:
                    print(f"✅ LLM生成的技能序列匹配期望: {actual_skills}\n")
                else:
                    print(f"⚠️  技能序列不完全匹配!")
                    print(f"   期望: {expected_skills}")
                    print(f"   实际: {actual_skills}\n")

        except Exception as e:
            print(f"❌ LLM规划失败: {e}\n")
            result["error"] = str(e)
            self.test_results.append(result)
            return False

        # 步骤3: Validator验证
        print("🔍 Validator验证计划...")
        is_valid, errors = self.validator.validate_plan(plan)

        result["steps"].append({
            "stage": "validation",
            "valid": is_valid,
            "errors": errors
        })

        if not is_valid:
            print(f"❌ 验证失败: {', '.join(errors)}\n")
            result["validation_failed"] = True
            self.test_results.append(result)
            return False

        print("✅ 计划通过验证\n")

        # 步骤4: Executor执行
        print("🚀 Executor执行计划...")
        print("-" * 70)

        try:
            execution_result = self.executor.execute(plan, timeout=30.0)

            result["steps"].append({
                "stage": "execution",
                "data": execution_result
            })

            print("-" * 70)
            print()

            # 步骤5: 验证执行结果与计划的匹配
            print("🔍 验证执行匹配性:")
            print(f"  计划步数: {len(plan['plan'])}")
            print(f"  执行成功: {execution_result['executed']}")
            print(f"  执行失败: {execution_result['failed']}")

            # 详细对比每一步
            print("\n📊 逐步对比:")
            all_matched = True
            for i, (planned_step, executed_log) in enumerate(zip(plan['plan'], execution_result['log']), 1):
                planned_skill = planned_step['name']
                planned_target = planned_step['params']['target']

                executed_skill = executed_log['action']['name']
                executed_target = executed_log['action']['params']['target']
                executed_success = executed_log['success']

                match = (planned_skill == executed_skill and
                        planned_target == executed_target and
                        executed_success)

                status = "✅" if match else "❌"
                print(f"  {status} 步骤 {i}:")
                print(f"      计划: {planned_skill}(target={planned_target})")
                print(f"      执行: {executed_skill}(target={executed_target}) - {'成功' if executed_success else '失败'}")

                if not match:
                    all_matched = False

            print()

            if all_matched and execution_result['success']:
                print("🎉 完美匹配！UI输入与实际执行完全一致\n")
                result["match_status"] = "perfect"
                return True
            elif execution_result['success']:
                print("✅ 执行成功，但有部分差异\n")
                result["match_status"] = "partial"
                return True
            else:
                print("❌ 执行失败\n")
                result["match_status"] = "failed"
                return False

        except Exception as e:
            print(f"❌ 执行错误: {e}\n")
            result["execution_error"] = str(e)
            self.test_results.append(result)
            return False

        finally:
            self.test_results.append(result)

    async def run_all_tests(self):
        """运行所有测试用例"""

        # 测试1: 简单移动命令
        await self.test_command(
            test_name="测试1: 移动到HOME",
            user_input="Go to home position",
            expected_skills=["moveTo"]  # 如果LLM正常工作
        )

        await asyncio.sleep(2)

        # 测试2: 抓取命令
        await self.test_command(
            test_name="测试2: 抓取螺栓",
            user_input="Pick up the bolt",
            expected_skills=["grasp"]
        )

        await asyncio.sleep(2)

        # 测试3: 完整拆解流程
        await self.test_command(
            test_name="测试3: 拆卸并放置螺栓",
            user_input="Remove the bolt and place it in the tray",
            expected_skills=["grasp", "release"]  # 或者 ["grasp", "release", "moveTo"]
        )

        await asyncio.sleep(2)

        # 测试4: 观察位置（测试LLM是否能正确映射到moveTo）
        await self.test_command(
            test_name="测试4: 移动到观察位置",
            user_input="Move to observation position",
            expected_skills=["moveTo"]
        )

        # 打印总结
        self.print_summary()

    def print_summary(self):
        """打印测试总结"""
        print("=" * 70)
        print("📊 测试总结")
        print("=" * 70)

        total = len(self.test_results)
        perfect = sum(1 for r in self.test_results if r.get("match_status") == "perfect")
        partial = sum(1 for r in self.test_results if r.get("match_status") == "partial")
        failed = sum(1 for r in self.test_results if r.get("match_status") == "failed")
        errors = sum(1 for r in self.test_results if "error" in r or "execution_error" in r)

        print(f"\n总测试数: {total}")
        print(f"完美匹配: {perfect} ✅")
        print(f"部分匹配: {partial} ⚠️")
        print(f"执行失败: {failed} ❌")
        print(f"系统错误: {errors} 🔥")

        if perfect == total:
            print("\n🎉 所有测试完美通过！UI输入与执行完全匹配！")
        elif perfect + partial == total:
            print("\n✅ 所有测试执行成功（有部分差异）")
        else:
            print("\n⚠️  部分测试失败，需要检查")

        # 保存详细结果
        result_file = "/tmp/ui_command_matching_results.json"
        with open(result_file, 'w', encoding='utf-8') as f:
            json.dump(self.test_results, f, indent=2, ensure_ascii=False)
        print(f"\n📄 详细结果已保存到: {result_file}")
        print()


async def main():
    """主函数"""
    tester = UICommandMatchingTest()
    await tester.run_all_tests()


if __name__ == "__main__":
    asyncio.run(main())
