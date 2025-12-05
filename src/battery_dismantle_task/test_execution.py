#!/usr/bin/env python3
"""
测试完整执行流程 - 从规划到执行
"""
import asyncio
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'llm_agent'))

from planner import Planner
from validator import Validator
from executor import Executor
import time

async def test_execution():
    """测试完整的执行流程"""

    print("=" * 60)
    print("🚀 完整执行流程测试")
    print("=" * 60)

    # 初始化
    print("\n📦 初始化组件...")
    try:
        executor = Executor(use_ros=True)
        planner = Planner(backend="ollama")
        validator = Validator()
        print("✅ 所有组件初始化成功")
    except Exception as e:
        print(f"❌ 初始化失败: {e}")
        return 1

    # 测试用例（从简单到复杂）
    test_cases = [
        {
            "name": "测试1: 移动到HOME位置",
            "input": "Go to home position",
            "use_demo": True  # 使用demo计划避免LLM调用
        },
        {
            "name": "测试2: 抓取螺栓（单步操作）",
            "plan": {
                "plan": [
                    {"step": 1, "name": "grasp", "params": {"target": "TopCoverBolts"}}
                ]
            }
        },
        {
            "name": "测试3: 释放螺栓",
            "plan": {
                "plan": [
                    {"step": 1, "name": "release", "params": {"target": "TopCoverBolts"}}
                ]
            }
        },
        {
            "name": "测试4: 返回HOME",
            "plan": {
                "plan": [
                    {"step": 1, "name": "moveTo", "params": {"target": "HOME"}}
                ]
            }
        },
    ]

    passed = 0
    total = len(test_cases)

    for i, test in enumerate(test_cases, 1):
        print(f"\n{'='*60}")
        print(f"{test['name']} ({i}/{total})")
        print(f"{'='*60}")

        try:
            # 获取或生成计划
            if 'plan' in test:
                plan = test['plan']
                print(f"📋 使用预定义计划 ({len(plan['plan'])} 步)")
            else:
                print(f"📝 输入: \"{test['input']}\"")
                print("🤖 生成计划...")
                plan = await planner.plan(test['input'], use_llm=not test.get('use_demo', False))

            # 显示计划
            print("\n📄 执行计划:")
            for step in plan['plan']:
                print(f"  步骤 {step['step']}: {step['name']}({step['params']})")

            # 验证
            print("\n🔍 验证计划...")
            is_valid, errors = validator.validate_plan(plan)
            if not is_valid:
                print(f"❌ 验证失败: {', '.join(errors)}")
                continue

            print("✅ 验证通过")

            # 执行
            print("\n🚀 开始执行...")
            print("-" * 60)
            results = executor.execute(plan, timeout=30.0)
            print("-" * 60)

            # 结果
            if results['success']:
                print(f"\n✅ 执行成功!")
                print(f"   成功步数: {results['executed']}")
                print(f"   失败步数: {results['failed']}")
                passed += 1
            else:
                print(f"\n❌ 执行失败")
                print(f"   成功步数: {results['executed']}")
                print(f"   失败步数: {results['failed']}")

            # 短暂延迟
            time.sleep(1)

        except Exception as e:
            print(f"\n❌ 测试异常: {str(e)}")
            import traceback
            traceback.print_exc()

    # 总结
    print(f"\n{'='*60}")
    print("📊 执行测试总结")
    print(f"{'='*60}")
    print(f"通过: {passed}/{total}")
    print(f"成功率: {passed/total*100:.1f}%")

    if passed == total:
        print("\n🎉 所有执行测试通过！系统工作正常。")
        return 0
    else:
        print(f"\n⚠️  {total - passed} 个测试失败")
        return 1

if __name__ == "__main__":
    exit_code = asyncio.run(test_execution())
    sys.exit(exit_code)
