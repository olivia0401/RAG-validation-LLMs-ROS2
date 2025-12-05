"""
统一规划逻辑 - Prompt + Parse
"""
import json
import asyncio
from pathlib import Path
from typing import Dict, List, Optional
from llm_client import LLMClient


class Planner:
    """任务规划器"""

    def __init__(self, config_dir: Optional[Path] = None, backend: str = "ollama"):
        """
        初始化规划器

        Args:
            config_dir: 配置文件目录
            backend: LLM后端 ("ollama" 或 "chutes")
        """
        if config_dir is None:
            config_dir = Path(__file__).parent / "config"

        # 加载配置
        with open(config_dir / "skills.json") as f:
            self.skills_config = json.load(f)

        with open(config_dir / "prompt.txt") as f:
            self.prompt_template = f.read()

        # 初始化LLM客户端
        self.llm = LLMClient(backend=backend)

    async def plan(self, task: str, use_llm: bool = True) -> Dict:
        """
        生成任务计划

        Args:
            task: 任务描述(自然语言)
            use_llm: 是否使用LLM(False则用demo计划)

        Returns:
            计划字典 {"plan": [...]}
        """
        # Check if LLM is available (ollama doesn't need api_key, so check backend config)
        needs_key = self.llm.config.get("requires_key", True)
        if not use_llm or (needs_key and not self.llm.api_key):
            print("📋 Using demo plan...")
            return self._demo_plan()

        # 构造prompt
        prompt = self._build_prompt(task)

        print(f"🤖 Planning: {task}")
        print("📡 Calling LLM...")

        try:
            # 调用LLM
            response = await self.llm.generate(prompt)

            # 解析JSON
            plan = self._parse_response(response)

            print("✅ Plan generated successfully")
            return plan

        except Exception as e:
            print(f"❌ LLM planning failed: {e}")
            print("🔄 Falling back to demo plan...")
            return self._demo_plan()

    def _build_prompt(self, task: str) -> str:
        """构造完整的prompt"""
        # 格式化技能列表
        skills_list = "\n".join([
            f"- {s['name']}: {s['description']}"
            for s in self.skills_config['available_skills']
        ])

        poses_list = ", ".join(self.skills_config['available_poses'])
        objects_list = ", ".join(self.skills_config['available_objects'])

        return self.prompt_template.format(
            task_description=task,
            skills_list=skills_list,
            poses_list=poses_list,
            objects_list=objects_list
        )

    def _parse_response(self, response: str) -> Dict:
        """从LLM响应中提取JSON"""
        # 找到JSON部分
        start = response.find("{")
        end = response.rfind("}") + 1

        if start < 0 or end <= start:
            raise ValueError("No valid JSON found in response")

        json_str = response[start:end]
        plan = json.loads(json_str)

        # 验证格式
        if "plan" not in plan:
            raise ValueError("Invalid plan format: missing 'plan' key")

        return plan

    def _demo_plan(self) -> Dict:
        """返回demo计划 - 完整的电池拆解流程"""
        return {
            "plan": [
                # 步骤1: 抓取顶盖螺栓（包含：打开夹爪->移动到approach->关闭夹爪->attach物体）
                {"step": 1, "name": "grasp", "params": {"target": "TopCoverBolts"}},

                # 步骤2: 释放顶盖螺栓到放置位置（包含：移动到place->打开夹爪->detach物体）
                {"step": 2, "name": "release", "params": {"target": "TopCoverBolts"}},

                # 步骤3: 移动到HOME位置观察
                {"step": 3, "name": "moveTo", "params": {"target": "HOME"}},

                # 步骤4: 抓取电池主体
                {"step": 4, "name": "grasp", "params": {"target": "BatteryBox_0"}},

                # 步骤5: 释放电池到放置位置
                {"step": 5, "name": "release", "params": {"target": "BatteryBox_0"}},

                # 步骤6: 回到HOME位置完成任务
                {"step": 6, "name": "moveTo", "params": {"target": "HOME"}}
            ]
        }

    def print_plan(self, plan: Dict):
        """打印计划"""
        print("\n✅ Generated Plan:")
        for action in plan['plan']:
            print(f"  Step {action['step']}: {action['name']}({action['params']})")


# 测试
async def test():
    planner = Planner()
    plan = await planner.plan("disassemble the battery", use_llm=False)
    planner.print_plan(plan)


if __name__ == "__main__":
    asyncio.run(test())
