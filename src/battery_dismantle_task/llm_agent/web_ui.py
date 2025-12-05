#!/usr/bin/env python3
"""
Web UI for LLM-Controlled Battery Disassembly Robot
Simple and intuitive interface using Gradio
"""
import gradio as gr
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from executor import Executor
from planner import Planner
from validator import Validator
import json
import time

class RobotUI:
    def __init__(self):
        self.executor = None
        self.planner = None
        self.validator = None
        self.system_running = False

    def initialize_system(self):
        """Initialize ROS2 connection"""
        if not self.system_running:
            try:
                self.executor = Executor(use_ros=True)
                self.planner = Planner(backend="ollama")
                self.validator = Validator()
                self.system_running = True
                return "✅ System initialized successfully!"
            except Exception as e:
                return f"❌ Initialization failed: {str(e)}"
        return "⚠️  System already running"

    def execute_prompt(self, user_prompt, progress=gr.Progress()):
        """Execute user prompt"""
        if not self.system_running:
            return "❌ Please initialize system first!", "", ""

        if not user_prompt.strip():
            return "⚠️  Please enter a command", "", ""

        log = []
        log.append(f"📝 Your Command: {user_prompt}\n")

        # Step 1: Planning
        progress(0.2, desc="Planning...")
        log.append("🧠 LLM is generating plan...")
        try:
            import asyncio
            plan = asyncio.run(self.planner.plan(user_prompt))
            num_steps = len(plan.get('plan', []))
            log.append(f"✅ Generated {num_steps}-step plan\n")
        except Exception as e:
            log.append(f"❌ Planning failed: {str(e)}\n")
            return "\n".join(log), "", json.dumps({"error": str(e)}, indent=2)

        # Step 2: Validation
        progress(0.4, desc="Validating...")
        log.append("🔍 Validating safety...")
        is_valid, errors = self.validator.validate_plan(plan)
        if not is_valid:
            log.append(f"❌ Validation failed: {', '.join(errors)}\n")
            return "\n".join(log), "", json.dumps(plan, indent=2)
        log.append("✅ Safety check passed\n")

        # Step 3: Execution
        progress(0.6, desc="Executing...")
        log.append("🚀 Executing plan...\n")

        plan_text = self._format_plan(plan)

        try:
            results = self.executor.execute(plan, timeout=20.0)

            # Format results
            progress(1.0, desc="Complete!")
            log.append(f"\n📊 Execution Results:")
            log.append(f"  Total steps: {results['executed'] + results['failed']}")
            log.append(f"  Successful: {results['executed']}")
            log.append(f"  Failed: {results['failed']}")
            log.append(f"  Success rate: {results['executed']/(results['executed']+results['failed'])*100:.0f}%")

            if results['success']:
                log.append("\n🎉 Task completed successfully!")
            else:
                log.append("\n❌ Task failed")

        except Exception as e:
            log.append(f"\n❌ Execution error: {str(e)}")
            return "\n".join(log), plan_text, json.dumps(plan, indent=2)

        return "\n".join(log), plan_text, json.dumps(results, indent=2)

    def _format_plan(self, plan):
        """Format plan for display"""
        lines = ["Step-by-step Plan:", "=" * 50]
        for step in plan.get('plan', []):
            skill = step['name']
            target = step['params'].get('target', '')
            target_str = f" → {target}" if target else ""
            lines.append(f"{step['step']}. {skill}{target_str}")
        return "\n".join(lines)

    def get_robot_state(self):
        """Get current robot state"""
        if not self.system_running or not self.executor:
            return "System not initialized"

        state = self.executor.get_current_state()
        if not state:
            return "No state available"

        lines = [
            "🤖 Current Robot State",
            "=" * 50,
            f"Gripper: {state['gripper_state']}",
            "",
            "Arm Joints:"
        ]
        for joint, pos in state['arm_joints'].items():
            lines.append(f"  {joint}: {pos:.4f} rad")

        return "\n".join(lines)

# Create UI instance
robot_ui = RobotUI()

# Build Gradio Interface
with gr.Blocks(title="LLM Robot Control", theme=gr.themes.Soft()) as demo:
    gr.Markdown("""
    # 🤖 LLM-Controlled Battery Disassembly Robot

    Control the robot using natural language commands powered by AI.
    """)

    with gr.Row():
        with gr.Column(scale=2):
            # Control Panel
            gr.Markdown("## 🎮 Control Panel")

            init_btn = gr.Button("🔌 Initialize System", variant="primary", size="lg")
            init_output = gr.Textbox(label="System Status", lines=2)

            gr.Markdown("---")

            prompt_input = gr.Textbox(
                label="💬 Command",
                placeholder="Example: Remove the bolt and place it in the tray",
                lines=2
            )

            execute_btn = gr.Button("▶️ Execute", variant="primary", size="lg")

            gr.Markdown("### Quick Commands:")
            with gr.Row():
                gr.Button("🏠 Go Home", size="sm").click(
                    lambda: "Go to home position",
                    outputs=prompt_input
                )
                gr.Button("🔩 Remove Bolt", size="sm").click(
                    lambda: "Remove the bolt and place it in the tray",
                    outputs=prompt_input
                )
                gr.Button("👁️ Observe", size="sm").click(
                    lambda: "Move to observation position",
                    outputs=prompt_input
                )

        with gr.Column(scale=1):
            # Status Panel
            gr.Markdown("## 📊 Robot Status")
            state_output = gr.Textbox(label="Current State", lines=12)
            refresh_btn = gr.Button("🔄 Refresh State")

    # Output Panel
    gr.Markdown("---")
    gr.Markdown("## 📋 Execution Log")

    with gr.Row():
        log_output = gr.Textbox(label="Console Output", lines=15)
        plan_output = gr.Textbox(label="Generated Plan", lines=15)

    with gr.Accordion("📄 Detailed Results (JSON)", open=False):
        json_output = gr.Code(label="Raw Data", language="json")

    # Examples
    gr.Examples(
        examples=[
            ["Remove the bolt and place it in the tray"],
            ["Go to home position"],
            ["Move to observation position"],
            ["Pick up the bolt"],
            ["Place object in tray"],
        ],
        inputs=prompt_input,
        label="Example Commands"
    )

    # Event handlers
    init_btn.click(
        fn=robot_ui.initialize_system,
        outputs=init_output
    )

    execute_btn.click(
        fn=robot_ui.execute_prompt,
        inputs=prompt_input,
        outputs=[log_output, plan_output, json_output]
    )

    refresh_btn.click(
        fn=robot_ui.get_robot_state,
        outputs=state_output
    )

    gr.Markdown("""
    ---
    ### 📖 Instructions:
    1. Click **Initialize System** first
    2. Enter your command in natural language
    3. Click **Execute** to run
    4. Watch the robot execute in RViz window

    ### ⚙️ Available Actions:
    - **Positions**: HOME, above, approach, grasp, lift, over, place, retreat, observe
    - **Skills**: moveTo, openGripper, closeGripper
    """)

# Launch
if __name__ == "__main__":
    print("🚀 Starting Web UI...")
    print("📍 Open browser at: http://localhost:7860")
    print("🤖 Make sure ROS2 system is running!")
    print()

    import os
    port = int(os.getenv('GRADIO_SERVER_PORT', '7860'))
    demo.launch(
        server_name="0.0.0.0",
        server_port=port,
        share=False
    )
