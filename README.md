# RAG-Enhanced LLM Control for ROS2 Battery Disassembly

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A research project investigating **Retrieval-Augmented Generation (RAG)** for improving Large Language Model (LLM) performance in robotic manipulation tasks. This system enables natural language control of a Kinova Gen3 robot arm for battery disassembly operations.

> 🔬 **Research Focus**: Validating whether RAG-based experience retrieval can enhance LLM planning accuracy and reduce hallucinations in robotic control scenarios.

---

## 🎥 Demo

[![Watch the demo](https://img.youtube.com/vi/vp2pIpejiL0/maxresdefault.jpg)](https://youtu.be/vp2pIpejiL0)

**Example Commands:**
- "Grasp the battery cover bolts"
- "Move to the placement area"
- "Return to home position"

---

## 📚 Table of Contents

- [Research Motivation](#-research-motivation)
- [System Architecture](#-system-architecture)
- [Key Features](#-key-features)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Usage](#-usage)
- [Project Structure](#-project-structure)
- [Technical Details](#-technical-details)
- [Performance Metrics](#-performance-metrics)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🔬 Research Motivation

### The Problem
Large Language Models (LLMs) can hallucinate when generating robot action plans, producing:
- Non-existent skill names
- Invalid parameter values
- Unsafe motion sequences

### Our Approach
**Retrieval-Augmented Generation (RAG)** to:
1. Store successful execution cases in a vector database (ChromaDB)
2. Retrieve similar historical cases for new commands
3. Provide LLM with concrete examples to reduce hallucinations

### Research Questions
- Does RAG improve LLM planning accuracy for robotic tasks?
- What is the optimal knowledge base size?
- How does RAG affect execution success rate?

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    User Interface                            │
│              (Web UI + Natural Language Input)              │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  LLM Planning Pipeline                       │
│  ┌────────────┐  ┌────────────┐  ┌────────────┐            │
│  │ RAG Engine │→ │ LLM Planner│→ │ Validator  │            │
│  │ (ChromaDB) │  │ (GPT/Local)│  │ (Schema)   │            │
│  └────────────┘  └────────────┘  └────────────┘            │
└─────────────────────────────────────────────────────────────┘
                              │
                    ROS2 Action Interface
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│            ROS2 Skill Execution Layer                        │
│         (Python Skill Server + MoveIt2)                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│         Kinova Gen3 Robot (Simulation/Hardware)              │
│              + Robotiq 2F-85 Gripper                         │
└─────────────────────────────────────────────────────────────┘
```

**Two Main Components:**
1. **`llm_agent/`** - Python LLM pipeline with RAG integration
2. **`battery_dismantle_task/`** - ROS2 package for robot control

---

## ✨ Key Features

### RAG System
- ✅ **Semantic Search**: ChromaDB with sentence-transformers embeddings
- ✅ **Experience Storage**: Successful execution cases automatically stored
- ✅ **Retrieval**: Top-k similar cases retrieved for each new task
- ✅ **Experiment Control**: Adjustable memory size for ablation studies

### Robot Control
- ✅ **Natural Language Commands**: "Grasp the battery", "Move to home position"
- ✅ **MoveIt2 Integration**: Advanced motion planning and collision avoidance
- ✅ **Dual-Layer Validation**: Schema validation + runtime safety checks
- ✅ **Real-time Visualization**: RViz integration for motion feedback

### Web Interface
- ✅ **Browser-based Control**: Gradio UI at `http://localhost:7862`
- ✅ **Execution Logs**: Real-time feedback and debugging
- ✅ **Multi-LLM Support**: OpenAI GPT-4, GPT-3.5, or local Ollama

---

## 🚀 Installation

### Prerequisites

- **OS**: Ubuntu 22.04
- **ROS2**: Humble (full installation)
- **Python**: 3.10+
- **Hardware**: Optional - Kinova Gen3 arm (works in simulation)

### Step 1: Clone Repository

```bash
git clone https://github.com/olivia0401/RAG-validation-LLMs-ROS2.git
cd RAG-validation-LLMs-ROS2
```

### Step 2: Install ROS2 Dependencies

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build workspace
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

### Step 3: Install Python Dependencies

```bash
cd src/llm_agent
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Step 4: Configure LLM API (Optional)

```bash
# For OpenAI GPT
export OPENAI_API_KEY="your-api-key-here"

# OR use local Ollama (free)
# Install Ollama from https://ollama.ai
ollama pull mistral
```

---

## 🎮 Quick Start

### Launch Complete System

```bash
# Terminal 1: Start robot simulation + skill server
source install/setup.bash
bash FINAL_START.sh
```

This will:
- ✅ Launch ROS2 control nodes
- ✅ Start MoveIt2 motion planning
- ✅ Initialize RViz visualization
- ✅ Start Web UI at http://localhost:7862

### Test via Web UI

1. Open browser: http://localhost:7862
2. Click **"Initialize System"**
3. Enter command: `"Grasp the top cover bolts"`
4. Click **"Execute"**
5. Watch robot move in RViz!

---

## 📖 Usage

### Available Commands

#### Basic Navigation
```
"Go to home position"
"Move to the placement area"
"Return to home"
```

#### Object Manipulation
```
"Grasp the top cover bolts"
"Pick up the battery box"
"Release the battery"
```

#### Complete Tasks
```
"Remove the battery cover bolts"  # grasp → move → release
"Disassemble the battery"          # multi-step sequence
```

### Available Skills

| Skill | Description | Parameters |
|-------|-------------|------------|
| `moveTo` | Move arm to named pose | `target`: HOME, place_bolts, approach_bolts |
| `grasp` | Pick up object | `target`: TopCoverBolts, BatteryBox_0 |
| `release` | Release object | `target`: object name |
| `openGripper` | Open gripper | None |
| `closeGripper` | Close gripper | None |

### Direct ROS2 Control (Advanced)

```bash
# Send command via ROS2 topic
ros2 topic pub --once /llm_commands std_msgs/msg/String \
  "{data: '{\"skill\": \"grasp\", \"target\": \"TopCoverBolts\"}'}"

# Monitor feedback
ros2 topic echo /llm_feedback
```

---

## 📂 Project Structure

```
RAG-validation-LLMs-ROS2/
├── src/
│   ├── llm_agent/                    # LLM planning pipeline
│   │   ├── planner.py                # Task planning with RAG
│   │   ├── rag_engine.py             # ChromaDB retrieval engine
│   │   ├── executor.py               # ROS2 skill execution
│   │   ├── validator.py              # Safety validation
│   │   ├── web_ui.py                 # Gradio interface
│   │   ├── topics.py                 # ROS2 topic names (config)
│   │   └── config/
│   │       ├── skills.json           # Available robot skills
│   │       ├── prompt.txt            # LLM system prompt
│   │       └── safety.yaml           # Safety constraints
│   │
│   └── battery_dismantle_task/       # ROS2 robot control
│       ├── battery_dismantle_task/
│       │   ├── skill_server.py       # Main ROS2 node
│       │   ├── motion_executor.py    # MoveIt2 execution
│       │   ├── scene_manager.py      # Collision object management
│       │   ├── skill_handlers.py     # High-level skill implementations
│       │   ├── motion_config.py      # Motion parameters (config)
│       │   └── object_definitions.py # Scene object definitions (config)
│       ├── config/
│       │   ├── waypoints.json        # Named joint positions
│       │   └── moveit/               # MoveIt2 configuration
│       └── launch/
│           └── fake_execution_complete.launch.py
│
├── FINAL_START.sh                    # Complete system launcher
├── README.md                         # This file
└── PROJECT_STRUCTURE.md              # Detailed structure documentation
```

See [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md) for detailed architecture documentation.

---

## 🔧 Technical Details

### RAG Implementation

**Vector Database**: ChromaDB with cosine similarity
**Embeddings**: `sentence-transformers/all-MiniLM-L6-v2`
**Retrieval**: Top-3 similar cases by default

```python
# Example stored case
{
  "task": "remove the battery bolts",
  "plan": [
    {"skill": "grasp", "target": "TopCoverBolts"},
    {"skill": "moveTo", "target": "place_bolts"},
    {"skill": "release", "target": "TopCoverBolts"}
  ],
  "execution_time": 12.5,
  "success": true
}
```

### Motion Planning

**Library**: MoveIt2 with OMPL planners
**Velocity Scaling**: 0.8x (configurable in `motion_config.py`)
**Collision Checking**: Automatic via planning scene
**Execution Mode**: Direct trajectory control (bypasses MoveIt execution for speed)

### Safety Validation

1. **Schema Validation**: Check skill names and parameter types
2. **Semantic Validation**: Verify object names exist
3. **Runtime Checks**: Joint limits, collision detection

---

## 📊 Performance Metrics

### Current Results (Preliminary)

| Metric | Value | Notes |
|--------|-------|-------|
| Task Completion Rate | 100% | With correct ROS2 commands |
| LLM Understanding Accuracy | ~85% | Natural language → skill plan |
| Average Planning Time | 1.8s | Including LLM + RAG retrieval |
| Average Execution Time | 3.8s | Per moveTo action |
| RAG Retrieval Time | <200ms | Top-3 cases from 35+ stored |

### Known Limitations

- ⚠️ LLM may occasionally misinterpret ambiguous commands
- ⚠️ RAG effectiveness depends on knowledge base quality
- ✅ Direct ROS2 commands have 100% reliability

---

## 🛠️ Tech Stack

**Robotics:**
- ROS2 Humble
- MoveIt2 (motion planning)
- RViz2 (visualization)
- Kinova Gen3 SDK

**AI/ML:**
- OpenAI GPT-4 / GPT-3.5 (optional)
- ChromaDB (vector database)
- SentenceTransformer (embeddings)
- Ollama (local LLM, optional)

**Web/Backend:**
- Gradio (web interface)
- Python 3.10+ (async/await)

---

## 🧪 Running Experiments

### Ablation Study: RAG vs No-RAG

```bash
# Disable RAG
cd src/llm_agent
# Edit planner.py: enable_rag=False

# Test commands and compare accuracy
```

### Memory Size Experiments

```python
# In planner.py
planner = Planner(
    enable_rag=True,
    rag_limit=10,  # Limit to 10 cases
    rag_seed=42    # Deterministic sampling
)
```

---

## 🤝 Contributing

Contributions are welcome! Areas for improvement:
- [ ] Expand skill library
- [ ] Add more complex manipulation tasks
- [ ] Improve LLM prompt engineering
- [ ] Implement reinforcement learning from feedback
- [ ] Hardware deployment documentation

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👤 Author

**Olivia Xu**
MSc Artificial Intelligence
📧 oliviaxu0401@gmail.com
🔗 [GitHub](https://github.com/olivia0401)

---

## 🙏 Acknowledgments

- ROS2 and MoveIt2 communities for excellent documentation
- Kinova Robotics for Gen3 SDK
- Open-source LLM community for making AI research accessible

---

## 📝 Citation

If you use this work in your research, please cite:

```bibtex
@software{xu2024rag_llm_ros2,
  author = {Xu, Olivia},
  title = {RAG-Enhanced LLM Control for ROS2 Battery Disassembly},
  year = {2024},
  url = {https://github.com/olivia0401/RAG-validation-LLMs-ROS2}
}
```

---

**⭐ If you find this project useful for your research, please consider starring it!**
