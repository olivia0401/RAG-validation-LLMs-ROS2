# Battery Dismantle Task

LLM-controlled battery disassembly task using **Kinova Gen3 7-DOF robot arm** with **Robotiq 2F-85 gripper** and **MoveIt Task Constructor (MTC)** framework.

## Overview

This ROS 2 package implements a hierarchical task planning approach for robotic battery disassembly. It uses MoveIt Task Constructor to execute waypoint-based motion sequences (approach, grasp, place, retreat) for various battery components.

**Key Features:**
- Data-driven waypoint configuration via JSON
- MTC-based hierarchical task planning
- Collision-aware motion planning with MoveIt 2
- Support for simulation and real hardware
- Modular design for future LLM integration

## Package Structure

```
battery_dismantle_task/
├── config/               # Configuration files
│   ├── waypoints.json    # Primary waypoint definitions (poses for arm & gripper)
│   ├── gen3_robotiq_2f_85.srdf  # Robot semantic description
│   └── *.yaml            # MoveIt parameters (controllers, planners, kinematics)
├── include/              # C++ header files
│   └── battery_dismantle_task/
│       ├── DismantleTask.h    # Main task class
│       └── task_utils.h       # Waypoint data structures & JSON parser
├── src/                  # C++ source files
│   ├── dismantle_node.cpp     # ROS 2 entry point
│   └── DismantleTask.cpp      # MTC task implementation
├── launch/               # Launch files
│   └── minimal_planning_fixed.launch.py  # Main launch file
├── scripts/              # Python utilities
│   ├── publish_scene.py       # Publish planning scene objects
│   └── clock_publisher.py     # Sim time publisher
├── test/                 # Test files (development only)
│   ├── simple_motion_test.cpp
│   └── test_*.launch.py
└── CMakeLists.txt
```

## Dependencies

### ROS 2 Packages
- `rclcpp` - ROS 2 C++ client library
- `moveit_ros_planning_interface` - MoveIt 2 motion planning
- `moveit_task_constructor_core` - Hierarchical task planning framework
- `ament_index_cpp` - Package resource indexing
- `nlohmann_json` - JSON parsing for waypoints
- `tf2_geometry_msgs` - Transform utilities

### External Dependencies
- ROS 2 Humble
- MoveIt 2
- Kinova Gen3 driver (`ros2_kortex`)
- Robotiq gripper driver (`ros2_robotiq_gripper`)

## Building

```bash
# Navigate to workspace
cd ~/llms-ros2

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build this package
colcon build --packages-select battery_dismantle_task --symlink-install

# Source the workspace
source install/setup.bash
```

**Clean build (if needed):**
```bash
colcon build --packages-select battery_dismantle_task --symlink-install --cmake-clean-first
```

## Usage

### Running the Main Task

Launch RViz visualization with MoveIt and the dismantle task:

```bash
ros2 launch battery_dismantle_task minimal_planning_fixed.launch.py
```

**With Isaac Sim integration:**
```bash
ros2 launch battery_dismantle_task minimal_planning_fixed.launch.py use_isaac_bridge:=true
```

### Running Standalone Node

If `move_group` is already running separately:

```bash
ros2 run battery_dismantle_task dismantle_node
```

## Configuration

### Waypoints Configuration

Edit `config/waypoints.json` to define joint poses:

```json
{
  "joints": {
    "arm": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"],
    "gripper": ["robotiq_85_left_knuckle_joint"]
  },
  "poses": {
    "HOME": [0, -1.57, 1.57, 0, 1.57, 0, 0],
    "OPEN": [0],
    "CLOSE": [0.8]
  },
  "sequences": {
    "TopCoverBolts": {
      "approach": "APPROACH_BOLT",
      "grasp": "CLOSE",
      "place": "PLACE_BOLT",
      "retreat": "HOME"
    }
  }
}
```

**Joint Value Ranges:**
- Arm joints: radians (typically -π to π)
- Gripper: 0.0 (fully open) to 0.8 (closed)

### Planning Groups

- `manipulator` - Kinova Gen3 arm (7 DOF)
- `gripper` - Robotiq 2F-85 gripper

## Architecture

### MTC Task Pipeline

The dismantle task follows this stage sequence for each component:

1. **Current State** - Capture initial robot state
2. **Approach** - Move arm to pre-grasp pose
3. **Grasp** - Close gripper on component
4. **Place** - Move to placement location
5. **Retreat** - Return to home pose

Each stage uses `JointInterpolationPlanner` for collision-free motion.

### Planning Scene

The battery and its subcomponents are spawned as collision objects:
- Battery body (base structure)
- Top cover bolts
- Battery cells
- Bottom plate

Objects are defined in `DismantleTask::spawnBatteryAndSubparts()`.

## Hardware vs Simulation

Launch files support mode selection via XACRO parameters:

- `use_fake_hardware: true` - Simulation mode (default)
- `use_fake_hardware: false` - Real Kinova Gen3 hardware
- `sim_ignition: true` - Ignition Gazebo simulation
- `use_isaac_bridge: true` - NVIDIA Isaac Sim integration

## Future Work

### LLM Integration (Planned)

1. LLM receives high-level task: "disassemble the battery"
2. LLM decomposes into subtasks via `robot_skills` services
3. Services trigger MTC execution in this package
4. Robot state feedback flows back to LLM for replanning

Target packages: `llm_bridge`, `robot_skills` (in `ws_moveit2` workspace)

## Troubleshooting

### Controllers Not Ready
If execution fails with "action server not available":
- Increase the 5-second wait in `dismantle_node.cpp:52`
- Check controller status: `ros2 control list_controllers`

### Planning Failures
- Verify waypoints are reachable: check joint limits in `config/*.yaml`
- Inspect RViz Motion Planning panel for collision warnings
- Review MTC task introspection in RViz "Motion Planning Tasks" panel

### Build Errors
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Check C++17 support: minimum GCC 7.0 or Clang 5.0

## Testing

```bash
# Run simple motion test
ros2 run battery_dismantle_task simple_motion_test

# Launch test files (in test/ directory)
ros2 launch battery_dismantle_task test/test_srdf.launch.py
```

## License

Apache 2.0

## Maintainer

Olivia (olivia@example.com)

## References

- [MoveIt 2 Documentation](https://moveit.picknik.ai/humble/index.html)
- [MoveIt Task Constructor Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/moveit_task_constructor/moveit_task_constructor_tutorial.html)
- [Kinova Gen3 ROS 2 Driver](https://github.com/Kinovarobotics/ros2_kortex)
- [Robotiq Gripper ROS 2](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
