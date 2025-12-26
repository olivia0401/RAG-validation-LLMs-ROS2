#!/usr/bin/python3
"""Main skill server node for robot control"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import json
import time
import threading

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from control_msgs.action import FollowJointTrajectory

# MoveItPy for planning scene manipulation
try:
    from moveit.planning import MoveItPy
    MOVEITPY_AVAILABLE = True
except ImportError:
    MOVEITPY_AVAILABLE = False

from .session import Session
from .motion_executor import MotionExecutor
from .scene_manager import SceneManager
from .skill_handlers import SkillHandlers

EXECUTION_LOCK = threading.Lock()


class SkillServer(Node):
    """Main skill server node"""

    def __init__(self):
        super().__init__("skill_server")
        self._setup_parameters()
        self._init_state()

    def _setup_parameters(self):
        """Declare and read ROS parameters"""
        self.declare_parameter("waypoints_path", "")
        self.declare_parameter("manipulator_group", "manipulator")
        self.declare_parameter("gripper_group", "gripper")
        self.declare_parameter("ee_attach_link", "robotiq_85_base_link")
        self.declare_parameter("vel_scale", 0.8)
        self.declare_parameter("acc_scale", 0.8)
        self.declare_parameter("scene_update_wait_s", 1.0)
        self.declare_parameter("open_gripper_pose_name", "OPEN")
        self.declare_parameter("close_gripper_pose_name", "CLOSE")

        self.waypoints_path = self.get_parameter("waypoints_path").get_parameter_value().string_value
        self.manipulator_group_name = self.get_parameter("manipulator_group").get_parameter_value().string_value
        self.gripper_group_name = self.get_parameter("gripper_group").get_parameter_value().string_value
        self.ee_attach_link = self.get_parameter("ee_attach_link").get_parameter_value().string_value
        self.vel_scale = self.get_parameter("vel_scale").get_parameter_value().double_value
        self.acc_scale = self.get_parameter("acc_scale").get_parameter_value().double_value
        self.open_gripper_pose = self.get_parameter("open_gripper_pose_name").get_parameter_value().string_value
        self.close_gripper_pose = self.get_parameter("close_gripper_pose_name").get_parameter_value().string_value

    def _init_state(self):
        """Initialize internal state"""
        self.waypoints_json_ = {}
        self.session_ = Session()
        self.last_joint_state_ = None
        self.motion_executor = None
        self.scene_manager = None
        self.skill_handlers = None
        self.moveit_py = None  # MoveItPy instance for planning scene access

    def init_ros(self):
        """Initialize ROS2 clients and load waypoints"""
        if not self.waypoints_path:
            self.get_logger().error("Parameter 'waypoints_path' is required!")
            raise RuntimeError("Missing waypoints_path parameter")

        self._init_action_clients()
        self._load_waypoints(self.waypoints_path)
        self._init_modules()
        self._init_topics()
        self._setup_collision_matrix()  # Disable problematic self-collisions

        self.get_logger().info(f"✅ Skill Server Ready! Listening on /llm_commands")

    def _init_action_clients(self):
        """Initialize action clients"""
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_callback_group = ReentrantCallbackGroup()

        self._move_group_action_client = ActionClient(
            self, MoveGroup, "/move_action",
            callback_group=self._action_callback_group
        )
        if not self._move_group_action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveGroup action server not available!")
        self.get_logger().info("✅ MoveGroup action server is available.")

        self._manipulator_controller_client = ActionClient(
            self, FollowJointTrajectory,
            "/fake_manipulator_controller/follow_joint_trajectory",
            callback_group=self._action_callback_group
        )
        self._gripper_controller_client = ActionClient(
            self, FollowJointTrajectory,
            "/fake_gripper_controller/follow_joint_trajectory",
            callback_group=self._action_callback_group
        )
        self.get_logger().info("✅ Controller action clients created.")

        self._planning_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        self.get_logger().info("✅ Planning scene client created.")

    def _init_modules(self):
        """Initialize executor modules"""
        self.motion_executor = MotionExecutor(
            self,
            self._move_group_action_client,
            self._manipulator_controller_client,
            self._gripper_controller_client
        )
        self.motion_executor.manipulator_group_name = self.manipulator_group_name
        self.motion_executor.vel_scale = self.vel_scale
        self.motion_executor.acc_scale = self.acc_scale

        self.scene_manager = SceneManager(self, self._planning_scene_client)
        self.skill_handlers = SkillHandlers(
            self, self.waypoints_json_, self.motion_executor, self.scene_manager
        )
        self.skill_handlers.open_gripper_pose = self.open_gripper_pose
        self.skill_handlers.close_gripper_pose = self.close_gripper_pose

    def _init_topics(self):
        """Initialize publishers and subscribers"""
        self.command_sub_ = self.create_subscription(
            String, "/llm_commands", self.command_callback, 10
        )
        self.feedback_pub_ = self.create_publisher(String, "/llm_feedback", 10)
        self.joint_state_sub_ = self.create_subscription(
            JointState, "/joint_states", self._joint_state_callback, 10
        )
        self.display_trajectory_sub_ = self.create_subscription(
            DisplayTrajectory, "/display_planned_path",
            self._display_trajectory_callback, 10
        )

    def _setup_collision_matrix(self):
        """Disable self-collision between gripper base and wrist links using MoveItPy (Method 7)"""
        self.get_logger().info("🔧 Method 7: Setting up ACM using MoveItPy planning scene read_write...")

        if not MOVEITPY_AVAILABLE:
            self.get_logger().warn("⚠️ MoveItPy not available, falling back to service-based method...")
            self._setup_collision_matrix_fallback()
            return

        try:
            # Initialize MoveItPy
            self.get_logger().info("Initializing MoveItPy...")
            self.moveit_py = MoveItPy(node_name="moveit_py_planning_scene")

            # Get planning scene monitor
            planning_scene_monitor = self.moveit_py.get_planning_scene_monitor()

            # Use read_write context to modify ACM
            self.get_logger().info("Acquiring planning scene write lock...")
            with planning_scene_monitor.read_write() as scene:
                self.get_logger().info("Modifying AllowedCollisionMatrix...")

                # Get the ACM
                acm = scene.allowed_collision_matrix

                # Set entries to allow collisions between gripper base and wrist links
                acm.set_entry("robotiq_85_base_link", "spherical_wrist_1_link", True)
                acm.set_entry("robotiq_85_base_link", "spherical_wrist_2_link", True)

                # Update the scene state
                scene.current_state.update()

                self.get_logger().info("✅ ACM modified: allowed collisions between:")
                self.get_logger().info("   - robotiq_85_base_link <-> spherical_wrist_1_link")
                self.get_logger().info("   - robotiq_85_base_link <-> spherical_wrist_2_link")

            self.get_logger().info("✅ Method 7 ACM setup completed successfully!")

        except Exception as e:
            self.get_logger().error(f"❌ Method 7 failed: {e}")
            self.get_logger().warn("Falling back to service-based method...")
            self._setup_collision_matrix_fallback()

    def _setup_collision_matrix_fallback(self):
        """Fallback: Use ApplyPlanningScene service to modify ACM"""
        self.get_logger().info("Using ApplyPlanningScene service fallback...")

        # Create planning scene client
        planning_scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        if not planning_scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("ApplyPlanningScene service not available, skipping ACM setup")
            return

        # Create planning scene with ACM modifications
        scene = PlanningScene()
        scene.is_diff = True  # Apply as diff to existing scene

        # Set allowed collisions for gripper base and wrist links
        acm = AllowedCollisionMatrix()
        acm.entry_names = ['robotiq_85_base_link', 'spherical_wrist_1_link', 'spherical_wrist_2_link']

        # Create entries: allow collision between base_link and both wrist links
        entry1 = AllowedCollisionEntry()
        entry1.enabled = [False, True, True]  # base_link with [self, wrist1, wrist2]

        entry2 = AllowedCollisionEntry()
        entry2.enabled = [True, False, False]  # wrist1 with [base_link, self, wrist2]

        entry3 = AllowedCollisionEntry()
        entry3.enabled = [True, False, False]  # wrist2 with [base_link, wrist1, self]

        acm.entry_values = [entry1, entry2, entry3]
        scene.allowed_collision_matrix = acm

        # Apply the scene
        request = ApplyPlanningScene.Request()
        request.scene = scene

        future = planning_scene_client.call_async(request)

        # Wait for result without spinning (avoid thread conflict)
        import time
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > 2.0:
                self.get_logger().warn("⏱️ Timeout waiting for ACM update")
                return
            time.sleep(0.01)

        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info("✅ Fallback ACM updated: disabled gripper-wrist collisions")
            else:
                self.get_logger().warn("❌ Failed to update ACM via fallback method")
        except Exception as e:
            self.get_logger().warn(f"❌ ACM update exception: {e}")

    def _load_waypoints(self, path):
        """Load waypoints from JSON file"""
        self.get_logger().info(f"Loading waypoints from: {path}")
        try:
            with open(path, 'r') as f:
                self.waypoints_json_ = json.load(f)
            if "poses" not in self.waypoints_json_:
                raise RuntimeError("Waypoints JSON missing 'poses' object")
            self.get_logger().info("✅ Waypoints loaded successfully")
        except Exception as e:
            raise RuntimeError(f"Failed to load waypoints: {e}")

    def _joint_state_callback(self, msg):
        """Store latest joint state"""
        self.last_joint_state_ = msg

    def _display_trajectory_callback(self, msg):
        """Log RViz planned trajectories"""
        try:
            if not msg.trajectory or len(msg.trajectory) == 0:
                return
            traj = msg.trajectory[0].joint_trajectory
            if not traj.points or len(traj.points) == 0:
                return

            final_point = traj.points[-1]
            target_positions = list(final_point.positions[:7])
            duration = final_point.time_from_start.sec + final_point.time_from_start.nanosec / 1e9

            self.get_logger().info(f"🎯 RViz planning detected! Duration: {duration:.2f}s")
        except Exception as e:
            self.get_logger().error(f"Error processing RViz trajectory: {e}")

    def publish_feedback(self, status, message, command_id="", stage="", code=0):
        """Publish feedback to LLM"""
        feedback = {
            "schema": "llm_fb/v1",
            "status": status,
            "message": message,
            "stage": stage,
            "code": code,
            "timestamp_ns": self.get_clock().now().nanoseconds,
        }
        if command_id:
            feedback["command_id"] = command_id

        msg = String()
        msg.data = json.dumps(feedback)
        self.feedback_pub_.publish(msg)

        feedback_msg = f"📤 Feedback({stage}/{status}): {message}"
        if status in ["failure", "rejected"]:
            self.get_logger().error(feedback_msg)
        else:
            self.get_logger().info(feedback_msg)

    def command_callback(self, msg):
        """Handle incoming skill commands"""
        if not EXECUTION_LOCK.acquire(blocking=False):
            self.get_logger().warn("Rejecting command: another skill in progress.")
            self.publish_feedback("rejected", "Skill execution in progress.")
            return

        try:
            self.get_logger().info(f"📨 Received command: {msg.data}")
            cmd = json.loads(msg.data)
            command_id = cmd.get("command_id", "")

            if cmd.get("schema") != "llm_cmd/v1":
                self.publish_feedback("rejected", "Unsupported schema", command_id)
                return

            skill = cmd.get("skill")
            if not skill:
                self.publish_feedback("rejected", "Missing 'skill'", command_id)
                return

            self._update_session_context(cmd)
            ok = self.dispatch_skill(skill, cmd, command_id)

            self.publish_feedback(
                "success" if ok else "failure",
                f"Skill '{skill}' {'completed' if ok else 'failed'}",
                command_id
            )
        except Exception as e:
            import traceback
            self.get_logger().error(f"FATAL ERROR: {e}")
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            self.publish_feedback("failure", f"Unexpected error: {e}")
        finally:
            EXECUTION_LOCK.release()

    def _update_session_context(self, cmd):
        """Update session context from command"""
        if "context" in cmd and isinstance(cmd["context"], dict):
            ctx = cmd["context"]
            if "object" in ctx:
                self.session_.object = ctx["object"]
            if "plan_only" in ctx:
                self.session_.plan_only = ctx["plan_only"]
            if "vel_scale" in ctx:
                self.session_.vel_scale = ctx["vel_scale"]
            if "acc_scale" in ctx:
                self.session_.acc_scale = ctx["acc_scale"]
            if "place" in ctx and isinstance(ctx["place"], dict):
                self.session_.place = ctx["place"]

    def dispatch_skill(self, skill, cmd, command_id):
        """Dispatch skill to appropriate handler"""
        try:
            # Atomic skills
            if skill == "moveTo":
                return self._handle_move_to(cmd, command_id)
            elif skill == "selectObject":
                return self._handle_select_object(cmd, command_id)
            elif skill == "selectPlace":
                return self._handle_select_place(cmd, command_id)
            elif skill == "openGripper":
                return self._handle_open_gripper(cmd)
            elif skill == "closeGripper":
                return self._handle_close_gripper(cmd)
            elif skill == "approach":
                return self._handle_approach(cmd, command_id)
            elif skill == "place":
                return self._handle_place(cmd, command_id)
            elif skill == "retreat":
                return self._handle_retreat(cmd, command_id)
            # High-level skills
            elif skill == "grasp":
                return self._handle_grasp(cmd, command_id)
            elif skill == "release":
                return self._handle_release(cmd, command_id)
            elif skill == "dismantle":
                return self._handle_dismantle(cmd, command_id)
            elif skill == "sequence":
                return self._handle_sequence(cmd, command_id)
            else:
                self.publish_feedback("rejected", f"Unknown skill: {skill}", command_id, "dispatch")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception in dispatch_skill ({skill}): {e}", exc_info=True)
            self.publish_feedback("failure", f"Exception: {e}", command_id, skill)
            return False

    def _get_pose_joints(self, name):
        """Get joint positions for a named pose"""
        return self.waypoints_json_.get("poses", {}).get(name)

    def _handle_move_to(self, cmd, command_id):
        """Handle moveTo skill"""
        target = cmd.get("target")
        if not target:
            self.publish_feedback("rejected", "moveTo missing 'target'", command_id, "moveTo")
            return False
        joints = self._get_pose_joints(target)
        if not joints:
            return False
        return self.motion_executor.plan_execute_arm(joints, "moveTo")

    def _handle_select_object(self, cmd, command_id):
        """Handle selectObject skill"""
        obj = cmd.get("params", {}).get("object_id")
        if not obj:
            self.publish_feedback("rejected", "selectObject requires params.object_id", command_id, "selectObject")
            return False
        self.session_.object = obj
        self.publish_feedback("progress", f"current object = {obj}", command_id, "selectObject")
        return True

    def _handle_select_place(self, cmd, command_id):
        """Handle selectPlace skill"""
        place_params = cmd.get("params", {})
        if not any(k in place_params for k in ["bin", "pose_name", "joints"]):
            self.publish_feedback("rejected", "selectPlace requires bin/pose_name/joints", command_id, "selectPlace")
            return False
        self.session_.place = place_params
        self.publish_feedback("progress", "place selected", command_id, "selectPlace")
        return True

    def _handle_open_gripper(self, cmd):
        """Handle openGripper skill"""
        pose_name = cmd.get("params", {}).get("pose_name", self.open_gripper_pose)
        joints = self._get_pose_joints(pose_name)
        if not joints:
            return False
        return self.motion_executor.plan_execute_gripper(joints, "openGripper")

    def _handle_close_gripper(self, cmd):
        """Handle closeGripper skill"""
        pose_name = cmd.get("params", {}).get("pose_name", self.close_gripper_pose)
        joints = self._get_pose_joints(pose_name)
        if not joints:
            return False
        return self.motion_executor.plan_execute_gripper(joints, "closeGripper")

    def _handle_approach(self, cmd, command_id):
        """Handle approach skill"""
        obj = self._extract_object_from_cmd(cmd) or self.session_.object
        if obj not in self.waypoints_json_.get("objects", {}):
            self.publish_feedback("rejected", f"approach: unknown object '{obj}'", command_id, "approach")
            return False
        joints = self.waypoints_json_["objects"][obj].get("approach")
        if not joints:
            return False
        return self.motion_executor.plan_execute_arm(joints, "approach")

    def _handle_place(self, cmd, command_id):
        """Handle place skill"""
        obj = self._extract_object_from_cmd(cmd) or self.session_.object
        local_place = cmd.get("params", {}) or self.session_.place
        target_joints = self.skill_handlers._resolve_place_joints(obj, local_place)
        if not target_joints:
            self.publish_feedback("failure", "place: cannot resolve joints", command_id, "place")
            return False
        return self.motion_executor.plan_execute_arm(target_joints, "place")

    def _handle_retreat(self, cmd, command_id):
        """Handle retreat skill"""
        obj = self._extract_object_from_cmd(cmd) or self.session_.object
        if obj not in self.waypoints_json_.get("objects", {}):
            self.publish_feedback("rejected", f"retreat: unknown object '{obj}'", command_id, "retreat")
            return False
        joints = self.waypoints_json_["objects"][obj].get("retreat")
        if not joints:
            return False
        return self.motion_executor.plan_execute_arm(joints, "retreat")

    def _handle_grasp(self, cmd, command_id):
        """Handle grasp skill"""
        obj = self._extract_object_from_cmd(cmd) or self.session_.object
        if not obj:
            self.publish_feedback("rejected", "grasp needs an object", command_id, "grasp")
            return False
        return self.skill_handlers.execute_grasp(obj)

    def _handle_release(self, cmd, command_id):
        """Handle release skill"""
        obj = self._extract_object_from_cmd(cmd) or self.session_.object
        if not obj:
            self.publish_feedback("rejected", "release needs an object", command_id, "release")
            return False
        place_joints = self.skill_handlers._resolve_place_joints(
            obj, cmd.get("place", {}) or self.session_.place
        )
        return self.skill_handlers.execute_release(obj, place_joints)

    def _handle_dismantle(self, cmd, command_id):
        """Handle dismantle skill"""
        targets = cmd.get("targets", [])
        if not targets:
            self.publish_feedback("rejected", "dismantle: missing targets", command_id, "dismantle")
            return False
        place_default = cmd.get("place", {})
        return self.skill_handlers.execute_dismantle(targets, place_default or self.session_.place)

    def _handle_sequence(self, cmd, command_id):
        """Handle sequence skill"""
        steps = cmd.get("params", {}).get("steps", [])
        if not steps:
            self.publish_feedback("rejected", "sequence needs params.steps", command_id, "sequence")
            return False
        for step in steps:
            sub_skill = step.get("skill")
            if not sub_skill:
                continue
            if not self.dispatch_skill(sub_skill, step, command_id):
                return False
        return True

    def _extract_object_from_cmd(self, cmd):
        """Extract object name from command"""
        if "target" in cmd and isinstance(cmd["target"], str):
            return cmd["target"]
        if "params" in cmd and isinstance(cmd["params"], dict):
            # Check for 'object', 'object_id', or 'target' in params
            return (cmd["params"].get("object") or
                    cmd["params"].get("object_id") or
                    cmd["params"].get("target") or "")
        return ""


def main(args=None):
    rclpy.init(args=args)
    skill_server_node = SkillServer()

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(skill_server_node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    time.sleep(2.0)

    try:
        skill_server_node.init_ros()
        thread.join()
    except Exception as e:
        skill_server_node.get_logger().fatal(f"Failed to initialize SkillServer: {e}")
    finally:
        executor.shutdown()
        skill_server_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
