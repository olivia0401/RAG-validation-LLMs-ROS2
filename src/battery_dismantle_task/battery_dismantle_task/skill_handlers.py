#!/usr/bin/python3
"""Skill execution handlers for robot tasks"""


class SkillHandlers:
    """Handles execution of high-level robot skills"""

    def __init__(self, node, waypoints, motion_executor, scene_manager):
        self.node = node
        self.waypoints = waypoints
        self.motion = motion_executor
        self.scene = scene_manager
        self.open_gripper_pose = "OPEN"
        self.close_gripper_pose = "CLOSE"

    def execute_grasp(self, object_name):
        """Execute complex grasp sequence"""
        self.node.get_logger().info(f"🤏 Executing complex grasp: {object_name}")

        if object_name not in self.waypoints.get("objects", {}):
            self.node.get_logger().error(f"Unknown object: {object_name}")
            return False

        obj_data = self.waypoints["objects"][object_name]

        # 1) Gripper on_approach
        pose = obj_data.get("gripper_hooks", {}).get("on_approach")
        if pose:
            joints = self.waypoints.get("poses", {}).get(pose)
            if not joints or not self.motion.plan_execute_gripper(joints, "gripper-open"):
                return False

        # 2) Approach
        joints = obj_data.get("approach")
        if not joints or not self.motion.plan_execute_arm(joints, "approach"):
            return False

        # 3) Gripper after_approach
        pose = obj_data.get("gripper_hooks", {}).get("after_approach")
        if pose:
            joints = self.waypoints.get("poses", {}).get(pose)
            if not joints or not self.motion.plan_execute_gripper(joints, "gripper-close"):
                return False

        # 4) Attach object to gripper in planning scene
        # NOTE: Disabled - visual_state_manager handles this based on feedback
        # self.scene.attach_object(object_name, "end_effector_link")

        self.node.get_logger().info(f"✅ Grasp '{object_name}' done")
        return True

    def execute_release(self, object_name, place_joints):
        """Execute release sequence"""
        self.node.get_logger().info(f"✋ Executing release: {object_name}")

        if object_name not in self.waypoints.get("objects", {}):
            self.node.get_logger().error(f"Unknown object: {object_name}")
            return False

        # 1) Move to place position
        if not place_joints:
            self.node.get_logger().error(f"Cannot resolve place for '{object_name}'")
            return False
        if not self.motion.plan_execute_arm(place_joints, "place"):
            return False

        # 2) Open gripper
        open_joints = self.waypoints.get("poses", {}).get(self.open_gripper_pose)
        if not open_joints or not self.motion.plan_execute_gripper(open_joints, "release-gripper"):
            return False

        # 3) Detach object from gripper
        # NOTE: Disabled - visual_state_manager handles this based on feedback
        # self.scene.detach_object(object_name, "end_effector_link")

        # 4) Retreat
        retreat_joints = self.waypoints["objects"][object_name].get("retreat")
        if retreat_joints and not self.motion.plan_execute_arm(retreat_joints, "retreat"):
            self.node.get_logger().warn("⚠️  Retreat failed, but release completed")

        self.node.get_logger().info(f"✅ Release '{object_name}' complete")
        return True

    def execute_dismantle(self, targets, place_default):
        """Execute dismantle sequence for multiple objects"""
        for i, obj in enumerate(targets):
            self.node.get_logger().info(f"Dismantle step {i+1}/{len(targets)}: {obj}")

            if not self.execute_grasp(obj):
                return False

            # Resolve place position
            place_joints = self._resolve_place_joints(obj, place_default)
            if not place_joints:
                self.node.get_logger().error(f"Cannot resolve place for {obj}")
                return False

            if not self.motion.plan_execute_arm(place_joints, "place"):
                return False

            # Open gripper
            open_joints = self.waypoints.get("poses", {}).get(self.open_gripper_pose)
            if not self.motion.plan_execute_gripper(open_joints, "openAfterPlace"):
                return False

            # Retreat
            retreat_joints = self.waypoints["objects"][obj].get("retreat")
            if retreat_joints and not self.motion.plan_execute_arm(retreat_joints, "retreat"):
                return False

        return True

    def _resolve_place_joints(self, object_name, place_in):
        """Resolve place joints from object and place info"""
        def try_bin_to_pose(bin_name):
            try:
                pose_name = self.waypoints["scene"]["bins"][bin_name]["pose_name"]
                return self.waypoints.get("poses", {}).get(pose_name)
            except (KeyError, TypeError):
                return None

        # From explicit 'place' parameter
        if place_in:
            if "bin" in place_in:
                joints = try_bin_to_pose(place_in["bin"])
                if joints:
                    return joints
            if "pose_name" in place_in:
                joints = self.waypoints.get("poses", {}).get(place_in["pose_name"])
                if joints:
                    return joints
            if "joints" in place_in and place_in["joints"]:
                return place_in["joints"]

        # From object's default place
        if object_name in self.waypoints.get("objects", {}):
            obj_data = self.waypoints["objects"][object_name]
            if "place" in obj_data and isinstance(obj_data["place"], list):
                return obj_data["place"]

        return None
