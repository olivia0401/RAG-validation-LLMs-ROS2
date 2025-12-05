#!/usr/bin/env python3
"""
Publish initial joint states for fake hardware mode.
This gives MoveIt an initial state to start from.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class InitialJointStatePublisher(Node):
    def __init__(self):
        super().__init__('initial_joint_state_publisher')

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Publish initial state at 10Hz for better state tracking
        # Higher frequency ensures MoveIt always has recent joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        # Auto-shutdown after 5 seconds to hand over to skill_server
        self.start_time = self.get_clock().now()
        self.shutdown_duration = 5.0  # seconds

        # Safe initial joint positions (non-colliding)
        # Using HOME position from waypoints.json to match launch file initial_positions
        self.joint_state = JointState()
        self.joint_state.name = [
            # Arm joints
            'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'joint_5', 'joint_6', 'joint_7',
            # Gripper main joint (mimic joints follow automatically)
            'robotiq_85_left_knuckle_joint'
        ]
        # HOME position from waypoints.json - matches launch file initial_positions
        # Gripper: left_knuckle at 0.0 = OPEN state (mimic joints follow automatically)
        self.joint_state.position = [
            # Arm joints
            0.0,      # joint_1
            0.2618,   # joint_2 (15 degrees)
            3.14159,  # joint_3 (180 degrees)
            -2.2689,  # joint_4 (-130 degrees)
            0.0,      # joint_5
            0.9599,   # joint_6 (55 degrees)
            1.5708,   # joint_7 (90 degrees)
            # Gripper main joint (all mimic joints follow this automatically via URDF)
            0.0       # robotiq_85_left_knuckle_joint (main actuated joint - OPEN)
        ]

        self.get_logger().info('Publishing joint states at 10Hz (7 arm + 1 gripper joint, OPEN)...')

    def publish_joint_states(self):
        # Check if we should shutdown
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.shutdown_duration:
            self.get_logger().info(f'✅ Handed over joint_states to skill_server after {elapsed:.1f}s. Shutting down.')
            raise SystemExit  # Cleanly exit the node

        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = InitialJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
