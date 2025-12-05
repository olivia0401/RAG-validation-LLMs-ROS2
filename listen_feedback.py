#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class FeedbackListener(Node):
    def __init__(self):
        super().__init__('feedback_listener')
        self.subscription = self.create_subscription(
            String,
            '/llm_feedback',
            self.feedback_callback,
            10)
        self.get_logger().info('Listening to /llm_feedback...')

    def feedback_callback(self, msg):
        try:
            data = json.loads(msg.data)
            print(json.dumps(data, indent=2))
        except:
            print(f"Raw: {msg.data}")

def main():
    rclpy.init()
    listener = FeedbackListener()
    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
