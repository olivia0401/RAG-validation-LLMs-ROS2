#!/usr/bin/env python3
"""
使用MoveIt IK服务计算准确的waypoints
"""
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped
import json

class WaypointCalculator(Node):
    def __init__(self):
        super().__init__('waypoint_calculator')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /compute_ik 服务...')

        self.get_logger().info('✅ IK服务已连接')

    def compute_ik(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """计算给定笛卡尔位置的关节角度"""
        import math
        from tf_transformations import quaternion_from_euler

        request = GetPositionIK.Request()
        request.ik_request.group_name = 'manipulator'
        request.ik_request.avoid_collisions = True

        # 设置目标位姿
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z

        # 转换欧拉角到四元数
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout.sec = 5

        # 调用IK服务
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.error_code.val == 1:  # SUCCESS
                joints = response.solution.joint_state.position[:7]
                self.get_logger().info(f'✅ IK成功: [{", ".join([f"{j:.3f}" for j in joints])}]')
                return list(joints)
            else:
                self.get_logger().error(f'❌ IK失败: error_code={response.error_code.val}')
                return None
        else:
            self.get_logger().error('❌ IK服务调用失败')
            return None

def main():
    rclpy.init()
    calculator = Node('waypoint_calculator')

    # 等待IK服务
    ik_client = calculator.create_client(GetPositionIK, '/compute_ik')
    calculator.get_logger().info('等待 /compute_ik 服务...')

    while not ik_client.wait_for_service(timeout_sec=1.0):
        calculator.get_logger().info('等待 /compute_ik 服务...')

    calculator.get_logger().info('✅ IK服务已连接')

    # 定义目标位姿
    import math

    def euler_to_quaternion(roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0, 0, 0, 0]
        q[3] = cr * cp * cy + sr * sp * sy  # w
        q[0] = sr * cp * cy - cr * sp * sy  # x
        q[1] = cr * sp * cy + sr * cp * sy  # y
        q[2] = cr * cp * sy - sr * sp * cy  # z
        return q

    waypoints = {}

    # 1. TopCoverBolts approach (从上方接近)
    # 物体位置: x=0.45, y=0.0, z=0.08 (顶部)
    # 夹爪应该在 z=0.25 (物体上方15cm)
    calculator.get_logger().info('\n计算 TopCoverBolts approach...')

    request = GetPositionIK.Request()
    request.ik_request.group_name = 'manipulator'
    request.ik_request.avoid_collisions = False

    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.pose.position.x = 0.45
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.25

    # 夹爪向下 (pitch = -90度)
    q = euler_to_quaternion(0, math.pi/2, 0)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    request.ik_request.pose_stamped = pose
    request.ik_request.timeout.sec = 5

    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(calculator, future, timeout_sec=10.0)

    if future.result() and future.result().error_code.val == 1:
        joints = list(future.result().solution.joint_state.position[:7])
        waypoints['TopCoverBolts_approach'] = [round(j, 4) for j in joints]
        calculator.get_logger().info(f'✅ TopCoverBolts approach: {waypoints["TopCoverBolts_approach"]}')
    else:
        calculator.get_logger().error('❌ TopCoverBolts approach IK失败')

    # 2. TopCoverBolts place (托盘位置)
    calculator.get_logger().info('\n计算 TopCoverBolts place...')

    pose.pose.position.x = 0.3
    pose.pose.position.y = -0.4
    pose.pose.position.z = 0.15

    request.ik_request.pose_stamped = pose
    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(calculator, future, timeout_sec=10.0)

    if future.result() and future.result().error_code.val == 1:
        joints = list(future.result().solution.joint_state.position[:7])
        waypoints['TopCoverBolts_place'] = [round(j, 4) for j in joints]
        calculator.get_logger().info(f'✅ TopCoverBolts place: {waypoints["TopCoverBolts_place"]}')
    else:
        calculator.get_logger().error('❌ TopCoverBolts place IK失败')

    # 3. BatteryBox approach (从侧面接近)
    calculator.get_logger().info('\n计算 BatteryBox_0 approach...')

    pose.pose.position.x = 0.45
    pose.pose.position.y = -0.15
    pose.pose.position.z = 0.15

    # 夹爪侧向
    q = euler_to_quaternion(0, math.pi/2, -math.pi/4)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    request.ik_request.pose_stamped = pose
    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(calculator, future, timeout_sec=10.0)

    if future.result() and future.result().error_code.val == 1:
        joints = list(future.result().solution.joint_state.position[:7])
        waypoints['BatteryBox_0_approach'] = [round(j, 4) for j in joints]
        calculator.get_logger().info(f'✅ BatteryBox_0 approach: {waypoints["BatteryBox_0_approach"]}')
    else:
        calculator.get_logger().error('❌ BatteryBox_0 approach IK失败')

    # 4. BatteryBox place (回收箱位置)
    calculator.get_logger().info('\n计算 BatteryBox_0 place...')

    pose.pose.position.x = 0.3
    pose.pose.position.y = 0.4
    pose.pose.position.z = 0.15

    request.ik_request.pose_stamped = pose
    future = ik_client.call_async(request)
    rclpy.spin_until_future_complete(calculator, future, timeout_sec=10.0)

    if future.result() and future.result().error_code.val == 1:
        joints = list(future.result().solution.joint_state.position[:7])
        waypoints['BatteryBox_0_place'] = [round(j, 4) for j in joints]
        calculator.get_logger().info(f'✅ BatteryBox_0 place: {waypoints["BatteryBox_0_place"]}')
    else:
        calculator.get_logger().error('❌ BatteryBox_0 place IK失败')

    # 打印结果
    calculator.get_logger().info('\n\n========== 计算结果 ==========')
    calculator.get_logger().info(json.dumps(waypoints, indent=2))

    # 保存到文件
    with open('/home/olivia/llms-ros2/computed_waypoints.json', 'w') as f:
        json.dump(waypoints, f, indent=2)

    calculator.get_logger().info('\n✅ 已保存到 /home/olivia/llms-ros2/computed_waypoints.json')

    calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
