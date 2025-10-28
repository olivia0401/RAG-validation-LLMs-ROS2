#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningSceneWorld, CollisionObject, AllowedCollisionMatrix, AllowedCollisionEntry
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene
import time

class ScenePublisher(Node):
    def __init__(self):
        super().__init__('scene_publisher')

        # 使用 planning scene 服务来添加对象
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # 等待服务可用
        self.get_logger().info('Waiting for /apply_planning_scene service...')
        while not self.scene_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available, publishing scene...')
        self.publish_scene()

    def publish_scene(self):
        # 创建 PlanningScene 请求
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world = PlanningSceneWorld()

        # 尺寸定义（方便调整）
        BATTERY_LENGTH = 0.35  # X方向：长35cm
        BATTERY_WIDTH = 0.25   # Y方向：宽25cm
        BATTERY_HEIGHT = 0.08  # Z方向：高8cm（降低电池高度）

        COVER_LENGTH = 0.36    # X方向：比电池稍大0.5cm每边
        COVER_WIDTH = 0.26     # Y方向：比电池稍大0.5cm每边
        COVER_THICKNESS = 0.01  # Z方向：厚度1cm（更薄更真实）

        # 位置定义
        BATTERY_BASE_X = 0.45  # Match original working configuration
        BATTERY_BASE_Y = 0.0
        BATTERY_BASE_Z = 0.0   # 电池底部在地面

        # BatteryBox_0 - 电池主体（参考实际锂电池模块尺寸）
        battery_box_0 = CollisionObject()
        battery_box_0.header.frame_id = 'base_link'
        battery_box_0.id = 'BatteryBox_0'
        battery_box_0.operation = CollisionObject.ADD

        primitive_box_0 = SolidPrimitive()
        primitive_box_0.type = SolidPrimitive.BOX
        primitive_box_0.dimensions = [BATTERY_LENGTH, BATTERY_WIDTH, BATTERY_HEIGHT]

        pose_box_0 = Pose()
        pose_box_0.position.x = BATTERY_BASE_X
        pose_box_0.position.y = BATTERY_BASE_Y
        pose_box_0.position.z = BATTERY_BASE_Z + BATTERY_HEIGHT/2  # 中心点在高度一半处
        pose_box_0.orientation.w = 1.0

        battery_box_0.primitives.append(primitive_box_0)
        battery_box_0.primitive_poses.append(pose_box_0)

        # TopCoverBolts - 电池顶盖（薄板，带有螺栓固定点）
        top_cover_bolts = CollisionObject()
        top_cover_bolts.header.frame_id = 'base_link'
        top_cover_bolts.id = 'TopCoverBolts'
        top_cover_bolts.operation = CollisionObject.ADD

        primitive_bolts = SolidPrimitive()
        primitive_bolts.type = SolidPrimitive.BOX
        primitive_bolts.dimensions = [COVER_LENGTH, COVER_WIDTH, COVER_THICKNESS]

        pose_bolts = Pose()
        pose_bolts.position.x = BATTERY_BASE_X
        pose_bolts.position.y = BATTERY_BASE_Y
        # 顶盖嵌入电池表面（与原始配置一致）
        pose_bolts.position.z = BATTERY_BASE_Z + BATTERY_HEIGHT - COVER_THICKNESS/2
        pose_bolts.orientation.w = 1.0

        top_cover_bolts.primitives.append(primitive_bolts)
        top_cover_bolts.primitive_poses.append(pose_bolts)

        # 添加所有对象到场景（WorkTable已删除，因为不需要）
        req.scene.world.collision_objects.append(battery_box_0)
        req.scene.world.collision_objects.append(top_cover_bolts)

        # 添加允许碰撞矩阵 - 允许TopCoverBolts和BatteryBox_0碰撞
        acm = AllowedCollisionMatrix()
        acm.entry_names = ['BatteryBox_0', 'TopCoverBolts']

        # 创建2x2矩阵: [BatteryBox_0, TopCoverBolts]
        entry1 = AllowedCollisionEntry()
        entry1.enabled = [False, True]  # BatteryBox_0 vs [BatteryBox_0, TopCoverBolts]

        entry2 = AllowedCollisionEntry()
        entry2.enabled = [True, False]  # TopCoverBolts vs [BatteryBox_0, TopCoverBolts]

        acm.entry_values = [entry1, entry2]
        req.scene.allowed_collision_matrix = acm

        # 调用服务
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info('成功添加场景对象: 电池主体和顶盖')
        else:
            self.get_logger().error('添加场景对象失败')

def main(args=None):
    rclpy.init(args=args)
    scene_publisher = ScenePublisher()
    time.sleep(10.0)  # 等待一秒确保消息发送
    scene_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
