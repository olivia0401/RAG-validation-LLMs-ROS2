#!/usr/bin/env python3
"""
可视化状态管理器 - 完全负责场景物体的创建和移动
在RViz中显示物体被抓取和移动的效果

工作原理：
1. 启动时创建电池和顶盖collision objects
2. 监听skill反馈
3. grasp成功 → 从场景移除物体，创建跟随夹爪的attached object
4. release成功 → 在目标位置重新创建物体
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from tf2_ros import TransformListener, Buffer
import json


class VisualStateManager(Node):
    def __init__(self):
        super().__init__('visual_state_manager')

        # 订阅skill反馈
        self.feedback_sub = self.create_subscription(
            String,
            '/llm_feedback',
            self.feedback_callback,
            10
        )

        # 订阅LLM命令以跟踪当前操作的目标
        self.command_sub = self.create_subscription(
            String,
            '/llm_command',
            self.command_callback,
            10
        )

        # 场景服务客户端
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')

        # 等待服务可用
        self.get_logger().info('等待 /apply_planning_scene 服务...')
        while not self.scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /apply_planning_scene 服务...')

        # TF监听器（用于获取夹爪位姿）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 物体尺寸定义（与publish_scene.py保持一致）
        BATTERY_BASE_X = 0.45
        BATTERY_BASE_Y = 0.0
        BATTERY_BASE_Z = 0.0
        BATTERY_HEIGHT = 0.08
        COVER_THICKNESS = 0.01

        self.object_definitions = {
            'TopCoverBolts': {
                'dimensions': [0.36, 0.26, 0.01],  # 顶盖
                'initial_pose': {
                    'x': BATTERY_BASE_X,
                    'y': BATTERY_BASE_Y,
                    'z': BATTERY_BASE_Z + BATTERY_HEIGHT - COVER_THICKNESS/2
                },
                'place_pose': {
                    'x': 0.3,
                    'y': -0.4,
                    'z': 0.005  # 托盘位置
                }
            },
            'BatteryBox_0': {
                'dimensions': [0.35, 0.25, 0.08],  # 电池主体
                'initial_pose': {
                    'x': BATTERY_BASE_X,
                    'y': BATTERY_BASE_Y,
                    'z': BATTERY_BASE_Z + 0.04  # 中心点
                },
                'place_pose': {
                    'x': 0.3,
                    'y': 0.4,
                    'z': 0.04  # 回收箱位置
                }
            }
        }

        # 当前抓取的物体
        self.grasped_object = None

        # 当前目标对象
        self.current_target = None

        # 等待一下让move_group完全启动
        self.get_logger().info('等待3秒让move_group完全启动...')
        import time
        time.sleep(3)

        # 初始化场景
        self.initialize_scene()

        # 定时器：更新attached object位姿（10Hz）
        self.update_timer = self.create_timer(0.1, self.update_attached_object)

        self.get_logger().info('✅ Visual State Manager ready!')

    def command_callback(self, msg):
        """处理LLM命令，提取当前操作的目标物体"""
        try:
            command_json = json.loads(msg.data)
            if 'params' in command_json and 'target' in command_json['params']:
                target = command_json['params']['target']
                if target in self.object_definitions:
                    self.current_target = target
                    self.get_logger().info(f"📝 当前目标: {target}")
        except Exception as e:
            self.get_logger().debug(f"Command parse error: {e}")

    def initialize_scene(self):
        """初始化场景：创建所有collision objects"""
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True

        # 创建TopCoverBolts
        top_cover = self.create_collision_object(
            'TopCoverBolts',
            self.object_definitions['TopCoverBolts']['dimensions'],
            self.object_definitions['TopCoverBolts']['initial_pose']
        )
        req.scene.world.collision_objects.append(top_cover)

        # 创建BatteryBox_0
        battery = self.create_collision_object(
            'BatteryBox_0',
            self.object_definitions['BatteryBox_0']['dimensions'],
            self.object_definitions['BatteryBox_0']['initial_pose']
        )
        req.scene.world.collision_objects.append(battery)

        # 添加允许碰撞矩阵 - 允许TopCoverBolts和BatteryBox_0碰撞
        from moveit_msgs.msg import AllowedCollisionMatrix, AllowedCollisionEntry
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() and future.result().success:
            self.get_logger().info('✅ 成功创建场景对象: TopCoverBolts, BatteryBox_0')
        else:
            self.get_logger().error('❌ 创建场景对象失败')

    def create_collision_object(self, object_id, dimensions, pose_dict):
        """创建collision object"""
        collision_obj = CollisionObject()
        collision_obj.header.frame_id = 'world'
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.ADD

        # 添加形状
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = dimensions

        # 设置位姿
        pose = Pose()
        pose.position.x = pose_dict['x']
        pose.position.y = pose_dict['y']
        pose.position.z = pose_dict['z']
        pose.orientation.w = 1.0

        collision_obj.primitives.append(primitive)
        collision_obj.primitive_poses.append(pose)

        return collision_obj

    def feedback_callback(self, msg):
        """处理skill反馈"""
        try:
            data = msg.data

            # 尝试解析JSON格式的feedback
            try:
                feedback_json = json.loads(data)

                # 检查grasp成功
                if (feedback_json.get('status') == 'success' and
                    'message' in feedback_json and
                    'grasp' in feedback_json['message'].lower() and
                    'completed' in feedback_json['message'].lower()):

                    # 使用current_target（从command_callback中设置）
                    if self.current_target and self.current_target in self.object_definitions:
                        self.get_logger().info(f"🤏 检测到抓取成功: {self.current_target}")
                        self.attach_object_visual(self.current_target)

                # 检查release成功
                elif (feedback_json.get('status') == 'success' and
                      'message' in feedback_json and
                      'release' in feedback_json['message'].lower() and
                      'completed' in feedback_json['message'].lower()):

                    if self.grasped_object:
                        self.get_logger().info(f"✋ 检测到放置成功: {self.grasped_object}")
                        self.detach_object_visual(self.grasped_object)

            except json.JSONDecodeError:
                # 旧格式兼容
                if 'grasp' in data and 'completed' in data:
                    if self.current_target and self.current_target in self.object_definitions:
                        self.get_logger().info(f"🤏 检测到抓取成功: {self.current_target}")
                        self.attach_object_visual(self.current_target)

                elif 'release' in data and 'completed' in data:
                    if self.grasped_object:
                        self.get_logger().info(f"✋ 检测到放置成功: {self.grasped_object}")
                        self.detach_object_visual(self.grasped_object)

        except Exception as e:
            self.get_logger().debug(f"Feedback parse error: {e}")  # 改为debug避免刷屏

    def attach_object_visual(self, object_id):
        """可视化：物体被抓取（从world中移除，准备跟随夹爪）"""
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True

        # 移除原始物体
        collision_obj = CollisionObject()
        collision_obj.id = object_id
        collision_obj.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects.append(collision_obj)

        # 调用服务
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().success:
            self.grasped_object = object_id
            self.get_logger().info(f"  ✅ {object_id} 已从场景移除（准备跟随夹爪）")
        else:
            self.get_logger().warn(f"  ⚠️ 无法移除 {object_id}")

    def detach_object_visual(self, object_id):
        """可视化：物体被放置（在目标位置重新创建）"""
        # 先移除跟随的临时物体
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True

        temp_obj = CollisionObject()
        temp_obj.id = f"{object_id}_attached"
        temp_obj.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects.append(temp_obj)

        self.scene_client.call_async(req)

        # 在目标位置重新创建物体
        if object_id not in self.object_definitions:
            self.get_logger().warn(f"未定义 {object_id} 的放置位置")
            self.grasped_object = None
            return

        req2 = ApplyPlanningScene.Request()
        req2.scene = PlanningScene()
        req2.scene.is_diff = True

        # 在放置位置创建物体
        placed_obj = self.create_collision_object(
            object_id,
            self.object_definitions[object_id]['dimensions'],
            self.object_definitions[object_id]['place_pose']
        )
        req2.scene.world.collision_objects.append(placed_obj)

        # 调用服务
        future = self.scene_client.call_async(req2)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() and future.result().success:
            self.get_logger().info(f"  ✅ {object_id} 已放置到目标位置")
            self.grasped_object = None
        else:
            self.get_logger().warn(f"  ⚠️ 无法放置 {object_id}")

    def update_attached_object(self):
        """定时更新：让抓取的物体跟随夹爪移动"""
        if self.grasped_object is None:
            return

        try:
            # 获取夹爪的当前位姿
            transform = self.tf_buffer.lookup_transform(
                'world',
                'robotiq_85_base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )

            # 在夹爪位置创建/更新物体
            req = ApplyPlanningScene.Request()
            req.scene = PlanningScene()
            req.scene.is_diff = True

            collision_obj = CollisionObject()
            collision_obj.header.frame_id = 'world'
            collision_obj.id = f"{self.grasped_object}_attached"
            collision_obj.operation = CollisionObject.ADD

            # 添加形状
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = self.object_definitions[self.grasped_object]['dimensions']

            # 设置位姿（物体跟随夹爪，偏移根据物体尺寸调整）
            pose = Pose()
            # 物体中心在夹爪下方，距离 = 夹爪指尖长度(约8cm) + 物体高度的一半
            object_height = self.object_definitions[self.grasped_object]['dimensions'][2]
            offset_z = -0.08 - object_height / 2.0  # 夹爪下方

            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z + offset_z
            pose.orientation = transform.transform.rotation

            collision_obj.primitives.append(primitive)
            collision_obj.primitive_poses.append(pose)
            req.scene.world.collision_objects.append(collision_obj)

            # 异步调用（不等待结果，避免阻塞）
            self.scene_client.call_async(req)

        except Exception as e:
            # TF查询可能失败（正常情况，不打印错误）
            pass


def main(args=None):
    rclpy.init(args=args)
    node = VisualStateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
