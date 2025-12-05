#!/usr/bin/env python3
"""使用MoveGroup接口直接测试MoveIt"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveGroupInterface
import time

def main():
    rclpy.init()
    node = Node("test_moveit_interface")

    print("🔧 初始化MoveGroupInterface...")
    try:
        # 创建manipulator的MoveGroupInterface
        move_group = MoveGroupInterface(node=node, group_name="manipulator")
        print("✅ MoveGroupInterface初始化成功")

        # 设置目标为HOME (joint values)
        print("\n📍 设置目标关节角度: HOME [0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0]")
        move_group.set_joint_value_target([0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0])

        # 规划
        print("🎯 开始规划...")
        success = move_group.go(wait=True)

        if success:
            print("✅ 规划并执行成功！")
        else:
            print("❌ 失败")

        # 停止运动
        move_group.stop()

    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
