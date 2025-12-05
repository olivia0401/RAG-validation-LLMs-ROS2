#!/usr/bin/env python3
"""直接测试MoveIt接口"""
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time

def main():
    rclpy.init()

    print("🔧 初始化MoveItPy...")
    try:
        moveit = MoveItPy(node_name="test_moveit_direct")
        print("✅ MoveItPy初始化成功")

        # 获取manipulator planning component
        manipulator = moveit.get_planning_component("manipulator")
        print(f"✅ 获取到planning component: manipulator")

        # 设置目标为HOME位置
        print("\n📍 设置目标: HOME [0.0, -1.57, 1.57, 0.0, 1.57, 0.0, 0.0]")
        manipulator.set_goal_state(
            configuration_name="HOME"
        )

        # 规划
        print("🎯 开始规划...")
        plan_result = manipulator.plan()

        if plan_result:
            print("✅ 规划成功！")
            print(f"  轨迹点数: {len(plan_result.trajectory.joint_trajectory.points)}")

            # 执行
            print("🚀 执行轨迹...")
            robot = moveit.get_robot()
            success = robot.execute(plan_result.trajectory, blocking=True)

            if success:
                print("✅ 执行成功！")
            else:
                print("❌ 执行失败")
        else:
            print("❌ 规划失败")

    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
