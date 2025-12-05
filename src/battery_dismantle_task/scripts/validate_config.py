#!/usr/bin/env python3
"""
配置一致性验证脚本
检查 waypoints.json 和 skills.json 的配置是否一致
"""
import json
import sys
from pathlib import Path

def load_json(filepath):
    """加载JSON文件"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)

def validate_configs():
    """验证所有配置文件的一致性"""

    # 获取配置文件路径
    pkg_dir = Path(__file__).parent.parent
    waypoints_path = pkg_dir / "config" / "waypoints.json"
    skills_path = pkg_dir / "llm_agent" / "config" / "skills.json"

    print("=" * 60)
    print("配置一致性验证")
    print("=" * 60)

    # 加载配置
    print(f"\n📂 加载配置文件...")
    print(f"  - waypoints.json: {waypoints_path}")
    print(f"  - skills.json: {skills_path}")

    waypoints = load_json(waypoints_path)
    skills_config = load_json(skills_path)

    errors = []
    warnings = []

    # 验证1: 检查skills.json中的可用姿态是否在waypoints.json中存在
    print(f"\n✅ 验证1: 检查可用姿态是否存在于waypoints中")
    available_poses = skills_config.get('available_poses', [])
    waypoint_poses = waypoints.get('poses', {}).keys()

    for pose in available_poses:
        if pose not in waypoint_poses:
            errors.append(f"  ❌ 姿态 '{pose}' 在skills.json中定义但不存在于waypoints.json")
        else:
            print(f"  ✓ {pose}: OK")

    # 验证2: 检查skills.json中的对象是否在waypoints.json中存在
    print(f"\n✅ 验证2: 检查可用对象是否存在于waypoints中")
    available_objects = skills_config.get('available_objects', [])
    waypoint_objects = waypoints.get('objects', {}).keys()

    for obj in available_objects:
        if obj not in waypoint_objects:
            errors.append(f"  ❌ 对象 '{obj}' 在skills.json中定义但不存在于waypoints.json")
        else:
            print(f"  ✓ {obj}: OK")

    # 验证3: 检查对象的approach姿态是否唯一
    print(f"\n✅ 验证3: 检查对象姿态的唯一性")
    objects = waypoints.get('objects', {})
    approach_poses = {}
    place_poses = {}

    for obj_name, obj_config in objects.items():
        # 检查approach
        if 'approach' in obj_config:
            approach_key = str(obj_config['approach'])
            if approach_key in approach_poses:
                warnings.append(f"  ⚠️  对象 '{obj_name}' 和 '{approach_poses[approach_key]}' 使用相同的approach姿态")
            else:
                approach_poses[approach_key] = obj_name
                print(f"  ✓ {obj_name}.approach: 唯一")

        # 检查place
        if 'place' in obj_config:
            place_key = str(obj_config['place'])
            if place_key in place_poses:
                warnings.append(f"  ⚠️  对象 '{obj_name}' 和 '{place_poses[place_key]}' 使用相同的place姿态")
            else:
                place_poses[place_key] = obj_name
                print(f"  ✓ {obj_name}.place: 唯一")

    # 验证4: 检查对象的必需字段
    print(f"\n✅ 验证4: 检查对象配置完整性")
    required_fields = ['approach', 'place', 'retreat', 'gripper_hooks', 'io']

    for obj_name, obj_config in objects.items():
        missing_fields = [f for f in required_fields if f not in obj_config]
        if missing_fields:
            errors.append(f"  ❌ 对象 '{obj_name}' 缺少字段: {', '.join(missing_fields)}")
        else:
            print(f"  ✓ {obj_name}: 所有必需字段完整")

        # 检查gripper_hooks
        if 'gripper_hooks' in obj_config:
            hooks = obj_config['gripper_hooks']
            required_hooks = ['on_approach', 'after_approach', 'after_place']
            missing_hooks = [h for h in required_hooks if h not in hooks]
            if missing_hooks:
                errors.append(f"  ❌ 对象 '{obj_name}' 缺少gripper_hooks: {', '.join(missing_hooks)}")
            else:
                # 验证引用的夹爪姿态是否存在
                for hook_name, pose_name in hooks.items():
                    if pose_name not in waypoint_poses:
                        errors.append(f"  ❌ 对象 '{obj_name}' 的 gripper_hooks.{hook_name} 引用了不存在的姿态 '{pose_name}'")

    # 验证5: 检查关节数量的一致性
    print(f"\n✅ 验证5: 检查关节数量一致性")
    if 'joints' in waypoints:
        arm_joints = waypoints['joints'].get('arm', [])
        expected_arm_dof = len(arm_joints)
        print(f"  期望的臂关节数量: {expected_arm_dof}")

        # 检查所有姿态的关节数量
        for pose_name, joint_values in waypoints['poses'].items():
            # 跳过夹爪姿态（OPEN, CLOSE只有1个关节）
            if pose_name in ['OPEN', 'CLOSE']:
                continue

            if len(joint_values) != expected_arm_dof:
                errors.append(f"  ❌ 姿态 '{pose_name}' 有 {len(joint_values)} 个关节值，期望 {expected_arm_dof}")
            else:
                print(f"  ✓ {pose_name}: {len(joint_values)} 个关节 (正确)")

        # 检查对象姿态的关节数量
        for obj_name, obj_config in objects.items():
            for field in ['approach', 'place', 'retreat']:
                if field in obj_config and isinstance(obj_config[field], list):
                    if len(obj_config[field]) != expected_arm_dof:
                        errors.append(f"  ❌ 对象 '{obj_name}.{field}' 有 {len(obj_config[field])} 个关节值，期望 {expected_arm_dof}")
                    else:
                        print(f"  ✓ {obj_name}.{field}: {len(obj_config[field])} 个关节 (正确)")

    # 输出总结
    print(f"\n" + "=" * 60)
    print("验证总结")
    print("=" * 60)

    if not errors and not warnings:
        print("✅ 所有验证通过！配置文件一致性良好。")
        return 0

    if warnings:
        print(f"\n⚠️  发现 {len(warnings)} 个警告:")
        for warning in warnings:
            print(warning)

    if errors:
        print(f"\n❌ 发现 {len(errors)} 个错误:")
        for error in errors:
            print(error)
        print("\n请修复以上错误后再运行系统。")
        return 1

    return 0

if __name__ == "__main__":
    exit_code = validate_configs()
    sys.exit(exit_code)
