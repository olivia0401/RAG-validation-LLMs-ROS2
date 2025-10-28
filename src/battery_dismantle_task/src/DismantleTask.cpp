#include <battery_dismantle_task/DismantleTask.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <sstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("DismantleTask");

DismantleTask::DismantleTask(const std::string& task_name, const rclcpp::Node::SharedPtr& node)
    : task_name_(task_name), 
      node_(node),
      robot_model_loader_(std::make_shared<robot_model_loader::RobotModelLoader>(node)),
      robot_model_(robot_model_loader_->getModel())
{
    if (!robot_model_) {
        RCLCPP_ERROR(LOGGER, "Failed to load robot model.");
        throw std::runtime_error("Failed to load robot model.");
    }
}

bool DismantleTask::init(const Waypoints& waypoints, const BatteryConfig& battery_config, const std::vector<std::string>& target_objects) {
    waypoints_ = waypoints;
    battery_config_ = battery_config;
    target_objects_ = target_objects;

    RCLCPP_INFO(LOGGER, "DismantleTask initialized for %zu objects", target_objects_.size());
    
    // Spawn battery and parts in the planning scene
    spawnBatteryAndSubparts();

    // Wait for the world to contain the objects
    if (!waitWorldContains(target_objects_, 5.0)) {
        RCLCPP_ERROR(LOGGER, "Failed to find all target objects in the planning scene.");
        return false;
    }

    return true;
}

bool DismantleTask::plan() {
    RCLCPP_INFO(LOGGER, "=== 开始构建MTC任务 ===");
    task_ = std::make_shared<mtc::Task>();
    task_->stages()->setName(task_name_);
    task_->setRobotModel(robot_model_);

    // Start from current state
    RCLCPP_INFO(LOGGER, "添加阶段: CurrentState（获取当前机器人状态）");
    task_->add(std::make_unique<mtc::stages::CurrentState>("current_state"));

    // Create a joint-space solver
    auto joint_solver = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    RCLCPP_INFO(LOGGER, "创建关节插值规划器");

    for (const auto& object_id : target_objects_) {
        RCLCPP_INFO(LOGGER, "=== 为对象 '%s' 构建拆卸序列 ===", object_id.c_str());
        const auto& seq = waypoints_.objects.at(object_id);

        // 1. Open gripper before approaching (现在可以安全使用，因为初始状态已设置)
        if (!seq.gripper_hooks.on_approach.empty()) {
            RCLCPP_INFO(LOGGER, "  步骤1: 打开夹爪 -> 目标姿态='%s'", seq.gripper_hooks.on_approach.c_str());
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper before " + object_id, joint_solver);
            stage->setGroup("gripper");
            stage->setGoal(waypoints_.gripperGoal(waypoints_.poses.at(seq.gripper_hooks.on_approach)));
            task_->add(std::move(stage));
        }

        // 2. Move to approach
        RCLCPP_INFO(LOGGER, "  步骤2: 移动到接近位置");
        auto stage_approach = std::make_unique<mtc::stages::MoveTo>("approach " + object_id, joint_solver);
        stage_approach->setGroup("manipulator");
        stage_approach->setGoal(waypoints_.armGoal(seq.approach));
        task_->add(std::move(stage_approach));

        // 3a. 允许gripper与目标物体碰撞（添加到ACM）
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper collision with " + object_id);
            stage->allowCollisions(object_id,
                task_->getRobotModel()->getJointModelGroup("gripper")->getLinkModelNames(),
                true);
            task_->add(std::move(stage));
        }

        // 3b. 允许电池主体与目标物体碰撞（因为它们本来就装配在一起）
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow battery collision with " + object_id);
            stage->allowCollisions(battery_config_.id, {object_id}, true);
            task_->add(std::move(stage));
        }

        // 3c. Close gripper after approaching
        if (!seq.gripper_hooks.after_approach.empty()) {
            auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", joint_solver);
            stage->setGroup("gripper");
            stage->setGoal(waypoints_.gripperGoal(waypoints_.poses.at(seq.gripper_hooks.after_approach)));
            task_->add(std::move(stage));
        }

        // 4. Attach object
        if (seq.io.do_attach_after_approach) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach " + object_id);
            stage->attachObject(object_id, "robotiq_85_base_link");
            task_->add(std::move(stage));
        }

        // 5. Move to place
        auto stage_place = std::make_unique<mtc::stages::MoveTo>("place " + object_id, joint_solver);
        stage_place->setGroup("manipulator");
        stage_place->setGoal(waypoints_.armGoal(seq.place));
        task_->add(std::move(stage_place));

        // 6. Open gripper after placing
        if (!seq.gripper_hooks.after_place.empty()) {
            auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper after place", joint_solver);
            stage->setGroup("gripper");
            stage->setGoal(waypoints_.gripperGoal(waypoints_.poses.at(seq.gripper_hooks.after_place)));
            task_->add(std::move(stage));
        }

        // 7. Detach object
        if (seq.io.do_detach_after_place) {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach " + object_id);
            stage->detachObject(object_id, "robotiq_85_base_link");
            task_->add(std::move(stage));
        }

        // 8. Retreat
        auto stage_retreat = std::make_unique<mtc::stages::MoveTo>("retreat from " + object_id, joint_solver);
        stage_retreat->setGroup("manipulator");
        stage_retreat->setGoal(waypoints_.armGoal(seq.retreat));
        task_->add(std::move(stage_retreat));
    }

    try {
        RCLCPP_INFO(LOGGER, "开始MTC任务规划，最多生成5个解...");
        task_->plan(5);
    } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "MTC初始化阶段异常: " << e);
        RCLCPP_ERROR(LOGGER, "请检查: 1) move_group是否运行 2) 机器人模型是否正确 3) 规划组名称是否正确");
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "规划时发生异常: %s", e.what());
        return false;
    }

    RCLCPP_INFO(LOGGER, "规划完成，找到 %zu 个解决方案", task_->numSolutions());

    if (task_->numSolutions() == 0) {
        RCLCPP_ERROR(LOGGER, "规划失败，未找到可行解。");
        RCLCPP_ERROR(LOGGER, "可能原因:");
        RCLCPP_ERROR(LOGGER, "  1. 路点配置导致碰撞");
        RCLCPP_ERROR(LOGGER, "  2. 关节角度超出限制");
        RCLCPP_ERROR(LOGGER, "  3. 规划组配置错误");

        // 打印任务阶段信息用于调试
        RCLCPP_ERROR(LOGGER, "=== 详细阶段信息 ===");

        // 使用 stringstream 捕获输出
        std::stringstream ss;
        task_->printState(ss);
        RCLCPP_ERROR_STREAM(LOGGER, ss.str());

        // 打印每个阶段的失败原因
        ss.str("");
        ss.clear();
        task_->explainFailure(ss);
        RCLCPP_ERROR(LOGGER, "=== 分析各阶段失败原因 ===");
        RCLCPP_ERROR_STREAM(LOGGER, ss.str());

        return false;
    }

    RCLCPP_INFO(LOGGER, "成功找到 %zu 个可行解！", task_->numSolutions());
    return true;
}

bool DismantleTask::execute() {
    if (!task_ || task_->numSolutions() == 0) {
        RCLCPP_ERROR(LOGGER, "无法执行，没有可用的规划方案");
        return false;
    }

    RCLCPP_INFO(LOGGER, "开始执行任务...");
    RCLCPP_INFO(LOGGER, "执行第1个解决方案（共%zu个）", task_->numSolutions());

    auto result = task_->execute(*task_->solutions().front());

    RCLCPP_INFO(LOGGER, "执行返回，结果代码: %d", result.val);

    if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        RCLCPP_INFO(LOGGER, "✓ 任务执行成功！");
        return true;
    } else if (result.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED) {
        RCLCPP_ERROR(LOGGER, "✗ 任务被抢占");
    } else if (result.val == moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT) {
        RCLCPP_ERROR(LOGGER, "✗ 任务执行超时");
    } else if (result.val == moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED) {
        RCLCPP_ERROR(LOGGER, "✗ 控制失败");
    } else {
        RCLCPP_ERROR(LOGGER, "✗ 任务执行失败，错误代码: %d", result.val);
    }
    return false;
}

void DismantleTask::spawnBatteryAndSubparts() {
    RCLCPP_INFO(LOGGER, "正在加载场景物体（从 waypoints 配置）...");

    // 1. 添加电池主体
    if (!waypoints_.scene.battery_id.empty()) {
        moveit_msgs::msg::CollisionObject battery_co;
        battery_co.header.frame_id = waypoints_.scene.battery_frame;
        battery_co.id = waypoints_.scene.battery_id;
        battery_co.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive battery_primitive;
        battery_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        battery_primitive.dimensions.resize(3);
        battery_primitive.dimensions[0] = waypoints_.scene.battery_size[0];
        battery_primitive.dimensions[1] = waypoints_.scene.battery_size[1];
        battery_primitive.dimensions[2] = waypoints_.scene.battery_size[2];

        battery_co.primitives.push_back(battery_primitive);
        battery_co.primitive_poses.push_back(waypoints_.scene.battery_pose);

        psi_.applyCollisionObject(battery_co);
        RCLCPP_INFO(LOGGER, "  ✓ 已添加电池主体: %s", waypoints_.scene.battery_id.c_str());
    }

    // 2. 添加所有配置的部件
    for (const auto& part : waypoints_.scene.parts) {
        moveit_msgs::msg::CollisionObject part_co;
        part_co.header.frame_id = part.frame;
        part_co.id = part.id;
        part_co.operation = moveit_msgs::msg::CollisionObject::ADD;

        shape_msgs::msg::SolidPrimitive part_primitive;

        // 根据类型设置形状
        if (part.type == "box") {
            part_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            part_primitive.dimensions.resize(3);
            part_primitive.dimensions[0] = part.size.size() > 0 ? part.size[0] : 0.1;
            part_primitive.dimensions[1] = part.size.size() > 1 ? part.size[1] : 0.1;
            part_primitive.dimensions[2] = part.size.size() > 2 ? part.size[2] : 0.1;
        } else if (part.type == "cylinder") {
            part_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
            part_primitive.dimensions.resize(2);
            part_primitive.dimensions[0] = part.size.size() > 0 ? part.size[0] : 0.1;  // height
            part_primitive.dimensions[1] = part.size.size() > 1 ? part.size[1] : 0.05; // radius
        } else {
            RCLCPP_WARN(LOGGER, "  ⚠️  未知的形状类型: %s, 使用默认 box", part.type.c_str());
            part_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
            part_primitive.dimensions.resize(3, 0.1);
        }

        part_co.primitives.push_back(part_primitive);
        part_co.primitive_poses.push_back(part.pose);

        psi_.applyCollisionObject(part_co);
        RCLCPP_INFO(LOGGER, "  ✓ 已添加部件: %s (%s)", part.id.c_str(), part.type.c_str());
    }

    RCLCPP_INFO(LOGGER, "✅ 场景加载完成！");
}

bool DismantleTask::waitWorldContains(const std::vector<std::string>& ids, double timeout_s) {
    auto start_time = node_->get_clock()->now();
    while ((node_->get_clock()->now() - start_time).seconds() < timeout_s) {
        auto object_map = psi_.getObjects(ids);
        bool all_found = true;
        for (const auto& id : ids) {
            if (object_map.find(id) == object_map.end()) {
                all_found = false;
                break;
            }
        }
        if (all_found) return true;
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
    return false;
}
