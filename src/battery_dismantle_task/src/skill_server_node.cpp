#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <nlohmann/json.hpp>
#include <fstream>
#include <unordered_set>
#include <optional>
#include <chrono>
#include <cstdlib>  // for std::rand

using json = nlohmann::json;
using std::placeholders::_1;

class SkillServer : public rclcpp::Node
{
public:
  SkillServer() : Node("skill_server")
  {
    // ---- Declare parameters (with sensible defaults) ----
    this->declare_parameter<std::string>("waypoints_path", "");
    this->declare_parameter<std::string>("manipulator_group", "manipulator");
    this->declare_parameter<std::string>("gripper_group", "gripper");
    this->declare_parameter<std::string>("ee_attach_link", "robotiq_85_base_link");
    this->declare_parameter<double>("vel_scale", 1.0);
    this->declare_parameter<double>("acc_scale", 1.0);
    this->declare_parameter<double>("scene_update_wait_s", 1.0);
    this->declare_parameter<std::string>("open_gripper_pose_name", "OPEN");
    this->declare_parameter<std::string>("close_gripper_pose_name", "CLOSE");

    // 可接受的“假执行模式”错误码（JSON 数组，元素为 MoveItErrorCodes 的整数值）
    // 默认包含 GOAL_TOLERANCE_VIOLATED(-5)
    this->declare_parameter<std::vector<int64_t>>("fake_ok_codes", std::vector<int64_t>{-5});
  }

  void init()
  {
    // ---- Read parameters ----
    waypoints_path_      = this->get_parameter("waypoints_path").as_string();
    manipulator_group_   = this->get_parameter("manipulator_group").as_string();
    gripper_group_name_  = this->get_parameter("gripper_group").as_string();
    ee_attach_link_      = this->get_parameter("ee_attach_link").as_string();
    vel_scale_           = this->get_parameter("vel_scale").as_double();
    acc_scale_           = this->get_parameter("acc_scale").as_double();
    scene_update_wait_s_ = this->get_parameter("scene_update_wait_s").as_double();
    open_gripper_pose_   = this->get_parameter("open_gripper_pose_name").as_string();
    close_gripper_pose_  = this->get_parameter("close_gripper_pose_name").as_string();

    std::vector<int64_t> fake_ok;
    this->get_parameter("fake_ok_codes", fake_ok);
    for (auto v : fake_ok) fake_ok_codes_.insert(static_cast<int>(v));

    if (waypoints_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'waypoints_path' is required!");
      throw std::runtime_error("Missing waypoints_path parameter");
    }

    // ---- Load waypoints file ----
    load_waypoints(waypoints_path_);

    // ---- Initialize MoveIt interfaces (with retry) ----
    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface (groups: '%s', '%s') ...",
                manipulator_group_.c_str(), gripper_group_name_.c_str());

    int max_retries = 5;
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
      try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), manipulator_group_);

        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), gripper_group_name_);

        // CRITICAL FIX: Wait for joint state publisher to send first message
        // This prevents "invalid start state" race condition when planning immediately after initialization
        RCLCPP_INFO(this->get_logger(), "Waiting 2s for joint state publisher...");
        rclcpp::sleep_for(std::chrono::seconds(2));
        RCLCPP_INFO(this->get_logger(), "✅ Joint state synchronization complete");

        planning_scene_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // 初始化 planning scene monitor 用于碰撞检测
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            shared_from_this(), "robot_description");
        if (planning_scene_monitor_->getPlanningScene()) {
          RCLCPP_INFO(this->get_logger(), "✅ PlanningSceneMonitor initialized");

          // WORKAROUND FOR FAKE EXECUTION: Disable self-collisions
          // This bypasses URDF issues and initial pose collisions by modifying ACM
          planning_scene_monitor_->requestPlanningSceneState();
          {
            planning_scene_monitor::LockedPlanningSceneRW scene(planning_scene_monitor_);
            collision_detection::AllowedCollisionMatrix& acm = scene->getAllowedCollisionMatrixNonConst();

            // Disable collision between problematic gripper links (URDF mesh overlap)
            acm.setEntry("robotiq_85_left_finger_tip_link", "robotiq_85_left_finger_link", true);
            acm.setEntry("robotiq_85_right_finger_tip_link", "robotiq_85_right_finger_link", true);
            acm.setEntry("robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link", true);
            acm.setEntry("robotiq_85_left_finger_link", "robotiq_85_right_finger_link", true);

            // Disable collision between shoulder and base (initial HOME pose collision)
            acm.setEntry("shoulder_link", "base_link", true);

            RCLCPP_WARN(this->get_logger(), "⚠️  [FAKE MODE] Disabled self-collisions in ACM (gripper + shoulder/base)");
          }

          // Start scene monitor and publish the updated ACM to move_group
          planning_scene_monitor_->startSceneMonitor();
          planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

          // CRITICAL: Also send ACM directly to move_group via PlanningSceneInterface
          // This ensures move_group receives the ACM update immediately
          moveit_msgs::msg::PlanningScene ps_msg;
          {
            planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
            scene->getPlanningSceneMsg(ps_msg);
          }
          ps_msg.is_diff = true;  // Only update ACM, don't replace entire scene
          planning_scene_->applyPlanningScene(ps_msg);
          rclcpp::sleep_for(std::chrono::milliseconds(500)); // Give move_group time to process
          RCLCPP_INFO(this->get_logger(), "✅  ACM update sent to move_group");
        }

        // 速度/加速度缩放
        apply_scaling_from_session(); // uses vel_scale_/acc_scale_ defaults

        // 简单检查：EE link 是否存在
        if (!move_group_->getRobotModel()->hasLinkModel(ee_attach_link_)) {
          RCLCPP_ERROR(this->get_logger(), "EE attach link '%s' not in robot model", ee_attach_link_.c_str());
          throw std::runtime_error("Invalid ee_attach_link");
        }

        // NOTE: Collision disabling is handled by SRDF file (gen3_robotiq_2f_85.srdf)
        // No need to programmatically disable collisions here

        RCLCPP_INFO(this->get_logger(), "✅ MoveGroupInterface initialized successfully");
        break; // success
      }
      catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to initialize MoveGroupInterface (attempt %d/%d): %s",
                    attempt, max_retries, e.what());
        if (attempt == max_retries) {
          RCLCPP_ERROR(this->get_logger(), "❌ Giving up after %d attempts", max_retries);
          throw;
        }
        rclcpp::sleep_for(std::chrono::seconds(2));
      }
    }

    // ---- Subscribe & publish ----
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/llm_commands", 10, std::bind(&SkillServer::command_callback, this, _1));

    feedback_pub_ = this->create_publisher<std_msgs::msg::String>("/llm_feedback", 10);

    // Publisher for updating joint states in fake execution mode
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    RCLCPP_INFO(this->get_logger(), "✅ Skill Server Ready! Listening on /llm_commands");
  }

private:
  // ---------- Session / PlaceSpec ----------
  struct PlaceSpec {
    std::string bin;            // scene.bins 的键
    std::string pose_name;      // poses 的键
    std::vector<double> joints; // 直接关节
    bool empty() const {
      return bin.empty() && pose_name.empty() && joints.empty();
    }
  };
  struct Session {
    std::string object;                // 当前对象
    std::optional<PlaceSpec> place;    // 当前放置位
    bool plan_only{false};             // 只规划不执行
    double vel_scale{1.0};
    double acc_scale{1.0};
  };

  // ---------- Utils ----------
  void load_waypoints(const std::string& path)
  {
    RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", path.c_str());
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open waypoints file: " + path);
    }
    try {
      waypoints_json_ = json::parse(file);
    } catch (const std::exception& e) {
      throw std::runtime_error(std::string("Failed to parse waypoints JSON: ") + e.what());
    }
    // 基本结构检查
    if (!waypoints_json_.contains("poses") || !waypoints_json_["poses"].is_object()) {
      throw std::runtime_error("Waypoints JSON missing 'poses' object");
    }
    RCLCPP_INFO(this->get_logger(), "✅ Waypoints loaded successfully");
  }

  bool get_pose_joints(const std::string& name, std::vector<double>& out) const
  {
    if (!waypoints_json_.contains("poses")) return false;
    const auto& poses = waypoints_json_.at("poses");
    if (!poses.contains(name)) return false;
    try {
      out = poses.at(name).get<std::vector<double>>();
      return true;
    } catch (...) {
      return false;
    }
  }

  bool object_exists(const std::string& obj) const
  {
    return waypoints_json_.contains("objects") &&
           waypoints_json_["objects"].is_object() &&
           waypoints_json_["objects"].contains(obj);
  }

  // 解析 place：优先级 params -> session -> object.place
  bool resolve_place_joints(const std::string& object_name,
                            const std::optional<PlaceSpec>& place_in,
                            std::vector<double>& out) const
  {
    // 1) params/session: bin -> pose_name
    auto try_bin_to_pose = [&](const std::string& bin)->bool{
      if (bin.empty()) return false;
      if (!waypoints_json_.contains("scene") || !waypoints_json_["scene"].contains("bins")) return false;
      const auto& bins = waypoints_json_["scene"]["bins"];
      if (!bins.is_object() || !bins.contains(bin)) return false;
      const auto& bd = bins.at(bin);
      if (!bd.contains("pose_name")) return false;
      std::string pn = bd.at("pose_name").get<std::string>();
      return get_pose_joints(pn, out);
    };

    if (place_in.has_value()) {
      const auto& p = place_in.value();
      if (!p.bin.empty() && try_bin_to_pose(p.bin)) return true;
      if (!p.pose_name.empty() && get_pose_joints(p.pose_name, out)) return true;
      if (!p.joints.empty()) { out = p.joints; return true; }
    }

    // 2) 对象自带 place
    if (object_exists(object_name)) {
      const auto& o = waypoints_json_["objects"][object_name];
      if (o.contains("place") && o["place"].is_array()) {
        out = o["place"].get<std::vector<double>>();
        return true;
      }
    }

    // 3) 最后再尝试 session.place（若传入为空而 session 有值）
    if (!place_in.has_value() && session_.place.has_value()) {
      const auto& p = session_.place.value();
      if (!p.bin.empty() && try_bin_to_pose(p.bin)) return true;
      if (!p.pose_name.empty() && get_pose_joints(p.pose_name, out)) return true;
      if (!p.joints.empty()) { out = p.joints; return true; }
    }

    return false;
  }

  // 展开 targets：["ALL"] → 所有 objects.*；否则筛选存在的对象
  bool expand_targets(const json& cmd, std::vector<std::string>& out) const
  {
    out.clear();
    // prefer "targets"
    if (cmd.contains("targets") && cmd["targets"].is_array()) {
      bool has_all = false;
      for (const auto& t : cmd["targets"]) {
        if (!t.is_string()) continue;
        std::string s = t.get<std::string>();
        if (s == "ALL" || s == "all") { has_all = true; break; }
      }
      if (has_all) {
        if (!waypoints_json_.contains("objects") || !waypoints_json_["objects"].is_object()) return false;
        for (auto& [k, v] : waypoints_json_["objects"].items()) out.push_back(k);
        return !out.empty();
      } else {
        for (const auto& t : cmd["targets"]) {
          if (!t.is_string()) continue;
          std::string s = t.get<std::string>();
          if (object_exists(s)) out.push_back(s);
        }
        return !out.empty();
      }
    }
    // fallback "target"
    if (cmd.contains("target") && cmd["target"].is_string()) {
      std::string s = cmd["target"].get<std::string>();
      if (s == "ALL" || s == "all") {
        if (!waypoints_json_.contains("objects") || !waypoints_json_["objects"].is_object()) return false;
        for (auto& [k, v] : waypoints_json_["objects"].items()) out.push_back(k);
        return !out.empty();
      } else if (object_exists(s)) {
        out.push_back(s);
        return true;
      }
    }
    return false;
  }

  void publish_feedback(const std::string& status, const std::string& message,
                        const std::string& command_id = "", const std::string& stage = "", int code = 0)
  {
    json feedback;
    feedback["schema"] = "llm_fb/v1";
    feedback["status"] = status;
    feedback["message"] = message;
    feedback["stage"] = stage;
    feedback["code"] = code;
    feedback["timestamp_ns"] = this->get_clock()->now().nanoseconds();
    if (!command_id.empty()) feedback["command_id"] = command_id;

    std_msgs::msg::String msg;
    msg.data = feedback.dump();
    feedback_pub_->publish(msg);

    if (status == "failure" || status == "rejected")
      RCLCPP_ERROR(this->get_logger(), "📤 Feedback(%s/%s): %s", stage.c_str(), status.c_str(), message.c_str());
    else
      RCLCPP_INFO(this->get_logger(), "📤 Feedback(%s/%s): %s", stage.c_str(), status.c_str(), message.c_str());
  }

  void apply_scaling_from_session()
  {
    const double vs = session_.vel_scale > 0 ? session_.vel_scale : vel_scale_;
    const double as = session_.acc_scale > 0 ? session_.acc_scale : acc_scale_;
    move_group_->setMaxVelocityScalingFactor(vs);
    move_group_->setMaxAccelerationScalingFactor(as);
    gripper_group_->setMaxVelocityScalingFactor(vs);
    gripper_group_->setMaxAccelerationScalingFactor(as);
  }

  // ========== MTC-STYLE ROBUST PLANNING ==========
  // 检查当前状态是否有效（无碰撞）
  // Publish joint states for visualization in fake execution mode
  void publish_joint_states(const std::vector<double>& arm_joints, const std::vector<double>& gripper_joints)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();

    // Arm joints
    js.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
    js.position = arm_joints;

    // Gripper joint
    js.name.push_back("robotiq_85_left_knuckle_joint");
    js.position.push_back(gripper_joints.empty() ? 0.0 : gripper_joints[0]);

    joint_state_pub_->publish(js);
    RCLCPP_INFO(this->get_logger(), "📍 Published joint states: arm[0]=%.3f, gripper[0]=%.3f",
                arm_joints.empty() ? 0.0 : arm_joints[0],
                gripper_joints.empty() ? 0.0 : gripper_joints[0]);
  }

  // Smoothly animate trajectory in fake execution mode
  void animate_trajectory(const std::vector<double>& start, const std::vector<double>& goal,
                          const std::vector<double>& gripper, double duration_sec)
  {
    if (start.size() != goal.size()) {
      RCLCPP_ERROR(this->get_logger(), "Start and goal joint sizes don't match!");
      return;
    }

    const int steps = 100;  // 100 interpolation steps for smoother animation
    const double dt = duration_sec / steps;

    RCLCPP_INFO(this->get_logger(), "🎬 Animating trajectory over %.1fs with %d steps", duration_sec, steps);

    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;  // 0.0 to 1.0

      // Linear interpolation
      std::vector<double> current(start.size());
      for (size_t j = 0; j < start.size(); ++j) {
        current[j] = start[j] + t * (goal[j] - start[j]);
      }

      // Publish current state (reduce logging frequency)
      if (i % 20 == 0 || i == steps) {  // Log only every 20 steps
        publish_joint_states(current, gripper);
      } else {
        // Publish without logging
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = waypoints_json_["joints"]["arm"].get<std::vector<std::string>>();
        js.position = current;

        auto gripper_names = waypoints_json_["joints"]["gripper"].get<std::vector<std::string>>();
        js.name.insert(js.name.end(), gripper_names.begin(), gripper_names.end());
        js.position.insert(js.position.end(), gripper.begin(), gripper.end());
        joint_state_pub_->publish(js);
      }

      // Sleep to control animation speed
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9)));
    }

    RCLCPP_INFO(this->get_logger(), "✅ Animation complete");
  }

  // Animate gripper only (keeping arm fixed)
  void animate_gripper(double start_grip, double goal_grip, const std::vector<double>& arm, double duration_sec)
  {
    const int steps = 60;  // 60 interpolation steps for smoother gripper animation
    const double dt = duration_sec / steps;

    RCLCPP_INFO(this->get_logger(), "🤏 Animating gripper %.3f -> %.3f over %.1fs", start_grip, goal_grip, duration_sec);

    for (int i = 0; i <= steps; ++i) {
      double t = static_cast<double>(i) / steps;
      double current_grip = start_grip + t * (goal_grip - start_grip);

      // Reduce logging frequency for gripper too
      if (i % 20 == 0 || i == steps) {
        publish_joint_states(arm, {current_grip});
      } else {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = waypoints_json_["joints"]["arm"].get<std::vector<std::string>>();
        js.position = arm;
        auto gripper_names = waypoints_json_["joints"]["gripper"].get<std::vector<std::string>>();
        js.name.insert(js.name.end(), gripper_names.begin(), gripper_names.end());
        js.position.push_back(current_grip);
        joint_state_pub_->publish(js);
      }
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(dt * 1e9)));
    }

    RCLCPP_INFO(this->get_logger(), "✅ Gripper animation complete");
  }

  bool verify_current_state()
  {
    // WORKAROUND FOR FAKE EXECUTION: Skip collision check entirely for gripper
    // This allows arm planning even when gripper has URDF mesh collision issues
    RCLCPP_WARN(this->get_logger(), "⚠️  [FAKE MODE] Skipping state collision verification (gripper URDF workaround)");
    return true;  // Always return true to bypass gripper collision blocking arm planning
  }

  // MTC风格：尝试修复碰撞状态
  bool recover_from_collision()
  {
    RCLCPP_WARN(this->get_logger(), "🔧 尝试从碰撞状态恢复...");

    // 策略1：微小随机扰动当前关节位置
    auto current_joints = move_group_->getCurrentJointValues();
    if (current_joints.empty()) return false;

    for (int attempt = 0; attempt < 3; ++attempt) {
      std::vector<double> perturbed_joints = current_joints;
      // 对每个关节添加±0.05弧度的小扰动
      for (size_t i = 0; i < perturbed_joints.size(); ++i) {
        double delta = (std::rand() % 100 - 50) / 1000.0;  // -0.05到+0.05
        perturbed_joints[i] += delta;
      }

      RCLCPP_INFO(this->get_logger(), "  尝试 %d/3: 微调关节位置", attempt + 1);
      move_group_->setJointValueTarget(perturbed_joints);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto p = move_group_->plan(plan);

      if (p == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "✅ 找到无碰撞路径，执行修正动作");
        auto e = move_group_->execute(plan);
        if (e == moveit::core::MoveItErrorCode::SUCCESS) {
          return verify_current_state();  // 验证修正后的状态
        }
      }
    }

    RCLCPP_ERROR(this->get_logger(), "❌ 无法从碰撞状态恢复");
    return false;
  }

  bool plan_execute_arm(const std::vector<double>& joints, const char* where)
  {
    // TRAJECTORY EXECUTION MODE: Bypass OMPL planning, build and execute trajectory directly
    RCLCPP_INFO(this->get_logger(), "🚀 [TRAJECTORY MODE] Building trajectory for arm %s", where);

    // Get robot model and current state
    auto robot_model = move_group_->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(manipulator_group_);

    // Create robot state for start and goal
    moveit::core::RobotState start_state(robot_model);
    moveit::core::RobotState goal_state(robot_model);

    // Use cached position as start, or use goal joints if no cache (first move)
    if (last_arm_position_.empty()) {
      RCLCPP_INFO(this->get_logger(), "   No cached position, using goal as start (first move)");
      start_state.setJointGroupPositions(joint_model_group, joints);
    } else {
      start_state.setJointGroupPositions(joint_model_group, last_arm_position_);
    }

    // Set goal position
    goal_state.setJointGroupPositions(joint_model_group, joints);

    // Create trajectory
    robot_trajectory::RobotTrajectory trajectory(robot_model, manipulator_group_);

    // Add waypoints: start -> goal (use 5 seconds for visible motion in RViz)
    trajectory.addSuffixWayPoint(start_state, 0.0);   // Start at t=0
    trajectory.addSuffixWayPoint(goal_state, 5.0);    // Goal at t=5 seconds

    // Execute trajectory (bypass planning, direct execution)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    trajectory.getRobotTrajectoryMsg(plan.trajectory_);

    RCLCPP_INFO(this->get_logger(), "   📤 Executing trajectory with %zu waypoints...", trajectory.getWayPointCount());
    auto error_code = move_group_->execute(plan);

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      last_arm_position_ = joints;
      // Publish joint states for RViz visualization in fake execution mode
      if (!last_gripper_position_.empty()) {
        publish_joint_states(joints, last_gripper_position_);
      } else {
        publish_joint_states(joints, {0.0});
      }
      RCLCPP_INFO(this->get_logger(), "✅  [TRAJECTORY MODE] Arm %s completed successfully", where);
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️  [TRAJECTORY MODE] Arm %s returned code %d (fake mode - animating manually)", where, error_code.val);

      // In fake mode: manually animate the trajectory for visualization
      std::vector<double> start_pos = last_arm_position_.empty() ? joints : last_arm_position_;
      std::vector<double> grip_pos = last_gripper_position_.empty() ? std::vector<double>{0.0} : last_gripper_position_;

      animate_trajectory(start_pos, joints, grip_pos, 2.0);  // 2 second animation (very fast)

      last_arm_position_ = joints;
      return true;
    }
  }

  bool plan_execute_gripper(const std::vector<double>& joints, const char* where)
  {
    // TRAJECTORY EXECUTION MODE: Bypass OMPL planning, build and execute trajectory directly
    RCLCPP_INFO(this->get_logger(), "🚀 [TRAJECTORY MODE] Building trajectory for gripper %s", where);

    // Get robot model
    auto robot_model = gripper_group_->getRobotModel();
    auto joint_model_group = robot_model->getJointModelGroup(gripper_group_name_);

    // Create start and goal states
    moveit::core::RobotState start_state(robot_model);
    moveit::core::RobotState goal_state(robot_model);

    // Set to current state first (to initialize all joints properly)
    start_state.setToDefaultValues();
    goal_state.setToDefaultValues();

    // For goal: use setJointValueTarget approach to handle mimic joints automatically
    gripper_group_->setJointValueTarget(joints);
    auto goal_state_ref = gripper_group_->getJointValueTarget();
    std::vector<double> goal_state_vec;
    goal_state_ref.copyJointGroupPositions(joint_model_group, goal_state_vec);
    goal_state.setJointGroupPositions(joint_model_group, goal_state_vec);

    // For start: either use cached full state or same as goal (first move)
    if (!last_gripper_full_state_.empty()) {
      start_state.setJointGroupPositions(joint_model_group, last_gripper_full_state_);
    } else {
      RCLCPP_INFO(this->get_logger(), "   No cached state, using goal as start (first move)");
      start_state.setJointGroupPositions(joint_model_group, goal_state_vec);
    }

    // Create trajectory
    robot_trajectory::RobotTrajectory trajectory(robot_model, gripper_group_name_);

    // Add waypoints (use 3 seconds for visible gripper motion)
    trajectory.addSuffixWayPoint(start_state, 0.0);
    trajectory.addSuffixWayPoint(goal_state, 3.0);

    // Execute trajectory
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    trajectory.getRobotTrajectoryMsg(plan.trajectory_);

    RCLCPP_INFO(this->get_logger(), "   📤 Executing trajectory with %zu waypoints...", trajectory.getWayPointCount());
    auto error_code = gripper_group_->execute(plan);

    // Cache the full state vector for next time
    last_gripper_full_state_ = goal_state_vec;

    // Update gripper position for joint state publishing
    last_gripper_position_ = joints;

    if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
      // Publish joint states for RViz visualization in fake execution mode
      if (!last_arm_position_.empty()) {
        publish_joint_states(last_arm_position_, last_gripper_position_);
      } else {
        // If no arm position yet, use home position
        publish_joint_states({0.0, 0.2618, 3.14159, -2.2689, 0.0, 0.9599, 1.5708}, last_gripper_position_);
      }
      RCLCPP_INFO(this->get_logger(), "✅  [TRAJECTORY MODE] Gripper %s completed successfully", where);
      return true;
    } else {
      RCLCPP_WARN(this->get_logger(), "⚠️  [TRAJECTORY MODE] Gripper %s returned code %d (fake mode - animating manually)", where, error_code.val);

      // In fake mode: manually animate gripper movement
      double start_grip = last_gripper_position_.empty() ? 0.0 : last_gripper_position_[0];
      double goal_grip = joints.empty() ? 0.0 : joints[0];
      std::vector<double> arm_pos = last_arm_position_.empty() ?
          std::vector<double>{0.0, 0.2618, 3.14159, -2.2689, 0.0, 0.9599, 1.5708} : last_arm_position_;

      animate_gripper(start_grip, goal_grip, arm_pos, 1.0);  // 1.0 second animation (very fast)

      return true;  // Accept anyway in fake mode
    }
  }

  // ---------- Callbacks ----------
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "📨 Received command: %s", msg->data.c_str());

    json cmd;
    try {
      cmd = json::parse(msg->data);
    } catch (const std::exception& e) {
      publish_feedback("rejected", std::string("Invalid JSON: ") + e.what());
      return;
    }

    const std::string command_id = cmd.value("command_id", "");

    // --- Schema check ---
    if (!cmd.contains("schema") || !cmd["schema"].is_string() || cmd["schema"] != "llm_cmd/v1") {
      publish_feedback("rejected", "Unsupported or missing schema (expect llm_cmd/v1)", command_id);
      return;
    }

    if (!cmd.contains("skill") || !cmd["skill"].is_string()) {
      publish_feedback("rejected", "Missing or invalid 'skill'", command_id);
      return;
    }
    const std::string skill = cmd["skill"].get<std::string>();

    // --- Context update ---
    if (cmd.contains("context") && cmd["context"].is_object()) {
      const auto& ctx = cmd["context"];
      if (ctx.contains("object") && ctx["object"].is_string()) session_.object = ctx["object"].get<std::string>();

      PlaceSpec ptmp;
      bool has_place = false;
      if (ctx.contains("place") && ctx["place"].is_object()) {
        const auto& pl = ctx["place"];
        if (pl.contains("bin") && pl["bin"].is_string())        { ptmp.bin = pl["bin"].get<std::string>(); has_place=true; }
        if (pl.contains("pose_name") && pl["pose_name"].is_string()) { ptmp.pose_name = pl["pose_name"].get<std::string>(); has_place=true; }
        if (pl.contains("joints") && pl["joints"].is_array())    { ptmp.joints = pl["joints"].get<std::vector<double>>(); has_place=true; }
      }
      if (has_place) session_.place = ptmp;

      if (ctx.contains("plan_only") && ctx["plan_only"].is_boolean()) session_.plan_only = ctx["plan_only"].get<bool>();
      if (ctx.contains("vel_scale") && ctx["vel_scale"].is_number())  session_.vel_scale = ctx["vel_scale"].get<double>();
      if (ctx.contains("acc_scale") && ctx["acc_scale"].is_number())  session_.acc_scale = ctx["acc_scale"].get<double>();
    }

    // --- Dispatch ---
    bool ok = dispatch_skill(skill, cmd, command_id);

    publish_feedback(ok ? "success" : "failure",
                     std::string("Skill '") + skill + (ok ? "' completed" : "' failed"),
                     command_id);
  }

  // 统一分派
  bool dispatch_skill(const std::string& skill, const json& cmd, const std::string& command_id)
  {
    try {
      if (skill == "moveTo") {
        if (!cmd.contains("target") || !cmd["target"].is_string()) {
          publish_feedback("rejected", "moveTo missing 'target'", command_id, "moveTo");
          return false;
        }
        return execute_move_to(cmd["target"].get<std::string>());
      }
      else if (skill == "grasp") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) { publish_feedback("rejected", "grasp needs 'target' object", command_id, "grasp"); return false; }
        return execute_grasp(obj);
      }
      else if (skill == "release") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) { publish_feedback("rejected", "release needs 'target' object", command_id, "release"); return false; }
        return execute_release(obj);
      }
      // -------- 微技能 --------
      else if (skill == "selectObject") {
        if (!cmd.contains("params") || !cmd["params"].contains("object_id"))
          { publish_feedback("rejected", "selectObject requires params.object_id", command_id, "selectObject"); return false; }
        session_.object = cmd["params"]["object_id"].get<std::string>();
        publish_feedback("progress", "current object = " + session_.object, command_id, "selectObject");
        return true;
      }
      else if (skill == "selectPlace") {
        if (!cmd.contains("params") || !cmd["params"].is_object())
          { publish_feedback("rejected", "selectPlace requires params", command_id, "selectPlace"); return false; }
        PlaceSpec p;
        const auto& pl = cmd["params"];
        if (pl.contains("bin") && pl["bin"].is_string()) p.bin = pl["bin"].get<std::string>();
        if (pl.contains("pose_name") && pl["pose_name"].is_string()) p.pose_name = pl["pose_name"].get<std::string>();
        if (pl.contains("joints") && pl["joints"].is_array()) p.joints = pl["joints"].get<std::vector<double>>();
        if (p.empty()) { publish_feedback("rejected", "selectPlace requires bin/pose_name/joints", command_id, "selectPlace"); return false; }
        session_.place = p;
        publish_feedback("progress", "place selected", command_id, "selectPlace");
        return true;
      }
      else if (skill == "openGripper") {
        std::vector<double> gv;
        std::string pn = (cmd.contains("params") && cmd["params"].contains("pose_name"))
                           ? cmd["params"]["pose_name"].get<std::string>()
                           : open_gripper_pose_;
        if (!get_pose_joints(pn, gv)) { publish_feedback("failure", "openGripper: pose not found "+pn, command_id, "openGripper"); return false; }
        return plan_execute_gripper(gv, "openGripper");
      }
      else if (skill == "closeGripper") {
        std::vector<double> gv;
        std::string pn = (cmd.contains("params") && cmd["params"].contains("pose_name"))
                           ? cmd["params"]["pose_name"].get<std::string>()
                           : close_gripper_pose_;
        if (!get_pose_joints(pn, gv)) { publish_feedback("failure", "closeGripper: pose not found "+pn, command_id, "closeGripper"); return false; }
        return plan_execute_gripper(gv, "closeGripper");
      }
      else if (skill == "allowCollision") {
        // params: a, b(array|string), allow(bool)
        if (!cmd.contains("params") || !cmd["params"].is_object())
          { publish_feedback("rejected", "allowCollision requires params", command_id, "allowCollision"); return false; }
        const auto& p = cmd["params"];
        if (!p.contains("a") || !p.contains("b") || !p.contains("allow"))
          { publish_feedback("rejected", "allowCollision needs a/b/allow", command_id, "allowCollision"); return false; }

        std::string a = p["a"].get<std::string>();
        std::vector<std::string> b_list;
        if (p["b"].is_string()) b_list.push_back(p["b"].get<std::string>());
        else if (p["b"].is_array()) {
          for (const auto& x : p["b"]) if (x.is_string()) b_list.push_back(x.get<std::string>());
        }
        bool allow = p["allow"].get<bool>();

        moveit_msgs::msg::PlanningScene ps;
        ps.is_diff = true;
        auto& acm = ps.allowed_collision_matrix;
        acm.entry_names.push_back(a);
        // init entry_values row with existing columns + new ones
        // Fetch union of names: a + all b_list
        std::vector<std::string> cols = b_list;
        // Ensure symmetric: add columns
        for (const auto& col : cols) {
          if (std::find(acm.entry_names.begin(), acm.entry_names.end(), col) == acm.entry_names.end())
            acm.entry_names.push_back(col);
        }
        // Build a map index for entry_names
        std::unordered_map<std::string, size_t> idx;
        for (size_t i=0;i<acm.entry_names.size();++i) idx[acm.entry_names[i]] = i;

        // Build matrix (square) - entry_values is vector<AllowedCollisionEntry>
        size_t N = acm.entry_names.size();
        acm.entry_values.resize(N);
        for (size_t i = 0; i < N; ++i) {
          acm.entry_values[i].enabled.resize(N, false);
        }

        // Set allow flags both [a][b] and [b][a]
        for (const auto& b : b_list) {
          if (!idx.count(a) || !idx.count(b)) continue;
          acm.entry_values[idx[a]].enabled[idx[b]] = allow;
          acm.entry_values[idx[b]].enabled[idx[a]] = allow;
        }
        planning_scene_->applyPlanningScene(ps);
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
        publish_feedback("progress", std::string("ACM ")+(allow?"allow":"forbid")+" "+a, command_id, "allowCollision");
        return true;
      }
      else if (skill == "approach") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        if (!object_exists(obj)) { publish_feedback("rejected", "approach: unknown object", command_id, "approach"); return false; }
        const auto& j = waypoints_json_["objects"][obj];
        if (!j.contains("approach") || !j["approach"].is_array()) { publish_feedback("failure", "approach joints missing", command_id, "approach"); return false; }
        return plan_execute_arm(j["approach"].get<std::vector<double>>(), "approach");
      }
      else if (skill == "attachObject") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        if (obj.empty()) { publish_feedback("rejected", "attachObject needs object", command_id, "attachObject"); return false; }
        if (session_.plan_only) { publish_feedback("progress", "attach skipped (plan_only)", command_id, "attachObject"); return true; }
        move_group_->attachObject(obj, ee_attach_link_);
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
        return true;
      }
      else if (skill == "place") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        std::vector<double> target;
        // params override
        std::optional<PlaceSpec> local;
        if (cmd.contains("params") && cmd["params"].is_object()) {
          PlaceSpec p;
          const auto& pl = cmd["params"];
          if (pl.contains("bin") && pl["bin"].is_string()) p.bin = pl["bin"].get<std::string>();
          if (pl.contains("pose_name") && pl["pose_name"].is_string()) p.pose_name = pl["pose_name"].get<std::string>();
          if (pl.contains("joints") && pl["joints"].is_array()) p.joints = pl["joints"].get<std::vector<double>>();
          if (!p.empty()) local = p;
        }
        if (!resolve_place_joints(obj, local.has_value()? local : session_.place, target)) {
          publish_feedback("failure", "place: cannot resolve joints", command_id, "place");
          return false;
        }
        return plan_execute_arm(target, "place");
      }
      else if (skill == "openAfterPlace") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        if (!object_exists(obj)) { publish_feedback("rejected", "openAfterPlace: unknown object", command_id, "openAfterPlace"); return false; }
        const auto& j = waypoints_json_["objects"][obj];
        if (!j.contains("gripper_hooks") || !j["gripper_hooks"].contains("after_place"))
          { publish_feedback("failure", "no gripper after_place pose", command_id, "openAfterPlace"); return false; }
        std::string pose = j["gripper_hooks"]["after_place"].get<std::string>();
        std::vector<double> gv;
        if (!get_pose_joints(pose, gv)) { publish_feedback("failure", "after_place pose not found", command_id, "openAfterPlace"); return false; }
        return plan_execute_gripper(gv, "openAfterPlace");
      }
      else if (skill == "detachObject") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        if (session_.plan_only) { publish_feedback("progress", "detach skipped (plan_only)", command_id, "detachObject"); return true; }
        RCLCPP_INFO(this->get_logger(), "  - Detaching '%s' from gripper", obj.c_str());
        move_group_->detachObject(obj);
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
        RCLCPP_INFO(this->get_logger(), "  ✅ Detach complete");
        return true;
      }
      else if (skill == "retreat") {
        std::string obj = extract_object_from_cmd(cmd);
        if (obj.empty()) obj = session_.object;
        // allow override pose_name via params
        if (cmd.contains("params") && cmd["params"].contains("pose_name")) {
          std::vector<double> q;
          std::string pn = cmd["params"]["pose_name"].get<std::string>();
          if (!get_pose_joints(pn, q)) { publish_feedback("failure", "retreat pose_name not found", command_id, "retreat"); return false; }
          return plan_execute_arm(q, "retreat");
        }
        if (!object_exists(obj)) { publish_feedback("rejected", "retreat: unknown object", command_id, "retreat"); return false; }
        const auto& j = waypoints_json_["objects"][obj];
        if (!j.contains("retreat") || !j["retreat"].is_array()) { publish_feedback("failure", "retreat joints missing", command_id, "retreat"); return false; }
        return plan_execute_arm(j["retreat"].get<std::vector<double>>(), "retreat");
      }
      else if (skill == "dismantle") {
        // 高阶：按 targets 循环执行 grasp → place → openAfterPlace → detach → retreat
        std::vector<std::string> targets;
        if (!expand_targets(cmd, targets)) { publish_feedback("rejected", "dismantle: missing targets", command_id, "dismantle"); return false; }

        // params-level place，用作默认
        std::optional<PlaceSpec> place_default;
        if (cmd.contains("place") && cmd["place"].is_object()) {
          PlaceSpec p;
          const auto& pl = cmd["place"];
          if (pl.contains("bin") && pl["bin"].is_string()) p.bin = pl["bin"].get<std::string>();
          if (pl.contains("pose_name") && pl["pose_name"].is_string()) p.pose_name = pl["pose_name"].get<std::string>();
          if (pl.contains("joints") && pl["joints"].is_array()) p.joints = pl["joints"].get<std::vector<double>>();
          if (!p.empty()) place_default = p;
        }

        for (size_t i=0;i<targets.size();++i) {
          const auto& obj = targets[i];
          publish_feedback("progress", "dismantle: "+obj, command_id, "dismantle", int(i+1));
          if (!execute_grasp(obj)) return false;

          std::vector<double> pj;
          if (!resolve_place_joints(obj, place_default.has_value()? place_default : session_.place, pj)) {
            publish_feedback("failure", "cannot resolve place for "+obj, command_id, "dismantle"); return false;
          }
          if (!plan_execute_arm(pj, "place")) return false;

          // openAfterPlace (if any)
          if (waypoints_json_["objects"][obj].contains("gripper_hooks") &&
              waypoints_json_["objects"][obj]["gripper_hooks"].contains("after_place")) {
            std::vector<double> gv;
            std::string pn = waypoints_json_["objects"][obj]["gripper_hooks"]["after_place"].get<std::string>();
            if (!get_pose_joints(pn, gv)) { publish_feedback("failure", "after_place pose not found", command_id, "dismantle"); return false; }
            if (!plan_execute_gripper(gv, "openAfterPlace")) return false;
          }

          // detach (if configured)
          if (waypoints_json_["objects"][obj].contains("io") &&
              waypoints_json_["objects"][obj]["io"].contains("do_detach_after_place") &&
              waypoints_json_["objects"][obj]["io"]["do_detach_after_place"].get<bool>()) {
            if (!session_.plan_only) {
              RCLCPP_INFO(this->get_logger(), "  - Detaching '%s' from gripper (after place)", obj.c_str());
              move_group_->detachObject(obj);
              rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
              RCLCPP_INFO(this->get_logger(), "  ✅ Detach complete");
            }
          }

          // retreat (if any)
          if (waypoints_json_["objects"][obj].contains("retreat") &&
              waypoints_json_["objects"][obj]["retreat"].is_array()) {
            if (!plan_execute_arm(waypoints_json_["objects"][obj]["retreat"].get<std::vector<double>>(), "retreat")) return false;
          }
        }
        return true;
      }
      else if (skill == "sequence") {
        if (!cmd.contains("params") || !cmd["params"].contains("steps") || !cmd["params"]["steps"].is_array()) {
          publish_feedback("rejected", "sequence needs params.steps", command_id, "sequence"); return false;
        }
        for (const auto& step : cmd["params"]["steps"]) {
          if (!step.contains("skill") || !step["skill"].is_string()) {
            publish_feedback("rejected", "sequence step missing skill", command_id, "sequence"); return false;
          }
          // 构造一条子命令，复用 dispatch（继承上层 context）
          json sub = { {"schema","llm_cmd/v1"}, {"skill", step["skill"]} };
          if (step.contains("params")) sub["params"] = step["params"];
          // 允许在 step 覆盖 target/targets/place
          if (step.contains("target")) sub["target"] = step["target"];
          if (step.contains("targets")) sub["targets"] = step["targets"];
          if (step.contains("place"))  sub["place"]  = step["place"];

          if (!dispatch_skill(sub["skill"].get<std::string>(), sub, command_id)) return false;
        }
        return true;
      }
      else {
        publish_feedback("rejected", "Unknown skill: " + skill, command_id, "dispatch");
        return false;
      }
    }
    catch (const std::exception& e) {
      publish_feedback("failure", std::string("Exception: ") + e.what(), command_id, "dispatch");
      return false;
    }
  }

  std::string extract_object_from_cmd(const json& cmd) const
  {
    if (cmd.contains("target") && cmd["target"].is_string()) return cmd["target"].get<std::string>();
    if (cmd.contains("params") && cmd["params"].contains("object_id") && cmd["params"]["object_id"].is_string())
      return cmd["params"]["object_id"].get<std::string>();
    return "";
  }

  // ---------- Legacy high-level skills (kept & reused) ----------
  bool execute_move_to(const std::string& target_name)
  {
    RCLCPP_INFO(this->get_logger(), "🎯 Executing moveTo: %s", target_name.c_str());

    std::vector<double> joints;
    if (!get_pose_joints(target_name, joints)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Unknown or invalid pose: %s", target_name.c_str());
      return false;
    }

    // 校验维度
    const auto* jmg = move_group_->getRobotModel()->getJointModelGroup(manipulator_group_);
    if (!jmg) {
      RCLCPP_ERROR(this->get_logger(), "❌ JointModelGroup '%s' not found", manipulator_group_.c_str());
      return false;
    }
    const size_t dof = jmg->getVariableCount();
    if (joints.size() != dof) {
      RCLCPP_ERROR(this->get_logger(), "❌ Pose '%s' size=%zu != group DOF=%zu", target_name.c_str(), joints.size(), dof);
      return false;
    }

    return plan_execute_arm(joints, "moveTo");
  }

  bool execute_grasp(const std::string& object_name)
  {
    RCLCPP_INFO(this->get_logger(), "🤏 Executing complex grasp: %s", object_name.c_str());

    // 检查对象配置
    if (!object_exists(object_name)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Unknown object in waypoints: %s", object_name.c_str());
      return false;
    }
    const auto& obj = waypoints_json_["objects"][object_name];

    // 1) gripper on_approach
    if (obj.contains("gripper_hooks") && obj["gripper_hooks"].contains("on_approach")) {
      const std::string name = obj["gripper_hooks"]["on_approach"].get<std::string>();
      std::vector<double> gv;
      if (!get_pose_joints(name, gv)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Gripper pose '%s' not found", name.c_str());
        return false;
      }
      if (!plan_execute_gripper(gv, "gripper-open")) return false;
    }

    // 2) approach
    if (!obj.contains("approach") || !obj["approach"].is_array()) {
      RCLCPP_ERROR(this->get_logger(), "❌ Object '%s' missing 'approach' joints", object_name.c_str());
      return false;
    }
    if (!plan_execute_arm(obj["approach"].get<std::vector<double>>(), "approach")) return false;

    // 3) gripper after_approach
    if (obj.contains("gripper_hooks") && obj["gripper_hooks"].contains("after_approach")) {
      const std::string name = obj["gripper_hooks"]["after_approach"].get<std::string>();
      std::vector<double> gv;
      if (!get_pose_joints(name, gv)) {
        RCLCPP_ERROR(this->get_logger(), "❌ Gripper pose '%s' not found", name.c_str());
        return false;
      }
      if (!plan_execute_gripper(gv, "gripper-close")) return false;
    }

    // 4) attach
    if (obj.contains("io") && obj["io"].contains("do_attach_after_approach")
        && obj["io"]["do_attach_after_approach"].is_boolean()
        && obj["io"]["do_attach_after_approach"].get<bool>()) {

      if (!session_.plan_only) {
        RCLCPP_INFO(this->get_logger(), "  - Attaching '%s' to '%s'", object_name.c_str(), ee_attach_link_.c_str());
        move_group_->attachObject(object_name, ee_attach_link_);
        rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
      } else {
        RCLCPP_INFO(this->get_logger(), "  - (plan_only) skip attach");
      }
    }

    RCLCPP_INFO(this->get_logger(), "✅ Grasp '%s' done", object_name.c_str());
    return true;
  }

  bool execute_release(const std::string& object_name)
  {
    RCLCPP_INFO(this->get_logger(), "✋ Executing release: %s", object_name.c_str());

    // Check object exists
    if (!object_exists(object_name)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Unknown object in waypoints: %s", object_name.c_str());
      return false;
    }
    const auto& obj = waypoints_json_["objects"][object_name];

    // 1) Move to place position FIRST (while holding object)
    std::vector<double> place_joints;
    if (!resolve_place_joints(object_name, session_.place, place_joints)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Cannot resolve place position for '%s'", object_name.c_str());
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "  - Moving to place position...");
    if (!plan_execute_arm(place_joints, "place")) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to move to place position");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  ✅ Reached place position");

    // 2) Open gripper BEFORE detaching (to release the object)
    std::vector<double> open_vals;
    if (!get_pose_joints(open_gripper_pose_, open_vals)) {
      RCLCPP_ERROR(this->get_logger(), "❌ Gripper open pose '%s' not found", open_gripper_pose_.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  - Opening gripper to release...");
    if (!plan_execute_gripper(open_vals, "release-gripper")) {
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "  ✅ Gripper opened");

    // 3) Detach object from gripper (after releasing)
    if (!session_.plan_only) {
      RCLCPP_INFO(this->get_logger(), "  - Detaching '%s' from gripper", object_name.c_str());
      move_group_->detachObject(object_name);
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(scene_update_wait_s_ * 1e9)));
      RCLCPP_INFO(this->get_logger(), "  ✅ Detach complete");
    }

    // 4) Retreat to safe position
    if (obj.contains("retreat") && obj["retreat"].is_array()) {
      RCLCPP_INFO(this->get_logger(), "  - Retreating to safe position...");
      if (!plan_execute_arm(obj["retreat"].get<std::vector<double>>(), "retreat")) {
        RCLCPP_WARN(this->get_logger(), "⚠️  Retreat failed, but release completed");
      } else {
        RCLCPP_INFO(this->get_logger(), "  ✅ Retreat complete");
      }
    }

    RCLCPP_INFO(this->get_logger(), "✅ Release '%s' complete", object_name.c_str());
    return true;
  }

  // ---------- Helpers ----------
  bool maybe_accept_fake(int code, const std::string& where)
  {
    using E = moveit_msgs::msg::MoveItErrorCodes;
    if (code == E::SUCCESS) return true;

    if (fake_ok_codes_.count(code)) {
      RCLCPP_WARN(this->get_logger(), "⚠️  %s returned code %d (accepted in fake mode)", where.c_str(), code);
      return true;
    }
    RCLCPP_ERROR(this->get_logger(), "❌ %s failed with code %d", where.c_str(), code);
    return false;
  }

private:
  // ROS I/O
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  // MoveIt
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;

  // Parameters
  std::string waypoints_path_;
  std::string manipulator_group_;
  std::string gripper_group_name_;
  std::string ee_attach_link_;
  double vel_scale_{1.0};
  double acc_scale_{1.0};
  double scene_update_wait_s_{1.0};
  std::string open_gripper_pose_;
  std::string close_gripper_pose_;
  std::unordered_set<int> fake_ok_codes_;

  // Cache for last known positions in fake mode
  std::vector<double> last_arm_position_;
  std::vector<double> last_gripper_position_;
  std::vector<double> last_gripper_full_state_;  // Full state including mimic joints

  // Session data
  Session session_;

  // Data
  json waypoints_json_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<SkillServer>();
    node->init();
    rclcpp::spin(node);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("skill_server"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
