#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <battery_dismantle_task/task_utils.h>

namespace mtc = moveit::task_constructor;

class DismantleTask {
public:
    DismantleTask(const std::string& task_name, const rclcpp::Node::SharedPtr& node);

    bool init(const Waypoints& waypoints, const BatteryConfig& battery_config, const std::vector<std::string>& target_objects);

    bool plan();

    bool execute();

private:
    std::string task_name_;
    rclcpp::Node::SharedPtr node_;
    mtc::TaskPtr task_;
    Waypoints waypoints_;
    BatteryConfig battery_config_;
    std::vector<std::string> target_objects_;
    moveit::planning_interface::PlanningSceneInterface psi_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    moveit::core::RobotModelConstPtr robot_model_;

    void setupPlanningScene();
    bool addBoxCOWait(const std::string& id, const std::string& frame,
                      const std::array<double,3>& size_xyz,
                      const geometry_msgs::msg::Pose& center_pose);
    bool waitWorldContains(const std::vector<std::string>& ids, double timeout_s = 5.0);
    void spawnBatteryAndSubparts();
    bool planAndExecForObject_JSON(const std::string& object_id);
};