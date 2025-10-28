#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

using nlohmann::json;

struct ScenePart {
    std::string id;
    std::string frame;
    std::string type;  // "box", "cylinder", etc.
    std::vector<double> size;
    geometry_msgs::msg::Pose pose;
};

struct BatteryConfig {
  std::string id   = "BatteryBox_0";
  std::string frame= "base_link";
  double size[3]   = {0.35, 0.25, 0.08}; // L W H (与publish_scene.py保持一致)
  geometry_msgs::msg::Pose pose;         // center pose
  geometry_msgs::msg::Pose place_pose;   // 预留
};

struct Waypoints {
    struct Defaults {
        double velocity_scaling = 0.2;
        double accel_scaling = 0.2;
    };

    struct GripperHooks {
        std::string on_approach;
        std::string after_approach;
        std::string after_place;
    };

    struct IO {
        bool do_attach_after_approach = false;
        bool do_detach_after_place = false;
    };

    struct ObjSeq {
        std::vector<double> approach, place, retreat;
        GripperHooks gripper_hooks;
        IO io;
    };

    std::vector<std::string> arm_joints;
    std::vector<std::string> gripper_joints;
    std::unordered_map<std::string, std::vector<double>> poses;
    Defaults defaults;
    std::unordered_map<std::string, ObjSeq> objects;

    // Scene configuration
    struct {
        std::string battery_id;
        std::string battery_frame;
        std::vector<double> battery_size;
        geometry_msgs::msg::Pose battery_pose;
        std::vector<ScenePart> parts;
    } scene;

    bool loadFromFile(const std::string& file) {
        std::ifstream f(file);
        if (!f.is_open()) {
            RCLCPP_ERROR(rclcpp::get_logger("task_utils"), "Could not open waypoints file: %s", file.c_str());
            return false;
        }

        try {
            json data = json::parse(f);

            // Load joints
            if (data.contains("joints")) {
                if (data["joints"].contains("arm"))
                    arm_joints = data["joints"]["arm"].get<std::vector<std::string>>();
                if (data["joints"].contains("gripper"))
                    gripper_joints = data["joints"]["gripper"].get<std::vector<std::string>>();
            }

            // Load default scaling
            if (data.contains("defaults")) {
                defaults.velocity_scaling = data["defaults"].value("velocity_scaling", 0.2);
                defaults.accel_scaling = data["defaults"].value("accel_scaling", 0.2);
            }

            // Load poses
            if (data.contains("poses")) {
                for (auto const& [key, val] : data["poses"].items()) {
                    poses[key] = val.get<std::vector<double>>();
                }
            }

            // Load objects
            if (data.contains("objects")) {
                for (auto const& [obj_name, obj_data] : data["objects"].items()) {
                    ObjSeq seq;
                    if (obj_data.contains("approach"))
                        seq.approach = obj_data["approach"].get<std::vector<double>>();
                    if (obj_data.contains("place"))
                        seq.place = obj_data["place"].get<std::vector<double>>();
                    if (obj_data.contains("retreat"))
                        seq.retreat = obj_data["retreat"].get<std::vector<double>>();

                    if (obj_data.contains("gripper_hooks")) {
                        seq.gripper_hooks.on_approach = obj_data["gripper_hooks"].value("on_approach", "");
                        seq.gripper_hooks.after_approach = obj_data["gripper_hooks"].value("after_approach", "");
                        seq.gripper_hooks.after_place = obj_data["gripper_hooks"].value("after_place", "");
                    }

                    if (obj_data.contains("io")) {
                        seq.io.do_attach_after_approach = obj_data["io"].value("do_attach_after_approach", false);
                        seq.io.do_detach_after_place = obj_data["io"].value("do_detach_after_place", false);
                    }
                    objects[obj_name] = seq;
                }
            }

            // Load scene configuration
            if (data.contains("scene")) {
                auto scene_data = data["scene"];

                // Load battery config
                if (scene_data.contains("battery")) {
                    auto battery = scene_data["battery"];
                    scene.battery_id = battery.value("id", "BatteryBox");
                    scene.battery_frame = battery.value("frame", "world");
                    scene.battery_size = battery.value("size", std::vector<double>{0.3, 0.2, 0.15});

                    if (battery.contains("pose")) {
                        auto pose_data = battery["pose"];
                        scene.battery_pose.position.x = pose_data["position"].value("x", 0.0);
                        scene.battery_pose.position.y = pose_data["position"].value("y", 0.0);
                        scene.battery_pose.position.z = pose_data["position"].value("z", 0.0);
                        scene.battery_pose.orientation.x = pose_data["orientation"].value("x", 0.0);
                        scene.battery_pose.orientation.y = pose_data["orientation"].value("y", 0.0);
                        scene.battery_pose.orientation.z = pose_data["orientation"].value("z", 0.0);
                        scene.battery_pose.orientation.w = pose_data["orientation"].value("w", 1.0);
                    }
                }

                // Load parts
                if (scene_data.contains("parts")) {
                    for (auto const& part_data : scene_data["parts"]) {
                        ScenePart part;
                        part.id = part_data.value("id", "");
                        part.frame = part_data.value("frame", "world");
                        part.type = part_data.value("type", "box");
                        part.size = part_data.value("size", std::vector<double>{0.1, 0.1, 0.1});

                        if (part_data.contains("pose")) {
                            auto pose_data = part_data["pose"];
                            part.pose.position.x = pose_data["position"].value("x", 0.0);
                            part.pose.position.y = pose_data["position"].value("y", 0.0);
                            part.pose.position.z = pose_data["position"].value("z", 0.0);
                            part.pose.orientation.x = pose_data["orientation"].value("x", 0.0);
                            part.pose.orientation.y = pose_data["orientation"].value("y", 0.0);
                            part.pose.orientation.z = pose_data["orientation"].value("z", 0.0);
                            part.pose.orientation.w = pose_data["orientation"].value("w", 1.0);
                        }

                        scene.parts.push_back(part);
                    }
                }
            }

            return true;

        } catch (json::parse_error& e) {
            RCLCPP_ERROR(rclcpp::get_logger("task_utils"), "Failed to parse waypoints file: %s, error: %s", file.c_str(), e.what());
            return false;
        }
        return false;
    }

    std::map<std::string, double> armGoal(const std::vector<double>& q) const {
        std::map<std::string, double> goal;
        for (size_t i = 0; i < arm_joints.size() && i < q.size(); ++i) {
            goal[arm_joints[i]] = q[i];
        }
        return goal;
    }

    std::map<std::string, double> gripperGoal(const std::vector<double>& values) const {
        std::map<std::string, double> goal;
        for (size_t i = 0; i < gripper_joints.size() && i < values.size(); ++i) {
            goal[gripper_joints[i]] = values[i];
        }
        return goal;
    }
};