#include <rclcpp/rclcpp.hpp>
#include <battery_dismantle_task/DismantleTask.h>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("dismantle_node");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "dismantle_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Spin the node in a separate thread
    std::thread spin_thread([node] { rclcpp::spin(node); });

    // Get the path to the waypoints file (已经由 automatically_declare_parameters_from_overrides 自动声明)
    std::string waypoints_path;
    if (!node->get_parameter("waypoints_path", waypoints_path) || waypoints_path.empty()) {
        RCLCPP_ERROR(LOGGER, "Please provide the 'waypoints_path' parameter!");
        rclcpp::shutdown();
        return 1;
    }

    // Load waypoints
    Waypoints waypoints;
    if (!waypoints.loadFromFile(waypoints_path)) {
        RCLCPP_ERROR(LOGGER, "Failed to load waypoints.");
        rclcpp::shutdown();
        return 1;
    }

    // Define battery config (can be loaded from params later)
    BatteryConfig battery_config;
    battery_config.pose.position.x = 0.5;
    battery_config.pose.position.y = 0.0;
    battery_config.pose.position.z = 0.05;
    battery_config.pose.orientation.w = 1.0;

    // Define target objects - 先测试TopCoverBolts完整流程
    std::vector<std::string> target_objects = {"TopCoverBolts"};

    // Create and run the task
    DismantleTask task("Battery Dismantle Task", node);
    if (task.init(waypoints, battery_config, target_objects)) {
        if (task.plan()) {
            RCLCPP_INFO(LOGGER, "✓ 规划成功！准备执行...");

            // Wait for controllers to be fully ready
            RCLCPP_INFO(LOGGER, "等待控制器 action servers 完全启动...");
            rclcpp::sleep_for(std::chrono::seconds(5));
            RCLCPP_INFO(LOGGER, "继续执行任务");

            bool success = task.execute();
            if (success) {
                RCLCPP_INFO(LOGGER, "✓✓✓ 执行完全成功！✓✓✓");
            } else {
                RCLCPP_ERROR(LOGGER, "✗✗✗ 执行失败 ✗✗✗");
            }
        } else {
            RCLCPP_ERROR(LOGGER, "Task planning failed.");
        }
    } else {
        RCLCPP_ERROR(LOGGER, "Task initialization failed.");
    }

    RCLCPP_INFO(LOGGER, "Node will shut down in 10 seconds.");
    rclcpp::sleep_for(std::chrono::seconds(10));

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}