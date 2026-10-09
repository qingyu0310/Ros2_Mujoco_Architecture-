/**
 * @file gimbal.cpp
 * @author qingyu
 * @brief 云台入口：同一进程里启动 yaw、pitch 和摩擦轮节点
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 */

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/gimbal/pitch.hpp"
#include "ros2_layer/node/gimbal/friction_wheel.hpp"
#include "ros2_layer/node/gimbal/yaw.hpp"

int main(int argc, char** argv)
{
    std::vector<std::string> arg_strings(argv, argv + argc);
    const std::string share_dir = ament_index_cpp::get_package_share_directory("project");
    const std::string params_file = share_dir + "/params/gimbal.yaml";

    if (std::filesystem::exists(params_file))
    {
        arg_strings.push_back("--ros-args");
        arg_strings.push_back("--params-file");
        arg_strings.push_back(params_file);
    }

    std::vector<char const*> args;
    args.reserve(arg_strings.size());
    for (const auto& arg : arg_strings)
    {
        args.push_back(arg.c_str());
    }

    rclcpp::init(static_cast<int>(args.size()), args.data());

    rclcpp::executors::MultiThreadedExecutor executor;
    auto yaw_node = std::make_shared<YawNode>();
    auto pitch_node = std::make_shared<PitchNode>();
    auto friction_wheel_node = std::make_shared<FrictionWheelNode>();
    RCLCPP_INFO(rclcpp::get_logger("gimbal"), "云台入口已启动：内部节点 /yaw、/pitch 和 /friction_wheel 已接入 executor");
    executor.add_node(yaw_node);
    executor.add_node(pitch_node);
    executor.add_node(friction_wheel_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
