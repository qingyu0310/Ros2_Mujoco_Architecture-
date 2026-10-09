/**
 * @file shooter.cpp
 * @author qingyu
 * @brief 本项目的发射（摩擦轮）节点入口：直接用 ros2 层的 ShooterNode
 * @version 0.1
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这里只有入口。节点本体在 ros2_layer/node/shooter/shooter.hpp。
 *       参数文件 params/shooter.yaml 自己从 install 里找（同 keyboard.cpp 的套路）
 */

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/shooter/shooter.hpp"

int main(int argc, char** argv)
{
    // 参数文件走命令行（--ros-args --params-file），不走 NodeOptions::arguments（rcl 会静默忽略）
    std::vector<std::string> arg_strings(argv, argv + argc);
    const std::string params_file = ament_index_cpp::get_package_share_directory("project") + "/params/shooter.yaml";

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

    rclcpp::spin(std::make_shared<ShooterNode>());

    rclcpp::shutdown();

    return 0;
}
