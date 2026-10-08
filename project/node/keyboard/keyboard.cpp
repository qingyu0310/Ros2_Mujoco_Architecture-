/**
 * @file keyboard.cpp
 * @author qingyu
 * @brief 本项目的第一人称输入节点入口：直接用 ros2 层的 KeyboardNode，不重复写 GLFW 输入逻辑
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这里只有入口。节点本体（参数、定时器、发布）在 ros2_layer 包的
 *       ros2_layer/node/keyboard/keyboard.hpp，跟本项目的机器无关，所以放通用层。
 * @note 发到哪条话题、重发周期、是否捕获鼠标，都在 params/keyboard.yaml（install 里那份，
 *       下面自己找，找不到就用代码默认值）。临时改加 --ros-args -p，优先级更高
 * @note 发的是 framework/msg/KeyboardState（兼容旧字段，同时带 26 键、按下沿、toggle 和鼠标 dx/dy），
 *       不是速度：哪个键算前进、给多快、正负号，全由具体控制节点自己解释。
 * @note 入口组合键盘节点与摄像头节点，共用一个 Teleop 画面窗口，焦点内读取键鼠。
 */

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/keyboard/keyboard.hpp"
#include "ros2_layer/node/camera/camera.hpp"

int main(int argc, char** argv)
{
    // 参数文件是**全局**参数，只能走命令行：塞进 NodeOptions::arguments 的话 rcl 不认，
    // 会一声不吭地忽略掉（节点照跑，参数全是代码默认值）
    std::vector<std::string> arg_strings(argv, argv + argc);
    const std::string params_file = ament_index_cpp::get_package_share_directory("project") + "/params/keyboard.yaml";

    if (std::filesystem::exists(params_file))
    {
        arg_strings.push_back("--ros-args");
        arg_strings.push_back("--params-file");
        arg_strings.push_back(params_file);
    }

    const std::string camera_params_file = ament_index_cpp::get_package_share_directory("project") + "/params/camera.yaml";
    if (std::filesystem::exists(camera_params_file))
    {
        arg_strings.push_back("--ros-args");
        arg_strings.push_back("--params-file");
        arg_strings.push_back(camera_params_file);
    }

    std::vector<char const*> args;
    args.reserve(arg_strings.size());

    for (const auto& arg : arg_strings)
    {
        args.push_back(arg.c_str());
    }

    rclcpp::init(static_cast<int>(args.size()), args.data());

    // 两个节点共用一个窗口：相机负责显示，键盘只借用句柄读取输入。
    // 单线程执行器保证 GLFW 操作在主线程；相机先创建、后销毁。
    auto camera_node = std::make_shared<CameraNode>("gimbal_camera");
    auto keyboard_node = std::make_shared<KeyboardNode>("keyboard", rclcpp::NodeOptions(), camera_node->window());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(camera_node);
    executor.add_node(keyboard_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
