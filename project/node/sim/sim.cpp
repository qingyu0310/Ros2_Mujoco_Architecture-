/**
 * @file sim.cpp
 * @author qingyu
 * @brief 本项目的仿真节点入口：MuJoCo 跑物理开界面，顶替原来 Gazebo 的位置
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这里只有入口和线程的分工。节点本体在 ros2_layer 的 ros2_layer/node/sim/sim.hpp，
 *       跟哪台机器人无关，所以放通用层；那边是纯头文件，不产二进制
 * @note 线程布局照抄 MuJoCo 自己的 simulate/main.cc：界面（主线程，阻塞） + 物理（另一个线程），
 *       两边靠 Simulate::mtx 串起来
 * @note 模型从 model 参数给：MuJoCo 没有运行时 spawn，世界和机器人只能在解析期拼成一份，
 *       拼的工作在 launch 里做
 */

#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ros2_layer/node/sim/sim.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 必须在主线程构造：界面（GLFW）的调用都得跟 RenderLoop 同一条线程
    auto node = std::make_shared<SimNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    // 物理线程：推物理、收力矩、发关节状态
    std::thread physics([&node, &executor] { node->physics_loop(executor); });

    // 主线交给界面，阻塞到窗口关掉
    node->render_loop();

    physics.join();

    rclcpp::shutdown();

    return 0;
}
