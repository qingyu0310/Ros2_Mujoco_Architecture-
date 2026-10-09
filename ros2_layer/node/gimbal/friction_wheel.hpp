/**
 * @file friction_wheel.hpp
 * @author qingyu
 * @brief 摩擦轮：鼠标左键长按启用，两只 M3508 反向速度闭环
 * @date 2026-10-09
 */
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "framework/msg/keyboard_state.hpp"
#include "framework/algorithm/controller/pid.hpp"

class FrictionWheelNode : public rclcpp::Node
{
public:
    explicit FrictionWheelNode(const std::string& name = "friction_wheel", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(name, options)
    {
        read_parameters();
        for (std::size_t i = 0; i < wheels_.size(); ++i)
        {
            wheels_[i].pid.configure(params_.pid);
            ros_.commands[i] = create_publisher<std_msgs::msg::Float64>(params_.motor_prefix + "/" + params_.motors[i] + "/cmd_force", 10);
        }
        ros_.keyboard = create_subscription<framework::msg::KeyboardState>(params_.keyboard, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg)
            {
                input_.held = msg->mouse_left;
                input_.received = std::chrono::steady_clock::now();
            });
        ros_.joints = create_subscription<sensor_msgs::msg::JointState>(params_.joint_states, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_states(*msg); });
        ros_.timer = create_wall_timer(std::chrono::duration<double>(params_.pid.dt), [this]() { control_tick(); });
        RCLCPP_INFO(get_logger(), "摩擦轮就绪：左键长按启动，目标 %.1f rad/s，松开撤掉力矩", params_.speed);
    }

private:
    using Pid = algorithm::controller::Pid<double>;

    struct Params
    {
        double speed {1650.0};  // 输出轴 rad/s，左正右负，沿枪管 +X 送弹
        double timeout {0.2}; // 输入和反馈断流后撤掉力矩，s
        Pid::Params pid {};
        std::array<std::string, 2> joints {"pitch_barrel_friction_wheel_l_joint", "pitch_barrel_friction_wheel_r_joint"};
        std::array<std::string, 2> motors {"friction_wheel_l", "friction_wheel_r"};
        std::string keyboard {"/keyboard"};
        std::string joint_states {"/joint_states"};
        std::string motor_prefix {"/motor"};
    };

    struct WheelState
    {
        Pid pid {};
        double velocity {0.0};
        bool valid {false};
        std::chrono::steady_clock::time_point received {};
    };

    struct InputState
    {
        bool held {false};
        std::chrono::steady_clock::time_point received {};
    };

    struct RosInterfaces
    {
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> commands;
        rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr keyboard;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints;
        rclcpp::TimerBase::SharedPtr timer;
    };

    Params params_ {};
    std::array<WheelState, 2> wheels_ {};
    InputState input_ {};
    RosInterfaces ros_ {};

    void read_parameters()
    {
        params_.speed = declare_parameter("target_speed_rad_s", 1650.0);
        params_.timeout = declare_parameter("input_state_timeout_s", 0.2);
        params_.pid.dt = declare_parameter("control_period_s", 0.001);
        params_.pid.kp = declare_parameter("speed_kp", 0.02);
        params_.pid.ki = declare_parameter("speed_ki", 0.1);
        params_.pid.output_limit = declare_parameter("torque_limit", 3.0);
        params_.keyboard = declare_parameter("keyboard_topic", params_.keyboard);
        params_.joint_states = declare_parameter("joint_states_topic", params_.joint_states);
        params_.motor_prefix = declare_parameter("motor_topic_prefix", params_.motor_prefix);
        params_.joints[0] = declare_parameter("left_joint", params_.joints[0]);
        params_.joints[1] = declare_parameter("right_joint", params_.joints[1]);
        if (!std::isfinite(params_.speed) || params_.speed < 0.0 || !std::isfinite(params_.timeout) || params_.timeout <= 0.0
            || !std::isfinite(params_.pid.output_limit) || params_.pid.output_limit <= 0.0)
        {
            throw std::invalid_argument("摩擦轮转速必须非负，超时和力矩幅值必须为正且有限");
        }
    }

    void on_joint_states(const sensor_msgs::msg::JointState& msg)
    {
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t w = 0; w < wheels_.size(); ++w)
        {
            for (std::size_t i = 0; i < msg.name.size(); ++i)
            {
                if (msg.name[i] != params_.joints[w]) continue;
                wheels_[w].valid = i < msg.velocity.size() && std::isfinite(msg.velocity[i]);
                if (wheels_[w].valid)
                {
                    wheels_[w].velocity = msg.velocity[i];
                    wheels_[w].received = now;
                }
                break;
            }
        }
    }

    void control_tick()
    {
        const auto now = std::chrono::steady_clock::now();
        bool active = input_.held && std::chrono::duration<double>(now - input_.received).count() <= params_.timeout;
        for (const auto& wheel : wheels_)
        {
            active = active && wheel.valid && std::chrono::duration<double>(now - wheel.received).count() <= params_.timeout;
        }
        for (std::size_t i = 0; i < wheels_.size(); ++i)
        {
            std_msgs::msg::Float64 command;
            if (active)
            {
                command.data = wheels_[i].pid.update(wheels_[i].velocity, i == 0 ? params_.speed : -params_.speed);
            }
            else
            {
                wheels_[i].pid.reset(wheels_[i].velocity);
                command.data = 0.0;
            }
            ros_.commands[i]->publish(command);
        }
    }
};
