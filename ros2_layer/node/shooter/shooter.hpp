/**
 * @file shooter.hpp
 * @author qingyu
 * @brief 摩擦轮：鼠标左键长按启用，两只 M3508 反向速度闭环
 * @version 0.1
 * @date 2026-10-09
 *
 * @copyright Copyright (c) 2026
 *
 * @note 左键长按给速度目标，两只电机反向转，沿枪管 +X 送弹；松开或输入/反馈断流
 *       撤掉力矩。速度环每轮一个 PID，共用一组增益和力矩限幅。
 */

#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/controller/pid.hpp"
#include "framework/msg/keyboard_state.hpp"

/**
 * @brief 摩擦轮节点：鼠标左键长按启用，两只电机反向速度闭环
 */
class ShooterNode : public rclcpp::Node
{
public:
    explicit ShooterNode(const std::string& node_name = "shooter", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        read_parameters();
        configure_pid();
        setup_ros_interfaces();
        start_control_timer();

        RCLCPP_INFO(get_logger(), "摩擦轮就绪：左键长按启动，目标 %.1f rad/s，松开撤掉力矩", control_params_.speed);
    }

private:
    using Pid = algorithm::controller::Pid<double>;

    /**
     * @brief 话题名：订阅和发布的 ROS 接口
     */
    struct TopicParams
    {
        std::string keyboard     {"/keyboard"};
        std::string joint_states {"/joint_states"};
        std::string motor_prefix {"/motor"};
    };

    /**
     * @brief 关节名：和 joint_states 里的名字对齐，左正右反
     */
    struct JointParams
    {
        std::array<std::string, 2> names {"pitch_barrel_friction_wheel_l_joint", "pitch_barrel_friction_wheel_r_joint"};
    };

    /**
     * @brief 速度环：目标转速、增益和力矩限幅
     */
    struct ControlParams
    {
        double speed {1650.0};         // 输出轴 rad/s，左正右负，沿枪管 +X 送弹
        double kp {0.02};
        double ki {0.1};
        double torque_limit {3.0};     // N·m，与模型 M3508 P19 力矩范围一致
    };

    /**
     * @brief 控制周期，以及输入和反馈断流超时
     */
    struct TimingParams
    {
        double control_period_s {0.001};
        double input_timeout_s {0.2};  // 输入和反馈断流后撤掉力矩，s
    };

    /**
     * @brief 单个摩擦轮的闭环状态
     */
    struct WheelState
    {
        Pid pid {};
        double velocity {0.0};
        bool valid {false};
        std::chrono::steady_clock::time_point received {};
    };

    /**
     * @brief 左键长按输入和到达时间
     */
    struct InputState
    {
        bool held {false};
        std::chrono::steady_clock::time_point received {};
    };

    TopicParams   topic_params_   {};
    JointParams   joint_params_   {};
    ControlParams control_params_ {};
    TimingParams  timing_params_  {};

    InputState    input_ {};
    std::array<WheelState, 2> wheels_ {};

    /**
     * @brief 节点订阅、发布和控制定时器
     */
    struct RosInterfaces
    {
        std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 2> wheel_cmd_pubs;

        rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr      keyboard_sub;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub;

        rclcpp::TimerBase::SharedPtr timer;
    };

    RosInterfaces ros_ {};

    /**
     * @brief 从 ROS 参数读入上面四组参数
     */
    void read_parameters()
    {
        timing_params_.control_period_s = declare_parameter("control_period_s",       0.001);
        timing_params_.input_timeout_s  = declare_parameter("input_state_timeout_s",  0.2);

        control_params_.speed           = declare_parameter("target_speed_rad_s",     1650.0);
        control_params_.kp              = declare_parameter("speed_kp",               0.02);
        control_params_.ki              = declare_parameter("speed_ki",               0.1);
        control_params_.torque_limit    = declare_parameter("torque_limit",           3.0);

        topic_params_.keyboard          = declare_parameter("keyboard_topic",         "/keyboard");
        topic_params_.joint_states      = declare_parameter("joint_states_topic",     "/joint_states");
        topic_params_.motor_prefix      = declare_parameter("motor_topic_prefix",     "/motor");

        joint_params_.names[0]          = declare_parameter("left_joint",             "pitch_barrel_friction_wheel_l_joint");
        joint_params_.names[1]          = declare_parameter("right_joint",            "pitch_barrel_friction_wheel_r_joint");
    }

    /**
     * @brief 检查目标转速非负，超时和力矩幅值为正且有限
     */
    void validate_parameters() const
    {
        if (!std::isfinite(control_params_.speed)          ||   control_params_.speed          < 0.0   || 
            !std::isfinite(timing_params_.input_timeout_s) || !(timing_params_.input_timeout_s > 0.0)  || 
            !std::isfinite(control_params_.torque_limit)   || !(control_params_.torque_limit   > 0.0))
        {
            throw std::invalid_argument("摩擦轮转速必须非负，超时和力矩幅值必须为正且有限");
        }
    }

    /**
     * @brief 从控制参数组配好两轮共用的速度环
     */
    void configure_pid()
    {
        validate_parameters();

        Pid::Params params;

        params.kp           = control_params_.kp;
        params.ki           = control_params_.ki;
        params.dt           = timing_params_.control_period_s;
        params.output_limit = control_params_.torque_limit;

        for (auto& wheel : wheels_)
        {
            wheel.pid.configure(params);
        }
    }

    /**
     * @brief 订阅键盘和关节，按电机名各建一个力矩发布者
     */
    void setup_ros_interfaces()
    {
        ros_.wheel_cmd_pubs[0] = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_prefix + "/friction_wheel_l/cmd_force", 10);
        ros_.wheel_cmd_pubs[1] = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_prefix + "/friction_wheel_r/cmd_force", 10);

        ros_.keyboard_sub = create_subscription<framework::msg::KeyboardState>(
            topic_params_.keyboard, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg)
            {
                input_.held = msg->mouse_left;
                input_.received = std::chrono::steady_clock::now();
            });

        ros_.joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
            topic_params_.joint_states, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_states(*msg); });
    }

    /**
     * @brief 按控制周期起控制定时器
     */
    void start_control_timer()
    {
        ros_.timer = create_wall_timer(std::chrono::duration<double>(timing_params_.control_period_s), [this]() { control_tick(); });
    }

    /**
     * @brief 按关节名取两只摩擦轮的速度，各自标记是否有效
     *
     * @param msg 关节状态消息
     */
    void on_joint_states(const sensor_msgs::msg::JointState& msg)
    {
        const auto now = std::chrono::steady_clock::now();
        
        for (std::size_t w = 0; w < wheels_.size(); ++w)
        {
            for (std::size_t i = 0; i < msg.name.size(); ++i)
            {
                if (msg.name[i] != joint_params_.names[w]) continue;
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

    /**
     * @brief 定时器里跑的一拍：左键和两轮反馈都新鲜才闭环，否则撤掉力矩
     */
    void control_tick()
    {
        const auto now = std::chrono::steady_clock::now();
        bool active = input_.held && std::chrono::duration<double>(now - input_.received).count() <= timing_params_.input_timeout_s;
        
        for (const auto& wheel : wheels_)
        {
            active = active && wheel.valid && std::chrono::duration<double>(now - wheel.received).count() <= timing_params_.input_timeout_s;
        }

        for (std::size_t i = 0; i < wheels_.size(); ++i)
        {
            std_msgs::msg::Float64 command;
            if (active)
            {
                command.data = wheels_[i].pid.update(wheels_[i].velocity, i == 0 ? control_params_.speed : -control_params_.speed);
            }
            else
            {
                wheels_[i].pid.reset(wheels_[i].velocity);
                command.data = 0.0;
            }
            ros_.wheel_cmd_pubs[i]->publish(command);
        }
    }
};
