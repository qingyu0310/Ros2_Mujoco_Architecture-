/**
 * @file pitch.hpp
 * @author qingyu
 * @brief pitch 俯仰节点：只管 pitch 目标积分和 pitch 电机力矩
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 *
 * @note yaw 和 pitch 分节点。这里不读大小 yaw，也不碰 yaw 分配律。
 *       鼠标 Y 轴只在这里积分成 pitch_des，控制器用 2 状态 LQR 输出 pitch 力矩。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/controller/lqr.hpp"
#include "framework/algorithm/math/angle.hpp"
#include "framework/msg/keyboard_state.hpp"

using namespace algorithm::math;

/**
 * @brief pitch 俯仰节点：鼠标 Y 轴积分成 pitch 目标，2 状态 LQR 出力矩
 */
class PitchNode : public rclcpp::Node
{
public:
    explicit PitchNode(const std::string& node_name = "pitch", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        read_parameters();
        pitch_des_ = std::clamp(normalize_radian_pm_pi(target_params_.initial), -std::abs(target_params_.soft_limit), std::abs(target_params_.soft_limit));
        configure_lqr();
        setup_ros_interfaces();
        start_control_timer();

        RCLCPP_INFO(get_logger(), "pitch 就绪：软约束 +/-%.1f deg，鼠标 Y 灵敏度 %.2f", target_params_.soft_limit * kRadToDeg, input_params_.mouse_sensitivity);
    }

private:
    static constexpr double kDegToRad = M_PI / 180.0;
    static constexpr double kRadToDeg = 180.0 / M_PI;

    using Lqr = algorithm::controller::Lqr<2, 1>;

    /**
     * @brief 话题名：订阅和发布的 ROS 接口
     */
    struct TopicParams
    {
        std::string keyboard {"/keyboard"};
        std::string joint_states {"/joint_states"};
        std::string motor_prefix {"/motor"};
    };

    /**
     * @brief 关节名：和 joint_states 里的名字对齐
     */
    struct JointParams
    {
        std::string pitch {"pitch_pitch_joint"};
    };

    /**
     * @brief 控制周期
     */
    struct TimingParams
    {
        double control_period_s {0.001};
    };

    /**
     * @brief 键盘输入映射
     */
    struct InputParams
    {
        double mouse_sensitivity {1.0};
    };

    /**
     * @brief 目标角：初值、限速和软限位
     */
    struct TargetParams
    {
        double initial {0.0};
        double rate_limit {M_PI / 2.0};
        double soft_limit {25.0 * kDegToRad};
    };

    /**
     * @brief 控制参数：力矩限幅、惯量和 LQR 权重
     */
    struct ControlParams
    {
        double ctrl_limit {7.0};
        double inertia {0.00035};
        double q_pos {20.0};
        double q_vel {1.0};
        double r {1.0};
    };

    /**
     * @brief 重力补偿：质量、质心位置和补偿符号
     */
    struct GravityCompParams
    {
        bool   enable {true};
        double mass   {0.09};
        double com_x  {0.065};
        double sign   {-1.0};
    };

    /**
     * @brief pitch 关节采样：角度和角速度
     */
    struct PitchSample
    {
        double q {0.0};
        double dq {0.0};
        double sim_time_s {0.0};
        bool valid {false};
    };

    Lqr lqr_ {};

    PitchSample pitch_ {};

    TimingParams      timing_params_  {};
    InputParams       input_params_   {};
    TargetParams      target_params_  {};
    ControlParams     control_params_ {};
    GravityCompParams gravity_params_ {};
    JointParams       joint_params_   {};
    TopicParams       topic_params_   {};

    double pitch_des_ {0.0};
    double pitch_input_ {0.0};

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pitch_cmd_pub_;
    
    rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr keyboard_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState> ::SharedPtr joint_states_sub_;

    /**
     * @brief 从 ROS 参数读入上面几组参数
     */
    void read_parameters()
    {
        timing_params_.control_period_s = declare_parameter("control_period_s",            0.001);

        input_params_.mouse_sensitivity = declare_parameter("mouse_pitch_sensitivity",     1.0);

        target_params_.initial          = declare_parameter("pitch_target_deg",            0.0)  * kDegToRad;
        target_params_.rate_limit       = declare_parameter("pitch_target_rate_limit_deg", 90.0) * kDegToRad;
        target_params_.soft_limit       = declare_parameter("pitch_soft_limit_deg",        25.0) * kDegToRad;

        control_params_.ctrl_limit      = declare_parameter("pitch_ctrl_limit",            7.0);
        control_params_.inertia         = declare_parameter("j_pitch",                     0.00035);
        control_params_.q_pos           = declare_parameter("pitch_q_pos",                 20.0);
        control_params_.q_vel           = declare_parameter("pitch_q_vel",                 1.0);
        control_params_.r               = declare_parameter("pitch_r",                     1.0);

        gravity_params_.enable          = declare_parameter("pitch_gravity_comp_enable",   true);
        gravity_params_.mass            = declare_parameter("pitch_gravity_comp_mass",     0.09);
        gravity_params_.com_x           = declare_parameter("pitch_gravity_comp_com_x",    0.065);
        gravity_params_.sign            = declare_parameter("pitch_gravity_comp_sign",     -1.0);

        joint_params_.pitch             = declare_parameter("pitch_joint",                 "pitch_pitch_joint");

        topic_params_.keyboard          = declare_parameter("keyboard_topic",              "/keyboard");
        topic_params_.joint_states      = declare_parameter("joint_states_topic",          "/joint_states");
        topic_params_.motor_prefix      = declare_parameter("motor_topic_prefix",          "/motor");
    }

    /**
     * @brief 按 pitch 惯量离散 A/B，配好 Q/R 后求解 LQR
     */
    void configure_lqr()
    {
        validate_parameters();

        Lqr::StateMatrix a = Lqr::StateMatrix::Identity();
        a(0, 1) = timing_params_.control_period_s;

        Lqr::InputMatrix b = Lqr::InputMatrix::Zero();
        b(1, 0) = timing_params_.control_period_s / control_params_.inertia;

        Lqr::StateMatrix q = Lqr::StateMatrix::Zero();
        q(0, 0) = control_params_.q_pos;
        q(1, 1) = control_params_.q_vel;

        Lqr::ControlMatrix r = Lqr::ControlMatrix::Zero();
        r(0, 0) = control_params_.r;

        lqr_.configure(a, b, q, r);
        lqr_.solve(500, 1e-10);
    }

    /**
     * @brief 检查控制周期、惯量和 R 是否为正
     */
    void validate_parameters() const
    {
        if (!(timing_params_.control_period_s > 0.0) || !(control_params_.inertia > 0.0) || !(control_params_.r > 0.0))
        {
            throw std::runtime_error("pitch control_period_s/j_pitch/pitch_r must be positive");
        }
    }

    /**
     * @brief 订阅关节和键盘，发布 pitch 力矩
     */
    void setup_ros_interfaces()
    {
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topic_params_.joint_states, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)  { on_joint_states(msg); });

        keyboard_sub_ = create_subscription<framework::msg::KeyboardState>(
            topic_params_.keyboard, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg) { on_keyboard(msg); });

        pitch_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_prefix + "/pitch/cmd_force", 10);
    }

    /**
     * @brief 按控制周期起控制定时器
     */
    void start_control_timer()
    {
        timer_ = create_wall_timer(std::chrono::duration<double>(timing_params_.control_period_s), std::bind(&PitchNode::control_tick, this));
    }

    /**
     * @brief 鼠标 Y 轴映射成 [-1, 1] 的 pitch 目标速率输入
     *
     * @param msg 键盘状态消息
     */
    void on_keyboard(const framework::msg::KeyboardState::SharedPtr& msg)
    {
        pitch_input_ = std::clamp(msg->mouse_dy * input_params_.mouse_sensitivity, -1.0, 1.0);
    }

    /**
     * @brief 按关节名取 pitch 的角度和角速度
     *
     * @param msg 关节状态消息
     */
    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr& msg)
    {
        PitchSample sample = pitch_;
        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == joint_params_.pitch)
            {
                sample.sim_time_s = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
                sample.q = msg->position[i];
                sample.dq = msg->velocity[i];
                sample.valid = true;
                break;
            }
        }
        pitch_ = sample;
    }

    /**
     * @brief 定时器里跑的一拍：积分目标、算力矩、发布、打日志
     */
    void control_tick()
    {
        if (!pitch_.valid)
        {
            return;
        }

        const double pitch_rate = std::clamp(pitch_input_ * target_params_.rate_limit,     -std::abs(target_params_.rate_limit), std::abs(target_params_.rate_limit));
        pitch_des_ = std::clamp(pitch_des_ + pitch_rate * timing_params_.control_period_s, -std::abs(target_params_.soft_limit), std::abs(target_params_.soft_limit));

        Lqr::State state = Lqr::State::Zero();
        // pitch 虽有机械限位，控制误差仍统一走 [-pi, pi]，避免异常多圈状态灌进 LQR。
        state(0) = radian_error_pm_pi(pitch_.q, pitch_des_);
        state(1) = pitch_.dq;

        Lqr::State target = Lqr::State::Zero();
        target(0) = 0.0;
        target(1) = 0.0;

        const double gravity_comp = gravity_params_.enable ? gravity_params_.sign * gravity_params_.mass * 9.81 * gravity_params_.com_x * std::cos(pitch_.q) : 0.0;
        const Lqr::Control u = lqr_.update(state, target);
        const double tau = std::clamp(u(0) + gravity_comp, -std::abs(control_params_.ctrl_limit), std::abs(control_params_.ctrl_limit));

        std_msgs::msg::Float64 msg;
        msg.data = tau;
        pitch_cmd_pub_->publish(msg);

        // 对比实验：时间来自关节消息（仿真时间），角度 deg、速度 rad/s、力矩 N·m。
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
                             "pitch t=%.4f g_on=%d q_deg=%.3f des_deg=%.3f dq_rad_s=%.4f lqr_Nm=%.5f grav_Nm=%.5f cmd_Nm=%.5f sat=%d",
                             pitch_.sim_time_s, gravity_params_.enable ? 1 : 0,
                             pitch_.q * kRadToDeg, pitch_des_ * kRadToDeg, pitch_.dq,
                             u(0), gravity_comp, tau,
                             std::abs(u(0) + gravity_comp) > std::abs(control_params_.ctrl_limit) ? 1 : 0);
    }
};
