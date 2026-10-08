/**
 * @file yaw.hpp
 * @author qingyu
 * @brief 小 yaw 世界角 LQR，大 yaw 软限位随动 LQR
 * @version 0.2
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @note 小轴闭环世界角，大轴只做随动参考：世界角误差先给大轴定目标，超出软限位的部分
 *       才转给大轴参考，限位内保持大轴参考不动。底盘朝向由 IMU 世界角和两轴编码器推算，
 *       不额外引入底盘 IMU 或速度反馈。
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/controller/lqr.hpp"
#include "framework/algorithm/math/angle.hpp"
#include "framework/msg/keyboard_state.hpp"

using namespace algorithm::math;

/**
 * @brief yaw 云台节点：小 yaw 世界角闭环，大 yaw 软限位随动
 */
class YawNode : public rclcpp::Node
{
public:
    explicit YawNode(const std::string& node_name = "yaw", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        read_parameters();

        configure_lqr();
        setup_ros_interfaces();

        timer_ = create_wall_timer(std::chrono::duration<double>(control_params_.control_period_s), std::bind(&YawNode::control_tick, this));

        RCLCPP_INFO(get_logger(), "云台就绪：小 yaw IMU 世界角闭环，大 yaw 仅在小轴目标越过软限位时随动");
    }

private:
    static constexpr double kDegToRad = M_PI / 180.0;
    static constexpr double kRadToDeg = 180.0 / M_PI;
    using YawLqr = algorithm::controller::Lqr<2, 1>;
    static constexpr double kYawJointDamping = 0.002;

    /**
     * @brief 被控对象物理参数：两轴转动惯量与电机电枢惯量，用来算 LQR 的输入矩阵
     */
    struct ModelParams
    {
        double j_big    {0.0025};
        double j_small  {0.0005};
        double armature_big   {0.0018};
        double armature_small {0.0018};
    };

    /**
     * @brief 控制周期与 LQR 权重：大小轴共用一个周期，Q/R 按轴各取一组
     */
    struct ControlParams
    {
        double control_period_s {0.001};
        std::array<double, 2> q_pos {1000.0, 20.0};
        std::array<double, 2> q_vel {1.0, 1.0};
        std::array<double, 2> r     {1.0, 1.0};
    };

    /**
     * @brief 键盘输入映射：鼠标横移灵敏度和世界角速度上限
     */
    struct InputParams
    {
        double mouse_yaw_sensitivity {-1.0};
        double target_rate_limit {M_PI};
    };

    /**
     * @brief 限位与限幅：小轴软限位，以及两轴力矩限幅
     */
    struct LimitParams
    {
        double small_soft_limit {M_PI / 3.0};
        double big_ctrl_limit {7.0};
        double small_ctrl_limit {7.0};
    };

    /**
     * @brief 关节名：和 joint_states 里的名字对齐
     */
    struct JointParams
    {
        std::string big_yaw_joint {"big_yaw_joint"};
        std::string small_yaw_joint {"small_yaw_joint"};
    };

    /**
     * @brief 话题名：订阅和发布的 ROS 接口
     */
    struct TopicParams
    {
        std::string keyboard_topic {"/keyboard"};
        std::string joint_states_topic {"/joint_states"};
        std::string motor_topic_prefix {"/motor"};
        std::string yaw_angle_topic {"/gimbal/imu/euler_rad"};
        std::string yaw_rate_topic {"/gimbal/imu/angular_velocity"};
    };

    /**
     * @brief 两轴关节采样：角度和角速度，两轴都收到才算有效
     */
    struct JointSample
    {
        double q_big {0.0};
        double dq_big {0.0};
        double q_small {0.0};
        double dq_small {0.0};
        bool valid {false};
    };

    /**
     * @brief IMU 世界角采样：世界 yaw、世界 yaw 角速度和各自的到达标志
     */
    struct ImuSample
    {
        double yaw_world {0.0};
        double yaw_rate_world {0.0};
        std::int64_t stamp_ns {0};
        bool angle_valid {false};
        bool rate_valid {false};
    };

    /**
     * @brief 两轴力矩指令
     */
    struct TorqueCommand
    {
        double big {0.0};
        double small {0.0};
    };

    
    YawLqr          big_joint_lqr_   {};
    YawLqr          small_world_lqr_ {};

    ImuSample       imu_    {};
    JointSample     joints_ {};

    ModelParams     model_params_   {};
    InputParams     input_params_   {};
    LimitParams     limit_params_   {};
    JointParams     joint_params_   {};
    TopicParams     topic_params_   {};
    ControlParams   control_params_ {};
    
    double theta_des_world_ {0.0};
    double keyboard_input_  {0.0};
    double big_reference_radian_ {0.0};

    bool   target_initialized_ {false};
    bool   big_reference_initialized_ {false};

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub_;
    rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr      keyboard_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_angle_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_rate_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr big_yaw_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr small_yaw_cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief 读 2 元数组参数，长度不是 2 直接抛错
     *
     * @param name 参数名
     * @param fallback 默认值，同时给出参数长度
     * @return std::array<double, 2> 参数值
     */
    std::array<double, 2> get_array_parameter(const std::string& name, const std::array<double, 2>& fallback)
    {
        const std::vector<double> defaults(fallback.begin(), fallback.end());
        const std::vector<double> values = declare_parameter<std::vector<double>>(name, defaults);
        if (values.size() != 2)
        {
            throw std::runtime_error("yaw 参数 " + name + " 必须恰好有两个数");
        }
        return {values[0], values[1]};
    }

    /**
     * @brief 从 ROS 参数读入上面六组参数
     */
    void read_parameters()
    {
        control_params_.control_period_s    = declare_parameter("control_period_s",         0.001);
        control_params_.q_pos               = get_array_parameter("q_pos",                  {1000.0, 20.0});
        control_params_.q_vel               = get_array_parameter("q_vel",                  {1.0, 1.0});
        control_params_.r                   = get_array_parameter("r",                      {1.0, 1.0});

        input_params_.target_rate_limit     = declare_parameter("target_rate_limit_deg",    180.0) * kDegToRad;
        input_params_.mouse_yaw_sensitivity = declare_parameter("mouse_yaw_sensitivity",    -1.0);

        limit_params_.small_soft_limit      = declare_parameter("small_yaw_soft_limit_deg", 60.0) * kDegToRad;
        limit_params_.big_ctrl_limit        = declare_parameter("big_yaw_ctrl_limit",       7.0);
        limit_params_.small_ctrl_limit      = declare_parameter("small_yaw_ctrl_limit",     7.0);

        model_params_.j_big                 = declare_parameter("j_big",                    0.0025);
        model_params_.j_small               = declare_parameter("j_small",                  0.0005);
        model_params_.armature_big          = declare_parameter("armature_big",             0.0018);
        model_params_.armature_small        = declare_parameter("armature_small",           0.0018);

        joint_params_.big_yaw_joint         = declare_parameter("big_yaw_joint",            "big_yaw_joint");
        joint_params_.small_yaw_joint       = declare_parameter("small_yaw_joint",          "small_yaw_joint");

        topic_params_.keyboard_topic        = declare_parameter("keyboard_topic",           "/keyboard");
        topic_params_.joint_states_topic    = declare_parameter("joint_states_topic",       "/joint_states");
        topic_params_.motor_topic_prefix    = declare_parameter("motor_topic_prefix",       "/motor");
        topic_params_.yaw_angle_topic       = declare_parameter("yaw_angle_topic",          "/gimbal/imu/euler_rad");
        topic_params_.yaw_rate_topic        = declare_parameter("yaw_rate_topic",           "/gimbal/imu/angular_velocity");
    }

    /**
     * @brief 大小轴各配一个 LQR：A/B 共用控制周期、按各轴惯量离散，Q/R 取各自权重
     */
    void configure_lqr()
    {
        if (!(control_params_.control_period_s  >  0.0  && model_params_.j_big            >  0.0   && model_params_.j_small > 0.0 &&
              model_params_.armature_big        >= 0.0  && model_params_.armature_small   >= 0.0   &&
              limit_params_.small_soft_limit    >  0.0  && limit_params_.small_soft_limit <  120.0 * kDegToRad &&
              input_params_.target_rate_limit   >  0.0  &&
              limit_params_.big_ctrl_limit      >  0.0  && limit_params_.small_ctrl_limit >  0.0))
        {
            throw std::runtime_error("yaw 周期、惯量、限位、限幅必须有效");
        }

        const std::array<double, 2> inertia {
            model_params_.j_big   + model_params_.j_small + model_params_.armature_big,
            model_params_.j_small + model_params_.armature_small
        };

        std::array<YawLqr*, 2> controllers {&big_joint_lqr_, &small_world_lqr_};

        for (std::size_t i = 0; i < controllers.size(); ++i)
        {
            if (!(control_params_.q_pos[i] > 0.0 && control_params_.q_vel[i] > 0.0 && control_params_.r[i] > 0.0))
            {
                throw std::runtime_error("yaw Q/R 必须为正数");
            }

            YawLqr::StateMatrix a = YawLqr::StateMatrix::Identity();
            a(0, 1) = control_params_.control_period_s;
            a(1, 1) -= kYawJointDamping * control_params_.control_period_s / inertia[i];
            YawLqr::InputMatrix b = YawLqr::InputMatrix::Zero();
            b(1, 0) = control_params_.control_period_s / inertia[i];
            YawLqr::StateMatrix q = YawLqr::StateMatrix::Zero();
            q(0, 0) = control_params_.q_pos[i];
            q(1, 1) = control_params_.q_vel[i];

            YawLqr::ControlMatrix r;
            r << control_params_.r[i];
            controllers[i]->configure(a, b, q, r);
            controllers[i]->solve(5000, 1e-10);
        }
    }

    /**
     * @brief 订阅键盘、关节和 IMU 世界角/角速度，发布两轴力矩
     */
    void setup_ros_interfaces()
    {
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topic_params_.joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)        { on_joint_states(msg); });

        keyboard_sub_ = create_subscription<framework::msg::KeyboardState>(
            topic_params_.keyboard_topic, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg)       { on_keyboard(msg); });

        yaw_angle_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            topic_params_.yaw_angle_topic, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)  { on_yaw_angle(msg); });

        yaw_rate_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            topic_params_.yaw_rate_topic, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)  { imu_.yaw_rate_world = msg->vector.z; imu_.rate_valid = true; });

        big_yaw_cmd_pub_   = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_topic_prefix + "/big_yaw/cmd_force",   10);
        small_yaw_cmd_pub_ = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_topic_prefix + "/small_yaw/cmd_force", 10);
    }

    /**
     * @brief 鼠标横移 + 左右方向键合成 [-1, 1] 的世界角速度输入
     *
     * @param msg 键盘状态消息
     */
    void on_keyboard(const framework::msg::KeyboardState::SharedPtr& msg)
    {
        const double arrow_input = (msg->right ? 1.0 : 0.0) - (msg->left ? 1.0 : 0.0);
        keyboard_input_ = std::clamp(msg->mouse_dx * input_params_.mouse_yaw_sensitivity + arrow_input, -1.0, 1.0);
    }

    /**
     * @brief 只接受更新时间戳的帧；首次收到时把世界目标初始化到当前角
     *
     * @param msg 世界角消息，z 分量是 yaw
     */
    void on_yaw_angle(const geometry_msgs::msg::Vector3Stamped::SharedPtr& msg)
    {
        const std::int64_t stamp_ns = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + static_cast<std::int64_t>(msg->header.stamp.nanosec);
        if (imu_.angle_valid && stamp_ns <= imu_.stamp_ns)
        {
            return;
        }
        imu_.yaw_world = normalize_radian_pm_pi(msg->vector.z);
        imu_.stamp_ns = stamp_ns;
        imu_.angle_valid = true;

        if (!target_initialized_)
        {
            theta_des_world_ = imu_.yaw_world;
            target_initialized_ = true;
            RCLCPP_INFO(get_logger(), "世界 yaw 目标初始化到 IMU 当前角度：%.1f deg", theta_des_world_ * kRadToDeg);
        }
    }

    /**
     * @brief 按关节名取大小 yaw 的角度和角速度，两轴都齐才标记有效
     *
     * @param msg 关节状态消息
     */
    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr& msg)
    {
        JointSample sample = joints_;
        bool got_big = false;
        bool got_small = false;
        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (i >= msg->position.size() || i >= msg->velocity.size())
            {
                continue;
            }
            if (msg->name[i] == joint_params_.big_yaw_joint)
            {
                sample.q_big = msg->position[i];
                sample.dq_big = msg->velocity[i];
                got_big = true;
            }
            else if (msg->name[i] == joint_params_.small_yaw_joint)
            {
                sample.q_small = msg->position[i];
                sample.dq_small = msg->velocity[i];
                got_small = true;
            }
        }
        sample.valid = got_big && got_small;
        joints_ = sample;
    }

    /**
     * @brief 定时器里跑的一拍：分配大小轴目标后各跑一次 LQR，发力矩
     */
    void control_tick()
    {
        if (!joints_.valid || !imu_.angle_valid || !imu_.rate_valid || !target_initialized_)
        {
            return;
        }
        if (!big_reference_initialized_)
        {
            big_reference_radian_ = normalize_radian_pm_pi(joints_.q_big);
            big_reference_initialized_ = true;
        }
        const double target_world_rate = keyboard_input_ * input_params_.target_rate_limit;
        theta_des_world_ = normalize_radian_pm_pi(theta_des_world_ + target_world_rate * control_params_.control_period_s);

        // 由现有云台姿态和两轴编码器求底盘朝向，不增加底盘 IMU/速度反馈。
        // 只作坐标分配，不能把世界角反馈纠偏或积分混入大轴参考。
        const double base_radian        = normalize_radian_pm_pi(imu_.yaw_world   - joints_.q_big - joints_.q_small);
        const double target_body_radian = normalize_radian_pm_pi(theta_des_world_ - base_radian);
        const double small_required     = radian_error_pm_pi(target_body_radian, big_reference_radian_);
        const double small_allocated    = std::clamp(small_required, -limit_params_.small_soft_limit, limit_params_.small_soft_limit);
        const double allocation_excess  = small_required - small_allocated;

        // 限位内保持历史大轴参考，不能每拍跟随实际大轴角度而放任漂移。
        if (allocation_excess != 0.0)
        {
            big_reference_radian_ = normalize_radian_pm_pi(big_reference_radian_ + allocation_excess);
        }

        YawLqr::State big_error;
        big_error << radian_error_pm_pi(joints_.q_big, big_reference_radian_), joints_.dq_big;

        YawLqr::State small_error;
        small_error << radian_error_pm_pi(imu_.yaw_world, theta_des_world_), imu_.yaw_rate_world - target_world_rate;

        // 两个 LQR 都以误差为反馈、零为目标；小轴补偿大轴运动造成的世界角扰动。
        const double big_raw   = big_joint_lqr_  .update(big_error,   YawLqr::State::Zero())(0);
        const double small_raw = small_world_lqr_.update(small_error, YawLqr::State::Zero())(0);
        const TorqueCommand command {
              std::clamp(big_raw,   -limit_params_.big_ctrl_limit,   limit_params_.big_ctrl_limit),
            std::clamp(small_raw, -limit_params_.small_ctrl_limit, limit_params_.small_ctrl_limit)
        };

        publish_torque(command);

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 100,
            "yaw协同 sim_t=%.6f | world des/imu/err=(%.2f, %.2f, %.2f) deg rate/des=(%.4f, %.4f) rad/s | "
            "base_est/body_des=(%.2f, %.2f) deg | small required/alloc/q=(%.2f, %.2f, %.2f) deg dq=%.4f | "
            "big ref/q/err=(%.2f, %.2f, %.2f) deg dq=%.4f alloc_delta=%.4f deg | "
            "tau raw=(%.5f, %.5f) sent=(%.5f, %.5f) Nm | input=%.3f",
            imu_.stamp_ns*1e-9,
            theta_des_world_*kRadToDeg, imu_.yaw_world*kRadToDeg, small_error(0)*kRadToDeg,
            imu_.yaw_rate_world, target_world_rate,
            base_radian*kRadToDeg, target_body_radian*kRadToDeg,
            small_required*kRadToDeg, small_allocated*kRadToDeg, joints_.q_small*kRadToDeg, joints_.dq_small,
            big_reference_radian_*kRadToDeg, normalize_radian_pm_pi(joints_.q_big)*kRadToDeg,
            big_error(0)*kRadToDeg, joints_.dq_big, allocation_excess*kRadToDeg,
            big_raw, small_raw, command.big, command.small, keyboard_input_);
    }

    /**
     * @brief 两轴力矩分别发到 <motor_prefix>/<joint>/cmd_force
     *
     * @param command 两轴力矩指令
     */
    void publish_torque(const TorqueCommand& command)
    {
        std_msgs::msg::Float64 msg;
        msg.data = command.big;
        big_yaw_cmd_pub_->publish(msg);
        msg.data = command.small;
        small_yaw_cmd_pub_->publish(msg);
    }
};
