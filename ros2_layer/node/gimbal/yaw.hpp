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
 *       不额外引入底盘 IMU，角速度订阅底盘轮速运动学估计。
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
#include "framework/msg/chassis_velocity.hpp"
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

        ros_.timer = create_wall_timer(std::chrono::duration<double>(control_params_.control_period_s), std::bind(&YawNode::control_tick, this));

        RCLCPP_INFO(get_logger(), "云台就绪：小 yaw 世界角闭环，大 yaw 软限随动，固定基座惯性前馈%s，参考导数低通 %.1f Hz",
            inertia_params_.enable ? "开启" : "关闭", inertia_params_.reference_lpf_hz);
    }

private:
    using YawLqr = algorithm::controller::Lqr<2, 1>;

    static constexpr double kDegToRad = M_PI / 180.0;
    static constexpr double kRadToDeg = 180.0 / M_PI;

    static constexpr double kYawJointDamping = 0.002;

    /**
     * @brief 被控对象物理参数：两轴转动惯量与电机电枢惯量，用来算 LQR 的输入矩阵
     */
    struct ModelParams
    {
        double j_big          {0.0025};
        double j_small        {0.0005};
        double armature_big   {0.0018};
        double armature_small {0.0018};
    };

    /**
     * @brief 控制周期与 LQR 权重：大小轴共用一个周期，Q/R 按轴各取一组
     */
    struct ControlParams
    {
        double control_period_s     {0.001};
        std::array<double, 2> q_pos {1000.0, 20.0};
        std::array<double, 2> q_vel {1.0,    1.0};
        std::array<double, 2> r     {1.0,    1.0};
    };

    /**
     * @brief 固定基座参考逆动力学前馈，不改变大小轴目标分配
     */
    struct InertiaParams
    {
        bool enable {true};
        double reference_lpf_hz {15.0};
    };

    /**
     * @brief 底盘角速度补偿配置，与惯性前馈配置分开管理
     */
    struct BaseRateParams
    {
        bool enable {true};
        double timeout_s {0.1};
    };

    /**
     * @brief 底盘话题采样，时间戳和接收时间随测量值一起管理
     */
    struct BaseRateSample
    {
        double rate {0.0};
        std::int64_t stamp_ns {0};
        bool valid {false};
        std::chrono::steady_clock::time_point received {};
    };

    /**
     * @brief 键盘输入状态和底盘模式请求
     */
    struct InputState
    {
        double command {0.0};
        bool spin_requested {false};
        bool follow_requested {false};
    };

    /**
     * @brief 世界目标与历史大轴分配参考
     */
    struct ReferenceState
    {
        double world {0.0};
        double big {0.0};
        double big_rate {0.0};
        double rate_reference {0.0};
        std::int64_t rate_stamp_ns {0};
        bool rate_initialized {false};
        bool target_initialized {false};
        bool big_initialized {false};
    };

    /**
     * @brief 分配参考的滤波速度、加速度和各项前馈力矩
     */
    struct InertiaState
    {
        bool   initialized        {false};

        double big_rate           {0.0};
        double small_rate         {0.0};
        double big_accel          {0.0};
        double small_accel        {0.0};
        double big_self           {0.0};
        double big_cross          {0.0};
        double small_self         {0.0};
        double small_cross        {0.0};
        double big_damping        {0.0};
        double small_damping      {0.0};
    };

    /**
     * @brief 键盘输入映射：鼠标横移灵敏度和世界角速度上限
     */
    struct InputParams
    {
        double mouse_yaw_sensitivity {-1.0};
        double target_rate_limit     {M_PI};
    };

    /**
     * @brief 限位与限幅：小轴软限位，以及两轴力矩限幅
     */
    struct LimitParams
    {
        double small_soft_limit {M_PI / 3.0};
        double big_ctrl_limit   {7.0};
        double small_ctrl_limit {7.0};
    };

    /**
     * @brief 关节名：和 joint_states 里的名字对齐
     */
    struct JointParams
    {
        std::string big_yaw_joint   {"big_yaw_joint"};
        std::string small_yaw_joint {"small_yaw_joint"};
    };

    /**
     * @brief 话题名：订阅和发布的 ROS 接口
     */
    struct TopicParams
    {
        std::string keyboard_topic     {"/keyboard"};
        std::string joint_states_topic {"/joint_states"};
        std::string motor_topic_prefix {"/motor"};
        std::string yaw_angle_topic    {"/gimbal/imu/euler_rad"};
        std::string yaw_rate_topic     {"/gimbal/imu/angular_velocity"};
        std::string base_velocity_topic {"/chassis/velocity"};
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
        std::int64_t stamp_ns {0};
        bool   valid {false};
    };

    /**
     * @brief IMU 世界角采样：世界 yaw、世界 yaw 角速度和各自的到达标志
     */
    struct ImuSample
    {
        Eigen::Vector3d rpy {Eigen::Vector3d::Zero()};
        Eigen::Vector3d gyro {Eigen::Vector3d::Zero()};
        std::int64_t rate_stamp_ns {0};
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

    /**
     * @brief 底盘话题提供的平面角速度与编码器推算的基座航向
     */
    struct BaseYawSample
    {
        double yaw {0.0};
        double heading_rate {0.0};
        double axis_rate {0.0};
        bool valid {false};
    };

    BaseYawSample base_yaw_ {};
    BaseRateSample base_rate_ {};
    BaseRateParams base_rate_params_ {};
    InputState input_ {};
    ReferenceState reference_ {};

    YawLqr big_joint_lqr_   {};
    YawLqr small_world_lqr_ {};

    InertiaState  inertia_  {};
    InertiaParams inertia_params_ {};

    ImuSample     imu_      {};
    JointSample   joints_   {};

    ModelParams   model_params_   {};
    InputParams   input_params_   {};
    LimitParams   limit_params_   {};
    JointParams   joint_params_   {};
    TopicParams   topic_params_   {};
    ControlParams control_params_ {};

    /**
     * @brief 节点订阅、发布和控制定时器
     */
    struct RosInterfaces
    {
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub;
        rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr      keyboard_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_angle_sub;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr yaw_rate_sub;

        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr big_yaw_cmd_pub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr small_yaw_cmd_pub;

        rclcpp::Subscription<framework::msg::ChassisVelocity>::SharedPtr base_velocity_sub;
        rclcpp::TimerBase::SharedPtr timer;
    };

    RosInterfaces ros_ {};

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

        limit_params_.small_soft_limit      = declare_parameter("small_yaw_soft_limit_deg", 60.0)  * kDegToRad;
        limit_params_.big_ctrl_limit        = declare_parameter("big_yaw_ctrl_limit",       7.0);
        limit_params_.small_ctrl_limit      = declare_parameter("small_yaw_ctrl_limit",     7.0);

        base_rate_params_.timeout_s = declare_parameter("yaw_base_rate_timeout_s", 0.1);
        base_rate_params_.enable = declare_parameter("yaw_base_rate_comp_enable", true);
        inertia_params_.enable              = declare_parameter("yaw_inertia_ff_enable",    true);
        inertia_params_.reference_lpf_hz    = declare_parameter("yaw_inertia_reference_lpf_hz", 15.0);

        model_params_.j_big                 = declare_parameter("j_big",                    0.0025);
        model_params_.j_small               = declare_parameter("j_small",                  0.0005);
        model_params_.armature_big          = declare_parameter("armature_big",             0.0018);
        model_params_.armature_small        = declare_parameter("armature_small",           0.0018);

        joint_params_.big_yaw_joint         = declare_parameter("big_yaw_joint",            "big_yaw_joint");
        joint_params_.small_yaw_joint       = declare_parameter("small_yaw_joint",          "small_yaw_joint");

        topic_params_.base_velocity_topic = declare_parameter("base_velocity_topic", "/chassis/velocity");
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
        if (!(base_rate_params_.timeout_s > 0.0) || !std::isfinite(base_rate_params_.timeout_s))
        {
            throw std::runtime_error("yaw_base_rate_timeout_s must be positive and finite");
        }
        if (!(inertia_params_.reference_lpf_hz > 0.0) || !std::isfinite(inertia_params_.reference_lpf_hz))
        {
            throw std::runtime_error("yaw_inertia_reference_lpf_hz must be positive and finite");
        }
        if (!(control_params_.control_period_s  >  0.0  && model_params_.j_big            >  0.0   && model_params_.j_small > 0.0 &&
              model_params_.armature_big        >= 0.0  && model_params_.armature_small   >= 0.0   &&
              limit_params_.small_soft_limit    >  0.0  && limit_params_.small_soft_limit <  360.0 * kDegToRad &&
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
        ros_.base_velocity_sub = create_subscription<framework::msg::ChassisVelocity>(
            topic_params_.base_velocity_topic, rclcpp::SensorDataQoS(),
            [this](const framework::msg::ChassisVelocity::SharedPtr msg)  { on_base_velocity(msg); });

        ros_.joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
            topic_params_.joint_states_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)       { on_joint_states(msg); });

        ros_.keyboard_sub = create_subscription<framework::msg::KeyboardState>(
            topic_params_.keyboard_topic, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg)      { on_keyboard(msg); });

        ros_.yaw_angle_sub = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            topic_params_.yaw_angle_topic, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_yaw_angle(msg); });

        ros_.yaw_rate_sub = create_subscription<geometry_msgs::msg::Vector3Stamped>(
            topic_params_.yaw_rate_topic, rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { on_yaw_rate(msg); });

        ros_.big_yaw_cmd_pub   = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_topic_prefix + "/big_yaw/cmd_force",   10);
        ros_.small_yaw_cmd_pub = create_publisher<std_msgs::msg::Float64>(topic_params_.motor_topic_prefix + "/small_yaw/cmd_force", 10);
    }

    /**
     * @brief 鼠标横移 + 左右方向键合成 [-1, 1] 的世界角速度输入
     *
     * @param msg 键盘状态消息
     */
    void on_keyboard(const framework::msg::KeyboardState::SharedPtr& msg)
    {
        input_.spin_requested    = msg->shift_hold || msg->spin_active;
        input_.follow_requested  = msg->ctrl_hold;
        const double arrow_input = (msg->right ? 1.0 : 0.0) - (msg->left ? 1.0 : 0.0);
        input_.command = std::clamp(msg->mouse_dx * input_params_.mouse_yaw_sensitivity + arrow_input, -1.0, 1.0);
    }

    /**
     * @brief 只接受更新时间戳的帧；首次收到时把世界目标初始化到当前角
     *
     * @param msg 世界角消息，z 分量是 yaw
     */
    void on_yaw_angle(const geometry_msgs::msg::Vector3Stamped::SharedPtr& msg)
    {
        const std::int64_t stamp_ns = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + static_cast<std::int64_t>(msg->header.stamp.nanosec);
        if (imu_.angle_valid && stamp_ns <= imu_.stamp_ns) {
            return;
        }

        imu_.rpy = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
        if (!imu_.rpy.allFinite()) {
            imu_.angle_valid = false;
            return;
        }
        imu_.yaw_world = normalize_radian_pm_pi(msg->vector.z);
        imu_.stamp_ns = stamp_ns;
        imu_.angle_valid = true;

        if (!reference_.target_initialized)
        {
            reference_.world = imu_.yaw_world;
            reference_.target_initialized = true;
            RCLCPP_INFO(get_logger(), "世界 yaw 目标初始化到 IMU 当前角度：%.1f deg", reference_.world * kRadToDeg);
        }
    }

    /**
     * @brief 按关节名取大小 yaw 的角度和角速度，两轴都齐才标记有效
     *
     * @param msg 关节状态消息
     */
    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr& msg)
    {
        JointSample sample    = joints_;
        sample.stamp_ns = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
        bool        got_big   = false;
        bool        got_small = false;

        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (i >= msg->position.size() || i >= msg->velocity.size()) {
                continue;
            }

            if (msg->name[i] == joint_params_.big_yaw_joint) {
                sample.q_big = msg->position[i];
                sample.dq_big = msg->velocity[i];
                got_big = true;
            }
            else if (msg->name[i] == joint_params_.small_yaw_joint) {
                sample.q_small = msg->position[i];
                sample.dq_small = msg->velocity[i];
                got_small = true;
            }
        }
        sample.valid = got_big && got_small && std::isfinite(sample.q_big) && std::isfinite(sample.dq_big) &&
                       std::isfinite(sample.q_small) && std::isfinite(sample.dq_small);
        joints_ = sample;
    }

    /**
     * @brief 保存传感器局部角速度；坐标转换与关节反解在控制前统一完成
     */
    void on_yaw_rate(const geometry_msgs::msg::Vector3Stamped::SharedPtr& msg)
    {
        const std::int64_t stamp = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
        if (imu_.rate_valid && stamp <= imu_.rate_stamp_ns)
        {
            return;
        }
        imu_.gyro = Eigen::Vector3d(msg->vector.x, msg->vector.y, msg->vector.z);
        imu_.rate_stamp_ns = stamp;
        imu_.rate_valid = imu_.gyro.allFinite();
    }

    /**
     * @brief 航向角速度由前向向量求导，避免将世界 omega.z 当欧拉 yaw 导数
     */
    static bool heading_rate(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& omega, double& rate)
    {
        const Eigen::Vector3d forward = rotation.col(0);
        const double horizontal_squared = forward.x() * forward.x() + forward.y() * forward.y();
        if (horizontal_squared < 1e-8)
        {
            return false;
        }
        const Eigen::Vector3d derivative = omega.cross(forward);
        rate = (forward.x() * derivative.y() - forward.y() * derivative.x()) / horizontal_squared;
        return std::isfinite(rate);
    }

    /**
     * @brief 接收底盘系实测轮速反解值，不接收目标自转指令作为反馈
     */
    void on_base_velocity(const framework::msg::ChassisVelocity::SharedPtr& msg)
    {
        const std::int64_t stamp = static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL + msg->header.stamp.nanosec;
        if (!std::isfinite(msg->wz_rad_s) || msg->header.frame_id != "base_link" || (base_rate_.valid && stamp <= base_rate_.stamp_ns))
        {
            return;
        }
        base_rate_.rate     = msg->wz_rad_s;
        base_rate_.stamp_ns = stamp;
        base_rate_.received = std::chrono::steady_clock::now();
        base_rate_.valid    = true;
    }

    /**
     * @brief 修正云台航向速率，底盘速度主反馈来自 chassis 发布的平面运动学估计
     * @note 底盘局部 z 速率用于平面 yaw 补偿；倾斜工况下不等同世界欧拉航向导数。
     */
    void update_base_kinematics()
    {
        base_yaw_ = BaseYawSample {};
        const Eigen::Matrix3d world_barrel = (Eigen::AngleAxisd(imu_.rpy.z(), Eigen::Vector3d::UnitZ())
                                           *  Eigen::AngleAxisd(imu_.rpy.y(), Eigen::Vector3d::UnitY())
                                           *  Eigen::AngleAxisd(imu_.rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();

        imu_.rate_valid = heading_rate(world_barrel, world_barrel * imu_.gyro, imu_.yaw_rate_world);

        const double received_age = std::chrono::duration<double>(std::chrono::steady_clock::now() - base_rate_.received).count();
        if (!base_rate_.valid || received_age > base_rate_params_.timeout_s || std::abs(imu_.stamp_ns - base_rate_.stamp_ns) * 1e-9 > base_rate_params_.timeout_s)
        {
            return;
        }

        base_yaw_.yaw          = normalize_radian_pm_pi(imu_.yaw_world - joints_.q_big - joints_.q_small);
        base_yaw_.axis_rate    = base_rate_.rate;
        base_yaw_.heading_rate = base_rate_.rate;
        base_yaw_.valid        = true;
    }

    /**
     * @brief 解析参考速度经低通后求加速度，不对含反馈测量的分配角二次差分
     * @note Shift / Ctrl、基座运动或启用底盘补偿却没有有效速度时，退出固定基座前馈。
     *       不修改参考角，不额外增加角度/速度限位。
     */
    bool update_inertia_feedforward(double big_rate_reference, double small_rate_reference)
    {
        const bool active = inertia_params_.enable && !input_.spin_requested && !input_.follow_requested &&
                          (!base_rate_params_.enable || (base_yaw_.valid && std::abs(base_yaw_.heading_rate) <= 0.01));

        if (!active || !inertia_.initialized)
        {
            inertia_ = InertiaState {};
            inertia_.initialized = active;
            return false;
        }

        const double dt             = control_params_.control_period_s;
        const double alpha          = -std::expm1(-2.0 * M_PI * inertia_params_.reference_lpf_hz * dt);
        const double raw_big_rate   = big_rate_reference;
        const double raw_small_rate = small_rate_reference;
        const double old_big_rate   = inertia_.big_rate;
        const double old_small_rate = inertia_.small_rate;

        inertia_.big_rate          += alpha * (raw_big_rate   - inertia_.big_rate);
        inertia_.small_rate        += alpha * (raw_small_rate - inertia_.small_rate);
        inertia_.big_accel          = (inertia_.big_rate      - old_big_rate)   / dt;
        inertia_.small_accel        = (inertia_.small_rate    - old_small_rate) / dt;

        inertia_.big_self           = (model_params_.j_big   + model_params_.j_small + model_params_.armature_big) * inertia_.big_accel;
        inertia_.big_cross          =  model_params_.j_small * inertia_.small_accel;
        inertia_.small_self         = (model_params_.j_small + model_params_.armature_small) * inertia_.small_accel;
        inertia_.small_cross        =  model_params_.j_small * inertia_.big_accel;
        inertia_.big_damping        =  kYawJointDamping      * inertia_.big_rate;
        inertia_.small_damping      =  kYawJointDamping      * inertia_.small_rate;

        return true;
    }

    /**
     * @brief 定时器里跑的一拍：分配大小轴目标后各跑一次 LQR，发力矩
     */
    void control_tick()
    {
        if (!joints_.valid || !imu_.angle_valid || !imu_.rate_valid || !reference_.target_initialized)
        {
            return;
        }
        // 三路话题来自同一次仿真发布；避免新编码器与旧姿态拼出假的底盘转角。
        if (joints_.stamp_ns != imu_.stamp_ns || imu_.rate_stamp_ns != imu_.stamp_ns)
        {
            return;
        }
        if (!reference_.big_initialized)
        {
            reference_.big = normalize_radian_pm_pi(joints_.q_big);
            reference_.big_initialized = true;
        }
        update_base_kinematics();
        if (!imu_.rate_valid)
        {
            return;
        }
        const bool   base_comp_active  = base_rate_params_.enable && base_yaw_.valid;
        const double target_world_rate = input_.command * input_params_.target_rate_limit;
        reference_.world = normalize_radian_pm_pi(reference_.world + target_world_rate * control_params_.control_period_s);

        // 由现有云台姿态和两轴编码器求底盘朝向，不增加底盘 IMU/速度反馈。
        // 只作坐标分配，不能把世界角反馈纠偏或积分混入大轴参考。
        const double base_radian        = base_comp_active ? base_yaw_.yaw : normalize_radian_pm_pi(imu_.yaw_world - joints_.q_big - joints_.q_small);
        const double target_body_radian = normalize_radian_pm_pi(reference_.world - base_radian);
        const double small_required     = radian_error_pm_pi(target_body_radian, reference_.big);
        const double small_allocated    = std::clamp(small_required, -limit_params_.small_soft_limit, limit_params_.small_soft_limit);
        const double allocation_excess  = small_required - small_allocated;

        // 限位内保持历史大轴参考，不能每拍跟随实际大轴角度而放任漂移。
        if (allocation_excess != 0.0)
        {
            reference_.big = normalize_radian_pm_pi(reference_.big + allocation_excess);
        }

        // 位置分配只在超限时移动大轴；速度分配使用解析底盘航向速度。
        const double body_reference_rate = target_world_rate - (base_comp_active ? base_yaw_.heading_rate : 0.0);
        // 速度参考与真正发出的角度参考一致；不能在交接边缘按 >= soft_limit
        // 将速度目标在 0 与 -base_rate 之间硬切换。
        // 差分与滤波都跟随新的仿真反馈时间；旧样本重复控制时保持速度参考。
        // 仿真变慢或反馈降频后，不能把一次数毫秒的位移除以固定 1 ms。
        if (!reference_.rate_initialized || imu_.stamp_ns < reference_.rate_stamp_ns)
        {
            reference_.rate_reference = reference_.big;
            reference_.rate_stamp_ns = imu_.stamp_ns;
            reference_.big_rate = 0.0;
            reference_.rate_initialized = true;
        }
        else if (imu_.stamp_ns > reference_.rate_stamp_ns)
        {
            const double sample_dt = (imu_.stamp_ns - reference_.rate_stamp_ns) * 1e-9;
            const double allocated_big_rate = radian_error_pm_pi(reference_.big, reference_.rate_reference) / sample_dt;
            const double reference_alpha = -std::expm1(-2.0 * M_PI * inertia_params_.reference_lpf_hz * sample_dt);
            reference_.big_rate += reference_alpha * (allocated_big_rate - reference_.big_rate);
            reference_.rate_reference = reference_.big;
            reference_.rate_stamp_ns = imu_.stamp_ns;
        }

        const double big_rate_reference = reference_.big_rate;
        // 小轴需抵消大轴实际速度，阻尼力矩按相对关节速度计算。
        const double small_rate_reference = body_reference_rate - joints_.dq_big;
        const bool   inertia_active = update_inertia_feedforward(big_rate_reference, body_reference_rate - big_rate_reference);

        YawLqr::State big_error;
        big_error   << radian_error_pm_pi(joints_.q_big,  reference_.big), joints_.dq_big - (base_comp_active ? big_rate_reference : (inertia_active ? inertia_.big_rate : 0.0));

        YawLqr::State small_error;
        small_error << radian_error_pm_pi(imu_.yaw_world, reference_.world), imu_.yaw_rate_world - target_world_rate;

        // 两个 LQR 都以误差为反馈、零为目标；小轴补偿大轴运动造成的世界角扰动。
        const double big_feedback   = big_joint_lqr_  .update(big_error,   YawLqr::State::Zero())(0);
        const double small_feedback = small_world_lqr_.update(small_error, YawLqr::State::Zero())(0);
        const double big_damping    = base_comp_active ? kYawJointDamping    * big_rate_reference   : inertia_.big_damping;
        const double small_damping  = base_comp_active ? kYawJointDamping    * small_rate_reference : inertia_.small_damping;
        const double big_raw        = big_feedback     + inertia_.big_self   + inertia_.big_cross   + big_damping;
        const double small_raw      = small_feedback   + inertia_.small_self + inertia_.small_cross + small_damping;

        const TorqueCommand command {
              std::clamp(big_raw,   -limit_params_.big_ctrl_limit,   limit_params_.big_ctrl_limit),
            std::clamp(small_raw, -limit_params_.small_ctrl_limit, limit_params_.small_ctrl_limit)
        };

        publish_torque(command);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20,
            "yaw基座 t=%.6f active=%d sample_valid=%d age_ms=%.2f stamp_delta_ms=%.2f gyro_z=%.5f base_heading_rate=%.5f base_axis_rate=%.5f body_ref_rate=%.5f joint_ref_rate=(%.5f,%.5f) damping=(%.5f,%.5f)",
            imu_.stamp_ns * 1e-9, base_comp_active ? 1 : 0, base_rate_.valid ? 1 : 0,
            base_rate_.valid ? std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - base_rate_.received).count() : -1.0,
            base_rate_.valid ? (imu_.stamp_ns - base_rate_.stamp_ns) * 1e-6 : 0.0,
            imu_.gyro.z(), base_yaw_.heading_rate, base_yaw_.axis_rate, body_reference_rate,
            big_rate_reference, small_rate_reference, big_damping, small_damping);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20,
            "yaw惯性 t=%.6f active=%d ref_rate=(%.5f,%.5f) ref_accel=(%.5f,%.5f) "
            "fb=(%.5f,%.5f) self=(%.5f,%.5f) cross=(%.5f,%.5f) damping=(%.5f,%.5f) sent=(%.5f,%.5f)",
            imu_.stamp_ns * 1e-9, inertia_active ? 1 : 0, inertia_.big_rate, inertia_.small_rate,
            inertia_.big_accel, inertia_.small_accel, big_feedback, small_feedback,
            inertia_.big_self, inertia_.small_self, inertia_.big_cross, inertia_.small_cross,
            big_damping, small_damping, command.big, command.small);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 20,
            "yaw协同 sim_t=%.6f | world des/imu/err=(%.2f, %.2f, %.2f) deg rate/des=(%.4f, %.4f) rad/s | "
            "base_est/body_des=(%.2f, %.2f) deg | small required/alloc/q=(%.2f, %.2f, %.2f) deg dq=%.4f | "
            "big ref/q/err=(%.2f, %.2f, %.2f) deg dq=%.4f alloc_delta=%.4f deg | "
            "tau raw=(%.5f, %.5f) sent=(%.5f, %.5f) Nm | input=%.3f",
            imu_.stamp_ns*1e-9,
            reference_.world*kRadToDeg, imu_.yaw_world*kRadToDeg, small_error(0)*kRadToDeg,
            imu_.yaw_rate_world, target_world_rate,
            base_radian*kRadToDeg, target_body_radian*kRadToDeg,
            small_required*kRadToDeg, small_allocated*kRadToDeg, joints_.q_small*kRadToDeg, joints_.dq_small,
            reference_.big*kRadToDeg, normalize_radian_pm_pi(joints_.q_big)*kRadToDeg,
            big_error(0)*kRadToDeg, joints_.dq_big, allocation_excess*kRadToDeg,
            big_raw, small_raw, command.big, command.small, input_.command);
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
        ros_.big_yaw_cmd_pub  ->publish(msg);
        msg.data = command.small;
        ros_.small_yaw_cmd_pub->publish(msg);
    }
};
