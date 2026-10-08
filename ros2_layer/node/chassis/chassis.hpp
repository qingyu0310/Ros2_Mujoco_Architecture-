/**
 * @file chassis.hpp
 * @author qingyu
 * @brief 底盘节点：第一人称云台视角驾驶，枪管指哪往哪走，Ctrl 按住跟随云台
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 *
 * @note 运动学链：键盘上/下给"沿枪管方向"的速度 -> 按枪管相对底盘的偏角
 *       q_big + q_small 旋到底盘系 -> 按 O 型全向轮安装几何分配四个轮子的目标转速
 *       -> 每轮一个速度 PD 出力矩。
 * @note 轮 i 正转驱动底盘沿 u_i = -normalize(z_hat x a_i) 运动。
 *       a_i 是轮轴单位向量，几何从参数给，别在代码里写死。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "framework/algorithm/math/angle.hpp"
#include "framework/algorithm/controller/pid.hpp"
#include "framework/msg/keyboard_state.hpp"

using namespace algorithm;

/**
 * @brief 底盘节点：第一人称云台视角驾驶，按枪管方向分配四轮目标转速
 */
class ChassisNode : public rclcpp::Node
{
public:
    explicit ChassisNode(const std::string& node_name = "chassis", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        read_parameters();
        validate_wheel_parameters();
        configure_follow_pid();
        build_wheel_geometry();
        setup_ros_interfaces();
        start_control_timer();
    }

private:
    /**
     * @brief 话题名：订阅和发布的 ROS 接口
     */
    struct TopicParams
    {
        std::string joint_states {"/joint_states"};
        std::string keyboard {"/keyboard"};
        std::string motor_prefix {"/motor"};
    };

    /**
     * @brief 关节名：从 joint_states 里认这两个关节取枪管偏角
     */
    struct JointParams
    {
        std::string big_yaw {"big_yaw_joint"};
        std::string small_yaw {"small_yaw_joint"};
    };

    /**
     * @brief 轮子几何：名字、安装位置和轮轴朝向角
     */
    struct WheelParams
    {
        std::vector<std::string> names;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> yaw_deg;
    };

    /**
     * @brief 控制周期
     */
    struct TimingParams
    {
        double control_period_s {0.001};
    };

    /**
     * @brief 驾驶输入：前进和横移的速度上限
     */
    struct DriveInputParams
    {
        double forward_speed {0.2};
        double strafe_speed {0.2};
    };

    /**
     * @brief 小陀螺自转的 yaw 角速度
     */
    struct SpinModeParams
    {
        double yaw_rate {2.0 * M_PI};
    };

    /**
     * @brief Ctrl 长按跟随：夹角 PID 输出底盘 yaw 角速度
     */
    struct FollowParams
    {
        double kp {3.0};
        double ki {0.0};
        double kd {0.0};
        double rate_limit {M_PI};
        double integral_limit {0.0};
        double dead_zone {0.5 * M_PI / 180.0};
        double input_timeout_s {0.2};
    };

    /**
     * @brief 轮速环：增益、力矩限幅和轮径
     */
    struct WheelControlParams
    {
        double kp {0.5};
        double torque_limit {0.3};
        double radius {0.04};
    };

    /**
     * @brief 单个轮子的安装几何和当前状态
     */
    struct WheelState
    {
        double ux {0.0};  // 驱动方向（底盘系单位向量）
        double uy {0.0};
        double m {0.0};   // 旋转力臂：轮速 -> 底盘 yaw 的贡献系数
        double dq {0.0};  // 当前轮速（rad/s）
    };

    /**
     * @brief 云台两轴角度采样
     */
    struct GimbalSample
    {
        double q_big {0.0};
        double q_small {0.0};
    };

    /**
     * @brief 底盘目标速度：枪管偏角和分配到轮子前的 vx/vy/wz
     */
    struct VelocityCommand
    {
        double barrel_angle {0.0};
        double vx {0.0};
        double vy {0.0};
        double wz {0.0};
    };

    /**
     * @brief 单个轮子的目标转速、当前转速和算出的力矩
     */
    struct WheelCommand
    {
        double dq_des {0.0};
        double tau {0.0};
        double dq {0.0};
    };

    TimingParams       timing_params_        {};
    DriveInputParams   drive_input_params_   {};
    SpinModeParams     spin_params_          {};
    JointParams        joint_params_         {};
    TopicParams        topic_params_         {};
    WheelParams        wheel_params_         {};
    FollowParams       follow_params_        {};
    WheelControlParams wheel_control_params_ {};

    controller::Pid<double> follow_pid_ {};

    bool follow_hold_ {false};
    bool follow_was_active_ {false};
    double follow_target_ {0.0};
    std::chrono::steady_clock::time_point keyboard_time_ {};

    GimbalSample gimbal_ {};
    std::vector<WheelState> wheels_;
    bool joint_states_seen_ {false};
    double forward_input_ {0.0};
    double strafe_input_ {0.0};
    bool spin_active_ {false};

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> wheel_pubs_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr       joint_states_sub_;
    rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr      keyboard_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief 把值对称限幅到 ±|limit|
     *
     * @param v 输入值
     * @param limit 限幅幅值
     * @return double 限幅后的值
     */
    static double clamp_abs(double v, double limit)
    {
        return std::clamp(v, -std::abs(limit), std::abs(limit));
    }

    /**
     * @brief 小写字母键在 key_down 数组里的下标
     *
     * @param key 小写字母键
     * @return std::size_t 下标
     */
    static constexpr std::size_t key_index(char key)
    {
        return static_cast<std::size_t>(key - 'a');
    }

    /**
     * @brief 从 ROS 参数读入上面几组参数
     */
    void read_parameters()
    {
        timing_params_.control_period_s = declare_parameter("control_period_s", 0.001);

        drive_input_params_.forward_speed = declare_parameter("forward_speed", 0.2);
        drive_input_params_.strafe_speed = declare_parameter("strafe_speed", drive_input_params_.forward_speed);

        spin_params_.yaw_rate = declare_parameter("spin_yaw_rate_deg", 360.0) * M_PI / 180.0;

        follow_params_.kp              = declare_parameter("follow_kp",                     3.0);
        follow_params_.ki              = declare_parameter("follow_ki",                     0.0);
        follow_params_.kd              = declare_parameter("follow_kd",                     0.0);
        follow_params_.rate_limit      = declare_parameter("follow_rate_limit_deg_s",       180.0) * M_PI / 180.0;
        follow_params_.integral_limit  = declare_parameter("follow_integral_limit_deg_s",   0.0) * M_PI / 180.0;
        follow_params_.dead_zone       = declare_parameter("follow_dead_zone_deg",          0.5) * M_PI / 180.0;
        follow_params_.input_timeout_s = declare_parameter("follow_input_timeout_s",        0.2);

        wheel_control_params_.kp       = declare_parameter("kp_wheel",                      0.5);
        wheel_control_params_.radius   = declare_parameter("wheel_radius",                  0.04);
        wheel_control_params_.torque_limit = declare_parameter("wheel_torque_limit",        0.3);

        joint_params_.big_yaw          = declare_parameter("big_yaw_joint",                 "big_yaw_joint");
        joint_params_.small_yaw        = declare_parameter("small_yaw_joint",               "small_yaw_joint");

        topic_params_.joint_states     = declare_parameter("joint_states_topic",            "/joint_states");
        topic_params_.keyboard         = declare_parameter("keyboard_topic",                "/keyboard");
        topic_params_.motor_prefix     = declare_parameter("motor_topic_prefix",            "/motor");

        wheel_params_.names = declare_parameter("wheel_names", std::vector<std::string> {
                                                 "front_left_wheel", 
                                                 "front_right_wheel", 
                                                 "rear_left_wheel", 
                                                 "rear_right_wheel" });
        wheel_params_.x = declare_parameter("wheel_x", std::vector<double>{0.08, 0.08, -0.08, -0.08});
        wheel_params_.y = declare_parameter("wheel_y", std::vector<double>{0.08, -0.08, 0.08, -0.08});
        wheel_params_.yaw_deg = declare_parameter("wheel_yaw_deg", std::vector<double>{135.0, 45.0, -135.0, -45.0});
    }

    /**
     * @brief 检查轮子几何数组长度一致、控制周期和轮径为正
     */
    void validate_wheel_parameters() const
    {
        const std::size_t n = wheel_params_.names.size();
        if (wheel_params_.x.size() != n || wheel_params_.y.size() != n || wheel_params_.yaw_deg.size() != n)
        {
            throw std::runtime_error("chassis: wheel_names / wheel_x / wheel_y / wheel_yaw_deg 数量对不上");
        }
        if (!(timing_params_.control_period_s > 0.0) || !(wheel_control_params_.radius > 0.0))
        {
            throw std::runtime_error("chassis: control_period_s / wheel_radius must be positive");
        }
    }

    /**
     * @brief 配置夹角 PID，正夹角需要底盘正转，因此将 PID 输出反号
     */
    void configure_follow_pid()
    {
        if (!(follow_params_.rate_limit > 0.0)      || !std::isfinite(follow_params_.rate_limit) ||
            !(follow_params_.input_timeout_s > 0.0) || !std::isfinite(follow_params_.input_timeout_s))
        {
            throw std::runtime_error("chassis: follow rate limit / input timeout must be positive and finite");
        }
        controller::Pid<double>::Params params;
        params.kp = follow_params_.kp;
        params.ki = follow_params_.ki;
        params.kd = follow_params_.kd;
        params.dt = timing_params_.control_period_s;
        params.dead_zone = follow_params_.dead_zone;
        params.output_limit = follow_params_.rate_limit;
        params.integral_limit = follow_params_.integral_limit;
        follow_pid_.configure(params);
    }

    /**
     * @brief Ctrl 跟随最近的直角方向，优先于 Shift；退出或切换方向时清除 PID 历史
     * @param barrel_angle 云台相对底盘偏角，rad
     * @return double 底盘目标 yaw 角速度，rad/s
     */
    double compute_yaw_rate_command(double barrel_angle)
    {
        const bool input_fresh = std::chrono::duration<double>(std::chrono::steady_clock::now() - keyboard_time_).count() <= follow_params_.input_timeout_s;
        const bool following   = follow_hold_ && input_fresh;

        // 全向底盘四个轮间方向等价，选择最近的 90 度整数倍，避免绕远归零。
        const double quarter_turn = M_PI / 2.0;
        const double nearest_target = math::normalize_radian_pm_pi(std::round(barrel_angle / quarter_turn) * quarter_turn);

        if (following != follow_was_active_ || (following && nearest_target != follow_target_))
        {
            follow_target_ = nearest_target;
            follow_pid_.reset(barrel_angle, follow_target_);
            follow_was_active_ = following;
            RCLCPP_INFO(get_logger(), "底盘跟随%s：夹角 %.2f deg，最近目标 %.2f deg", following ? "开启" : "退出", barrel_angle * 180.0 / M_PI, follow_target_ * 180.0 / M_PI);
        }
        if (following)
        {
            // alpha = yaw_gimbal - yaw_base，底盘正转会使 alpha 减小。
            // PID 使用 target - measurement，故反号后才是正确的底盘转向。
            const double wz = -follow_pid_.update_angle(barrel_angle, follow_target_);
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 100, "底盘跟随 angle_deg=%.3f target_deg=%.3f wz_rad_s=%.4f", barrel_angle * 180.0 / M_PI, follow_target_ * 180.0 / M_PI, wz);
            return wz;
        }
        return input_fresh && spin_active_ ? spin_params_.yaw_rate : 0.0;
    }

    /**
     * @brief 按参数里的安装几何建每个轮子的驱动方向和旋转力臂
     */
    void build_wheel_geometry()
    {
        wheels_.clear();
        wheels_.resize(wheel_params_.names.size());
        wheel_pubs_.clear();

        for (std::size_t i = 0; i < wheels_.size(); ++i)
        {
            wheels_[i] = compute_wheel_state(i);
        }
    }

    /**
     * @brief 由第 i 个轮子的轮轴朝向和安装位置算出驱动方向和旋转力臂
     *
     * @param i 轮子下标
     * @return WheelState 该轮子的几何
     */
    WheelState compute_wheel_state(std::size_t i) const
    {
        const double yaw = wheel_params_.yaw_deg[i] * M_PI / 180.0;
        const double ax = -std::sin(yaw);  // Rz(yaw) * (0,1,0) = (-sin, cos, 0)
        const double ay =  std::cos(yaw);

        const double ux = ay;   // u = -(z_hat x a) = (ay, -ax)
        const double uy = -ax;
        const double norm = std::hypot(ux, uy);

        WheelState wheel;
        wheel.ux = ux / norm;
        wheel.uy = uy / norm;
        wheel.m = wheel.ux * (-wheel_params_.y[i]) + wheel.uy * wheel_params_.x[i];  // u dot (z_hat x r)
        return wheel;
    }

    /**
     * @brief 订阅关节和键盘，按轮子名各建一个力矩发布者
     */
    void setup_ros_interfaces()
    {
        joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            topic_params_.joint_states, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_states(msg); });

        keyboard_sub_ = create_subscription<framework::msg::KeyboardState>(
            topic_params_.keyboard, 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg) { on_keyboard(msg); });

        for (const auto& name : wheel_params_.names)
        {
            wheel_pubs_.push_back(create_publisher<std_msgs::msg::Float64>(topic_params_.motor_prefix + "/" + name + "/cmd_force", 10));
        }
    }

    /**
     * @brief 按控制周期起控制定时器
     */
    void start_control_timer()
    {
        timer_ = create_wall_timer(std::chrono::duration<double>(timing_params_.control_period_s), std::bind(&ChassisNode::control_tick, this));
    }

    /**
     * @brief WASD 取出前进/横移输入，spin_active 或 shift 按住算小陀螺
     *
     * @param msg 键盘状态消息
     */
    void on_keyboard(const framework::msg::KeyboardState::SharedPtr& msg)
    {
        const auto is_down = [msg](char key) {
            return msg->key_down[key_index(key)];
        };

        forward_input_ = (is_down('w') ? 1.0 : 0.0) - (is_down('s') ? 1.0 : 0.0);
        strafe_input_  = (is_down('a') ? 1.0 : 0.0) - (is_down('d') ? 1.0 : 0.0);
        spin_active_   = msg->spin_active || msg->shift_hold;
        follow_hold_   = msg->ctrl_hold;
        keyboard_time_ = std::chrono::steady_clock::now();
    }

    /**
     * @brief 取云台两轴角度和各轮当前转速；三者都收到才算这一拍可用
     *
     * @param msg 关节状态消息
     */
    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr& msg)
    {
        bool saw_any_wheel = false;
        bool saw_big       = false;
        bool saw_small     = false;

        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == joint_params_.big_yaw)
            {
                gimbal_.q_big = msg->position[i];
                saw_big = true;
            }
            else if (msg->name[i] == joint_params_.small_yaw)
            {
                gimbal_.q_small = msg->position[i];
                saw_small = true;
            }
            else
            {
                saw_any_wheel = update_wheel_speed_from_joint(msg->name[i], msg->velocity[i]) || saw_any_wheel;
            }
        }

        joint_states_seen_ = saw_big && saw_small && saw_any_wheel;
    }

    /**
     * @brief 按 "<轮子名>_joint" 匹配，把该关节的转速写进对应轮子
     *
     * @param joint_name 关节名
     * @param dq 该关节的角速度
     * @return bool 是否匹配上某个轮子
     */
    bool update_wheel_speed_from_joint(const std::string& joint_name, double dq)
    {
        for (std::size_t i = 0; i < wheel_params_.names.size(); ++i)
        {
            if (joint_name == wheel_params_.names[i] + "_joint")
            {
                wheels_[i].dq = dq;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief 定时器里跑的一拍：算底盘目标速度、分配四轮目标转速、出力矩并发出
     */
    void control_tick()
    {
        if (!joint_states_seen_)
        {
            return;
        }

        // 目标速度：键盘输入先在枪管系，再按枪管相对底盘的偏角 q_big + q_small 旋到底盘系
        VelocityCommand velocity;
        velocity.barrel_angle = math::normalize_radian_pm_pi(gimbal_.q_big + gimbal_.q_small);

        const double  vx_barrel = forward_input_ * drive_input_params_.forward_speed;
        const double  vy_barrel = strafe_input_ * drive_input_params_.strafe_speed;
        velocity.vx = vx_barrel * std::cos(velocity.barrel_angle) - vy_barrel * std::sin(velocity.barrel_angle);
        velocity.vy = vx_barrel * std::sin(velocity.barrel_angle) + vy_barrel * std::cos(velocity.barrel_angle);

        // Ctrl 长按最近直角方向对齐优先，其次 Shift 自转；普通驾驶不给旋转目标。
        velocity.wz = compute_yaw_rate_command(velocity.barrel_angle);

        // 按轮子安装几何分配到各轮目标转速，每轮一个速度 PD
        std::vector<WheelCommand> wheel_cmds(wheels_.size());
        for (std::size_t i = 0; i < wheels_.size(); ++i)
        {
            wheel_cmds[i].dq_des = (wheels_[i].ux * velocity.vx + wheels_[i].uy * velocity.vy + wheels_[i].m * velocity.wz) / wheel_control_params_.radius;
            wheel_cmds[i].dq     = wheels_[i].dq;
            wheel_cmds[i].tau    = std::clamp(wheel_control_params_.kp * (wheel_cmds[i].dq_des - wheel_cmds[i].dq), -wheel_control_params_.torque_limit, wheel_control_params_.torque_limit);
        }

        std_msgs::msg::Float64 msg;
        const std::size_t n_pub = std::min(wheel_cmds.size(), wheel_pubs_.size());
        for (std::size_t i = 0; i < n_pub; ++i)
        {
            msg.data = wheel_cmds[i].tau;
            wheel_pubs_[i]->publish(msg);
        }
    }
};
