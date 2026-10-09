/**
 * @file sim.hpp
 * @author qingyu
 * @brief MuJoCo 仿真节点：跑物理、收每关节一路力矩写进 data->ctrl、发 /joint_states、
 *        /imu（底盘）和 /gimbal/imu（云台 pitch_link），界面就是 MuJoCo 自带的 simulate
 *        （面板、暂停、鼠标、扰动都在）
 * @version 0.1
 * @date 2026-10-02
 *
 * @copyright Copyright (c) 2026
 *
 * @note 线程布局照抄 MuJoCo 自己的 simulate/main.cc：界面（mujoco::Simulate）在**主线程**跑
 *       RenderLoop()，物理在**另一个线程**，两边靠 Simulate::mtx 串起来。UI 的开/关/暂停
 *       按钮、keyframe、历史、扰动都是 Simulate 自己管的，我们只负责把力矩写进 ctrl
 * @note 为什么不用 bin/simulate 那个可执行文件：它是独立进程、跑自己的一份物理，外面没法把
 *       力矩塞进去。MuJoCo 不把 simulate 发成库，官方集成包（ros-controls/mujoco_ros2_control）
 *       的做法也是"直接从本地 MuJoCo 安装编 simulate 源码"，这里跟着它走
 * @note 话题接线全在参数里：模型从 model 参数给的路径加载（launch 把世界和机器人拼成一份临时文件），
 *       执行器按 <cmd_force_topic_prefix>/<执行器名>/cmd_force 订阅，关节状态发到 joint_states_topic
 * @note 拖文件换模型、UI 上那个 Reload 没接：换了模型我们缓存的关节下标就得重建，先不做
 */

#pragma once

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "framework/msg/keyboard_state.hpp"

#include <simulate/glfw_adapter.h>
#include <simulate/simulate.h>

/**
 * @brief MuJoCo 仿真节点：一个模型、一路物理，力矩进、关节状态出，界面用 simulate
 *
 * @note 执行器索引就是 ctrl 的下标，跟模型 <actuator> 段里的先后顺序一致（不是关节名）
 * @note 物理线程里跑 ROS：订阅回调（写 ctrl）跟 mj_step 在同一线程，ctrl 不会一边写一边读
 * @note 左键平移、右键旋转：改的是 MuJoCo 源码里 platform_ui_adapter.cc 那两处对调
 *       （原来是"按住 Alt 才换"，现在默认就换），所以这里用普通的 GlfwAdapter
 */
class SimNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造仿真节点：加载模型、起界面、接上话题
     *
     * @param node_name 节点名
     * @param options 节点选项
     *
     * @throw std::runtime_error model 参数没给、或者模型加载失败时抛，别静默跑一个空模型
     *
     * @note 必须在主线程构造：界面（GLFW）下面的调用都得跟 RenderLoop 同一条线程
     */
    explicit SimNode(const std::string& node_name = "sim", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        const std::string model_path = declare_parameter("model", "");
        if (model_path.empty())
        {
            throw std::runtime_error("sim: 参数 model 没给（模型文件的绝对路径）");
        }

        char error[1024] = {};
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
        if (model_ == nullptr)
        {
            throw std::runtime_error(std::string("sim: 加载模型失败：") + error);
        }
        data_ = mj_makeData(model_);
        setup_projectiles();

        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);
        mjv_defaultPerturb(&perturb_);

        // 界面：跟 bin/simulate 用的是同一套 Simulate
        sim_ = std::make_unique<mujoco::Simulate>(
            std::make_unique<mujoco::GlfwAdapter>(), &camera_, &option_, &perturb_, /* is_passive = */ false);

        // 相机先挂到机器人本体上（模型里那个自由关节所在的 body）
        for (int i = 0; i < model_->njnt; ++i)
        {
            if (model_->jnt_type[i] == mjJNT_FREE)
            {
                camera_.type        = mjCAMERA_TRACKING;
                camera_.trackbodyid = model_->jnt_bodyid[i];
                break;
            }
        }
        camera_.distance  = declare_parameter("camera_distance", 1.5);
        camera_.azimuth   = declare_parameter("camera_azimuth_deg", 90.0);
        camera_.elevation = declare_parameter("camera_elevation_deg", -15.0);
        camera_.lookat[0] = declare_parameter("camera_lookat_x", 0.0);
        camera_.lookat[1] = declare_parameter("camera_lookat_y", 0.0);
        camera_.lookat[2] = declare_parameter("camera_lookat_z", 0.0);

        // 模型交给界面这一步（sim_->Load）**不能在这里做**：它会阻塞等渲染线程接手，
        // 而这时 RenderLoop 还没跑起来。放进 physics_loop 的开头，跟上游 main.cc 一样
        model_path_ = model_path;

        const std::string prefix            = declare_parameter("cmd_force_topic_prefix", "/motor");
        const std::string joint_state_topic = declare_parameter("joint_states_topic", "/joint_states");
        const bool enable_cmd_force_topics  = declare_parameter("enable_cmd_force_topics", true);

        // 一路执行器一条力矩话题，名字按执行器名推出来，不用再维护一份顺序表
        if (enable_cmd_force_topics)
        {
            for (int i = 0; i < model_->nu; ++i)
            {
                const char* actuator_name = mj_id2name(model_, mjOBJ_ACTUATOR, i);
                if (actuator_name == nullptr)
                {
                    throw std::runtime_error("sim: 模型里有没名字的执行器，话题没法接");
                }
                const std::string topic = prefix + "/" + actuator_name + "/cmd_force";

                // 只记最新一条：物理这一拍用哪个值由物理线程决定，跟话题频率解耦
                cmd_subs_.push_back(create_subscription<std_msgs::msg::Float64>(
                    topic, 10,
                    [this, index = static_cast<std::size_t>(i)](const std_msgs::msg::Float64::SharedPtr msg) {
                        data_->ctrl[index] = msg->data;
                        cmd_received_[index] = msg->data;
                        cmd_seen_[index]  = true;
                        cmd_time_[index]  = std::chrono::steady_clock::now();
                    }));
            }
        }

        // 关节状态只报单自由度的关节（hinge / slide）：自由关节的 qpos 是七个数，
        // 塞进 JointState 的一条 name 里没意义
        for (int i = 0; i < model_->njnt; ++i)
        {
            const int type = model_->jnt_type[i];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            {
                continue;
            }
            const char* joint_name = mj_id2name(model_, mjOBJ_JOINT, i);
            if (joint_name == nullptr)
            {
                continue;
            }
            joints_.push_back(JointRef{joint_name, model_->jnt_qposadr[i], model_->jnt_dofadr[i]});
        }

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, rclcpp::SensorDataQoS());
        base_velocity_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/base_velocity", rclcpp::SensorDataQoS());
        gimbal_camera_.topic = declare_parameter("gimbal_camera_topic", "/gimbal/camera/image_raw");
        gimbal_camera_.name = declare_parameter("gimbal_camera_name", "pitch_camera");
        gimbal_camera_.width = declare_parameter("gimbal_camera_width", 640);
        gimbal_camera_.height = declare_parameter("gimbal_camera_height", 360);
        gimbal_camera_.publish_period_s = declare_parameter("gimbal_camera_period_s", 1.0 / 30.0);
        gimbal_camera_.pub = create_publisher<sensor_msgs::msg::Image>(gimbal_camera_.topic, rclcpp::SensorDataQoS());
        overhead_camera_.name = declare_parameter("overhead_camera_name", "pitch_overhead_camera");
        overhead_camera_.topic = declare_parameter("overhead_camera_topic", "/chassis/camera/image_raw");
        overhead_camera_.width = declare_parameter("overhead_camera_width", 640);
        overhead_camera_.height = declare_parameter("overhead_camera_height", 480);
        overhead_camera_.pub = create_publisher<sensor_msgs::msg::Image>(overhead_camera_.topic, rclcpp::SensorDataQoS());
        for (int i = 0; i < model_->njnt; ++i)
        {
            if (model_->jnt_type[i] == mjJNT_FREE)
            {
                root_dof_adr_ = model_->jnt_dofadr[i];
                break;
            }
        }

        // IMU：按 <前缀>_quat/_gyro/_accel 三个传感器名找一路，找到几路发几路。
        // 模型没挂就只发关节状态，把缺哪路说清楚。
        // 只有云台那一路：底盘不挂 IMU，`/imu` 已删
        add_imu_channel("gimbal_imu", declare_parameter("gimbal_imu_topic", "/gimbal/imu"), "pitch_link");

        pub_period_ = declare_parameter("publish_period_s", 0.001);

        // 力矩超时：没了 motor 节点中转，掉线松力矩这道安全网挪到这里。
        // 每一路执行器独立计时，收到过指令才开始盯
        cmd_timeout_s_ = declare_parameter("cmd_timeout_s", 0.05);
        cmd_seen_.assign(static_cast<std::size_t>(model_->nu), false);
        cmd_received_.assign(static_cast<std::size_t>(model_->nu), 0.0);
        cmd_time_.resize(static_cast<std::size_t>(model_->nu));

        register_model_tuning_parameters();

        RCLCPP_INFO(get_logger(), "仿真就绪：模型 %s，%ld 个执行器、%zu 个关节，步长 %.6f s，力矩话题%s%s",
                    model_path.c_str(), static_cast<long>(model_->nu), joints_.size(), model_->opt.timestep,
                    enable_cmd_force_topics ? "前缀 " : "已关闭",
                    enable_cmd_force_topics ? prefix.c_str() : "");
    }

    /**
     * @brief 析构：先关界面，再放模型和数据
     */
    ~SimNode() override
    {
        sim_.reset();

        if (data_ != nullptr)
        {
            mj_deleteData(data_);
        }
        if (model_ != nullptr)
        {
            mj_deleteModel(model_);
        }
    }

    SimNode(const SimNode&)            = delete;
    SimNode& operator=(const SimNode&) = delete;

    /**
     * @brief 跑界面循环，**阻塞**到窗口关掉
     *
     * @note 必须在主线程调：GLFW 的事件循环只能在建窗口的那条线程上跑
     */
    void render_loop()
    {
        sim_->RenderLoop();

        // RenderLoop 返回（窗口关了）**不会**自己置 exitrequest，得我们来：
        // 不置的话物理线程还在循环里等，主线的 join() 会一直挂住
        sim_->exitrequest.store(1);
    }

    /**
     * @brief 物理线程：推物理 + 收发 ROS，直到窗口关掉
     *
     * @param executor 单线程 executor，节点已经加进去了
     *
     * @note 结构照抄 simulate/main.cc 的 PhysicsLoop：每轮先拿 sim_->mtx，UI 那边
     *       （Simulate::Sync，在渲染线程里）改模型/数据时也拿这把锁
     * @note 暂停（sim_->run == 0）时走 mj_forward 而不是 mj_step：界面上拖关节滑条、
     *       改参数才跟手
     */
    void physics_loop(rclcpp::executors::SingleThreadedExecutor& executor)
    {
        // 把模型交给界面：这一句会阻塞到渲染线程接手为止，所以必须等 RenderLoop 跑起来再调
        sim_->Load(model_, data_, model_path_.c_str());
        {
            const mujoco::MutexLock lock(sim_->mtx);
            mj_forward(model_, data_);
        }
        setup_camera_renderer(gimbal_camera_);
        setup_camera_renderer(overhead_camera_);

        auto   sync_cpu = mujoco::Simulate::Clock::now();
        mjtNum sync_sim = 0.0;
        int    last_run = -1;
        auto   last_publish = mujoco::Simulate::Clock::now();
        auto   last_camera_publish = mujoco::Simulate::Clock::now();

        while (!sim_->exitrequest.load() && rclcpp::ok())
        {
            const auto tick_begin = mujoco::Simulate::Clock::now();

            {
                const mujoco::MutexLock lock(sim_->mtx);

                // 在"跑"和"暂停"之间切换时把计时器清掉，不然界面上的性能面板一直是旧数据
                if (sim_->run != last_run)
                {
                    if (last_run != -1)
                    {
                        std::memset(data_->timer, 0, sizeof(data_->timer));
                        std::memset(sim_->timer_prev_, 0, sizeof(sim_->timer_prev_));
                    }
                    last_run = sim_->run;
                }

                if (sim_->run)
                {
                    const double elapsed_cpu_s = std::chrono::duration<double>(tick_begin - sync_cpu).count();
                    const double elapsed_sim_s = data_->time - sync_sim;
                    const double slowdown      = 100.0 / mujoco::Simulate::percentRealTime[sim_->real_time_index];

                    // 跟墙上时钟对不上了（差超过 kSyncMisalignS）就重同步，这一步单独走
                    const bool misaligned = std::abs(elapsed_cpu_s / slowdown - elapsed_sim_s) > kSyncMisalignS;

                    if (elapsed_sim_s < 0.0 || misaligned || sim_->speed_changed)
                    {
                        sync_cpu            = tick_begin;
                        sync_sim            = data_->time;
                        sim_->speed_changed = false;
                        sim_->InjectNoise(sim_->key);
                        enforce_command_timeout();
                        executor.spin_some();
                        update_projectiles();
                        mj_step(model_, data_);
                    }
                    else
                    {
                        // 追到跟墙上时钟齐为止，但一轮最多占 refresh 周期的 kSimRefreshFraction，
                        // 剩下的时间留给界面刷新，不然窗口会卡住。
                        // 每一步都收一次命令：力矩当拍写进 ctrl，不等下一轮外层循环
                        const double refresh_s = kSimRefreshFraction / sim_->refresh_rate;
                        while (std::chrono::duration<double>(mujoco::Simulate::Clock::now() - sync_cpu).count() >
                                   (data_->time - sync_sim) * slowdown &&
                               std::chrono::duration<double>(mujoco::Simulate::Clock::now() - tick_begin).count() < refresh_s)
                        {
                            sim_->InjectNoise(sim_->key);
                            enforce_command_timeout();
                            executor.spin_some();
                            update_projectiles();
                            mj_step(model_, data_);
                        }
                    }

                    sim_->AddToHistory();
                }
                else
                {
                    mj_forward(model_, data_);
                    sim_->speed_changed = true;
                    executor.spin_some();
                }

                const auto now = mujoco::Simulate::Clock::now();
                if (std::chrono::duration<double>(now - last_publish).count() >= pub_period_)
                {
                    last_publish = now;
                    publish_joint_state();
                    publish_imu();
                    publish_base_velocity();
                }

                const auto camera_now = mujoco::Simulate::Clock::now();
                if (std::chrono::duration<double>(camera_now - last_camera_publish).count() >= gimbal_camera_.publish_period_s)
                {
                    last_camera_publish = camera_now;
                    publish_camera_image(gimbal_camera_);
                    publish_camera_image(overhead_camera_);
                }
            }

            // 外层循环不能睡满 1 ms：1 ms 步长下每轮最多追一步，睡 1 ms 就把仿真钉死在
            // ~0.9× 实时（实测 895 Hz），整条控制链的往返延迟全被它放大。睡一小片，
            // 让循环转得快几倍，追步交给上面的 while，界面刷新有 kSimRefreshFraction 兜着
            std::this_thread::sleep_for(std::chrono::microseconds(200));
        }

        // ROS 被关掉（Ctrl-C）时也得把窗口关掉：RenderLoop 的退出条件是
        // "窗口被关 或 exitrequest"，不置这一下，主线就永远卡在那儿，
        // 外面只能一路 SIGTERM → SIGKILL
        shutdown_camera_renderer(gimbal_camera_);
        shutdown_camera_renderer(overhead_camera_);
        sim_->exitrequest.store(1);
    }

private:
    struct ProjectileState
    {
        int body;
        int geom;
        int qpos;
        int dof;
        bool active {false};
        bool muzzle_reported {false};
    };

    struct LauncherState
    {
        std::vector<ProjectileState> balls;
        int feed_site {-1};
        bool pending {false};
        bool right_held {false};
        std::uint64_t shots {0};
        double last_load {-1.0};
        double last_time {0.0};
        rclcpp::Subscription<framework::msg::KeyboardState>::SharedPtr keyboard;
    } launcher_;

    void setup_projectiles()
    {
        launcher_.feed_site = mj_name2id(model_, mjOBJ_SITE, "pitch_barrel_projectile_feed");
        if (launcher_.feed_site < 0) return;
        for (int i = 0; i < 4; ++i)
        {
            const std::string name = "projectile_" + std::to_string(i);
            const int body = mj_name2id(model_, mjOBJ_BODY, name.c_str());
            const int geom = mj_name2id(model_, mjOBJ_GEOM, (name + "_geom").c_str());
            const int joint = mj_name2id(model_, mjOBJ_JOINT, (name + "_joint").c_str());
            if (body < 0 || geom < 0 || joint < 0) continue;
            launcher_.balls.push_back({body, geom, model_->jnt_qposadr[joint], model_->jnt_dofadr[joint]});
        }
        launcher_.keyboard = create_subscription<framework::msg::KeyboardState>(declare_parameter("keyboard_topic", "/keyboard"), 10,
            [this](const framework::msg::KeyboardState::SharedPtr msg)
            {
                // 只用按下沿，长按右键不会连发；暂停时仅保留一次装填请求。
                if (msg->mouse_right && !launcher_.right_held) launcher_.pending = true;
                launcher_.right_held = msg->mouse_right;
            });
    }

    void update_projectiles()
    {
        if (launcher_.feed_site < 0) return;
        const bool reset = data_->time < launcher_.last_time;
        launcher_.last_time = data_->time;
        for (auto& ball : launcher_.balls)
        {
            if (ball.active && !ball.muzzle_reported && !reset)
            {
                const mjtNum* rotation = data_->site_xmat + 9 * launcher_.feed_site;
                const mjtNum axis[3] {rotation[0], rotation[3], rotation[6]};
                mjtNum offset[3];
                mju_sub3(offset, data_->xpos + 3 * ball.body, data_->site_xpos + 3 * launcher_.feed_site);
                // 装填点 x=0.050，枪口 x=0.109；以弹丸中心越过枪口为测量时刻。
                if (mju_dot3(offset, axis) >= 0.059)
                {
                    const int parent = model_->site_bodyid[launcher_.feed_site];
                    mjtNum parent_velocity[6], ball_velocity[6], arm[3], tangential[3], relative[3];
                    mj_objectVelocity(model_, data_, mjOBJ_BODY, parent, parent_velocity, 0);
                    mj_objectVelocity(model_, data_, mjOBJ_BODY, ball.body, ball_velocity, 0);
                    mju_sub3(arm, data_->xpos + 3 * ball.body, data_->xipos + 3 * parent);
                    mju_cross(tangential, parent_velocity, arm);
                    mju_sub3(relative, ball_velocity + 3, parent_velocity + 3);
                    mju_sub3(relative, relative, tangential);
                    RCLCPP_INFO(get_logger(), "弹丸枪口轴向速度=%.2f m/s（相对枪管）", mju_dot3(relative, axis));
                    ball.muzzle_reported = true;
                }
            }
            if (reset) ball.active = false;
            if (!ball.active)
            {
                model_->body_contype[ball.body] = 0;
                model_->body_conaffinity[ball.body] = 0;
                model_->geom_contype[ball.geom] = 0;
                model_->geom_conaffinity[ball.geom] = 0;
                mju_copy(data_->qpos + ball.qpos, model_->qpos0 + ball.qpos, 7);
                mju_zero(data_->qvel + ball.dof, 6);
            }
        }
        if (reset)
        {
            launcher_.pending = false;
            launcher_.last_load = -1.0;
            launcher_.shots = 0;
        }
        if (!launcher_.pending) return;
        launcher_.pending = false;
        if (data_->time - launcher_.last_load < 0.15) return;
        mj_forward(model_, data_);
        const mjtNum* feed = data_->site_xpos + 3 * launcher_.feed_site;
        // 装填点尚有弹丸时拒绝重叠装填。
        for (const auto& ball : launcher_.balls)
        {
            if (ball.active && mju_dist3(data_->xpos + 3 * ball.body, feed) < 0.025) return;
        }
        if (launcher_.balls.size() != 4) return;
        {
            // 第 3 次装填回收第 1 颗，第 4 次回收第 2 颗；最多保留两发在场。
            if (launcher_.shots >= 2)
            {
                auto& recycled = launcher_.balls[(launcher_.shots - 2) % 4];
                recycled.active = false;
                recycled.muzzle_reported = false;
                model_->body_contype[recycled.body] = 0;
                model_->body_conaffinity[recycled.body] = 0;
                model_->geom_contype[recycled.geom] = 0;
                model_->geom_conaffinity[recycled.geom] = 0;
                mju_copy(data_->qpos + recycled.qpos, model_->qpos0 + recycled.qpos, 7);
                mju_zero(data_->qvel + recycled.dof, 6);
            }
            auto& ball = launcher_.balls[launcher_.shots % 4];
            mju_copy3(data_->qpos + ball.qpos, feed);
            data_->qpos[ball.qpos + 3] = 1.0;
            mju_zero(data_->qpos + ball.qpos + 4, 3);
            // 继承装填点的刚体速度；不添加枪口速度，由摩擦轮接触加速。
            const int parent = model_->site_bodyid[launcher_.feed_site];
            mjtNum velocity[6], offset[3], tangential[3];
            mj_objectVelocity(model_, data_, mjOBJ_BODY, parent, velocity, 0);
            mju_sub3(offset, feed, data_->xipos + 3 * parent);
            mju_cross(tangential, velocity, offset);
            mju_add3(data_->qvel + ball.dof, velocity + 3, tangential);
            mju_copy3(data_->qvel + ball.dof + 3, velocity);
            model_->body_contype[ball.body] = 1;
            model_->body_conaffinity[ball.body] = 1;
            model_->geom_contype[ball.geom] = 1;
            model_->geom_conaffinity[ball.geom] = 1;
            ball.active = true;
            ball.muzzle_reported = false;
            launcher_.last_load = data_->time;
            ++launcher_.shots;
            mj_forward(model_, data_);
            RCLCPP_INFO(get_logger(), "装填 17mm 弹丸：%s", mj_id2name(model_, mjOBJ_BODY, ball.body));
            return;
        }
    }

    // 界面：Simulate 存的是这三个的引用，它们得比 sim_ 先构造、后析构
    mjvCamera  camera_ {};
    mjvOption  option_ {};
    mjvPerturb perturb_ {};
    std::unique_ptr<mujoco::Simulate> sim_;

    /**
     * @brief 要让下游看见的一个关节：名字 + 它在 qpos / qvel 里的位置
     */
    struct JointRef
    {
        std::string name;
        // mjModel 里的下标是 int，这里只能跟着它
        int qpos_adr {};
        int dof_adr {};
    };

    struct GeomParameterRef
    {
        int  geom_id {};
        bool euler_deg {false};
    };

    struct SiteParameterRef
    {
        int  site_id {};
        bool euler_deg {false};
    };

    struct JointAxisParameterRef
    {
        int joint_id {};
    };

    double pub_period_ {0.001};

    // 力矩超时：超过 cmd_timeout_s_ 没新指令就把那一路 ctrl 清零（收到过指令才盯）
    double                                     cmd_timeout_s_ {0.05};
    std::vector<bool>                          cmd_seen_;
    std::vector<double>                        cmd_received_;
    std::chrono::steady_clock::time_point yaw_diagnostic_time_ {};
    std::vector<std::chrono::steady_clock::time_point> cmd_time_;

    std::string model_path_;

    mjModel* model_ {nullptr};
    mjData*  data_ {nullptr};

    std::vector<JointRef> joints_;
    std::unordered_map<std::string, GeomParameterRef> geom_param_refs_;
    std::unordered_map<std::string, SiteParameterRef> site_param_refs_;
    std::unordered_map<std::string, JointAxisParameterRef> joint_axis_param_refs_;

    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> cmd_subs_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr           joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr     base_velocity_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr    model_param_callback_;

    struct CameraChannel
    {
        std::string name {"pitch_camera"};
        std::string topic {"/gimbal/camera/image_raw"};
        int width {640};
        int height {360};
        double publish_period_s {1.0 / 30.0};
        int camera_id {-1};
        GLFWwindow* window {nullptr};
        mjvScene scene {};
        mjrContext context {};
        bool ready {false};
        std::vector<unsigned char> rgb;
        std::vector<unsigned char> rgb_flipped;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    };
    CameraChannel gimbal_camera_ {};
    CameraChannel overhead_camera_ {};

    // 一路 IMU：三个传感器在 sensordata 里的起点（-1 表示模型里没有）+ 发布出口
    struct ImuChannel
    {
        int quat_adr {-1};
        int gyro_adr {-1};
        int accel_adr {-1};
        std::string frame_id;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub;
    };
    std::vector<ImuChannel> imu_channels_;

    int root_dof_adr_ {-1};

    // 跟墙上时钟对不上的阈值，超过就重新同步（仿真秒）
    static constexpr double kSyncMisalignS = 0.1;
    // 每一轮最多把 refresh 周期的这个比例用来推物理，剩下的留给界面
    static constexpr double kSimRefreshFraction = 0.7;

    /**
     * @brief 暴露运行时模型调参入口：
     *        geom.<geom_name>.pos / geom.<geom_name>.euler_deg
     *        site.<site_name>.pos / site.<site_name>.euler_deg / joint.<joint_name>.axis
     *
     * @note pos 单位是 m，euler_deg 单位是 degree，顺序固定按 xyz；axis 是三维方向向量。
     * @note 参数回调由 physics_loop 里的 executor.spin_some() 调用，此时已经拿着 sim_->mtx，
     *       所以这里直接改 mjModel 并 mj_forward，不再重复加锁。
     */
    void register_model_tuning_parameters()
    {
        for (int i = 0; i < model_->ngeom; ++i)
        {
            const char* geom_name = mj_id2name(model_, mjOBJ_GEOM, i);
            if (geom_name == nullptr)
            {
                continue;
            }

            const std::string base = std::string("geom.") + geom_name;
            const std::string pos_name = base + ".pos";
            const std::string euler_name = base + ".euler_deg";

            std::vector<double> pos {
                model_->geom_pos[3 * i + 0],
                model_->geom_pos[3 * i + 1],
                model_->geom_pos[3 * i + 2],
            };
            declare_parameter(pos_name, pos);
            declare_parameter(euler_name, geom_euler_deg(i));

            geom_param_refs_[pos_name] = GeomParameterRef{i, false};
            geom_param_refs_[euler_name] = GeomParameterRef{i, true};
        }

        for (int i = 0; i < model_->nsite; ++i)
        {
            const char* site_name = mj_id2name(model_, mjOBJ_SITE, i);
            if (site_name == nullptr)
            {
                continue;
            }

            const std::string base = std::string("site.") + site_name;
            const std::string pos_name = base + ".pos";
            const std::string euler_name = base + ".euler_deg";

            std::vector<double> pos {
                model_->site_pos[3 * i + 0],
                model_->site_pos[3 * i + 1],
                model_->site_pos[3 * i + 2],
            };
            declare_parameter(pos_name, pos);
            declare_parameter(euler_name, site_euler_deg(i));

            site_param_refs_[pos_name] = SiteParameterRef{i, false};
            site_param_refs_[euler_name] = SiteParameterRef{i, true};
        }

        for (int i = 0; i < model_->njnt; ++i)
        {
            const int type = model_->jnt_type[i];
            if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
            {
                continue;
            }

            const char* joint_name = mj_id2name(model_, mjOBJ_JOINT, i);
            if (joint_name == nullptr)
            {
                continue;
            }

            const std::string axis_name = std::string("joint.") + joint_name + ".axis";
            std::vector<double> axis {
                model_->jnt_axis[3 * i + 0],
                model_->jnt_axis[3 * i + 1],
                model_->jnt_axis[3 * i + 2],
            };
            declare_parameter(axis_name, axis);
            joint_axis_param_refs_[axis_name] = JointAxisParameterRef{i};
        }

        model_param_callback_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                return on_model_parameters(params);
            });
    }

    std::vector<double> geom_euler_deg(int geom_id) const
    {
        const mjtNum* q = model_->geom_quat + 4 * geom_id;
        return quat_euler_deg(q);
    }

    std::vector<double> site_euler_deg(int site_id) const
    {
        const mjtNum* q = model_->site_quat + 4 * site_id;
        return quat_euler_deg(q);
    }

    static std::vector<double> quat_euler_deg(const mjtNum* q)
    {
        const double w = q[0];
        const double x = q[1];
        const double y = q[2];
        const double z = q[3];

        const double r00 = 1.0 - 2.0 * (y * y + z * z);
        const double r10 = 2.0 * (x * y + z * w);
        const double r20 = 2.0 * (x * z - y * w);
        const double r21 = 2.0 * (y * z + x * w);
        const double r22 = 1.0 - 2.0 * (x * x + y * y);

        const double pitch = std::asin(std::clamp(-r20, -1.0, 1.0));
        const double roll = std::atan2(r21, r22);
        const double yaw = std::atan2(r10, r00);
        constexpr double kRadToDeg = 180.0 / M_PI;
        return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
    }

    static bool parameter_vector3(const rclcpp::Parameter& param, std::vector<double>& out)
    {
        out.clear();
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
            out = param.as_double_array();
        }
        else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY)
        {
            const std::vector<int64_t> ints = param.as_integer_array();
            out.reserve(ints.size());
            for (const int64_t value : ints)
            {
                out.push_back(static_cast<double>(value));
            }
        }
        else
        {
            return false;
        }
        return out.size() == 3;
    }

    rcl_interfaces::msg::SetParametersResult on_model_parameters(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool changed = false;
        for (const rclcpp::Parameter& param : params)
        {
            const auto ref = geom_param_refs_.find(param.get_name());
            if (ref != geom_param_refs_.end())
            {
                std::vector<double> value;
                if (!parameter_vector3(param, value))
                {
                    result.successful = false;
                    result.reason = param.get_name() + " 需要 3 个数字，例如 [0.0, 0.0, 0.025]";
                    return result;
                }

                const int geom_id = ref->second.geom_id;
                if (ref->second.euler_deg)
                {
                    mjtNum euler_rad[3] = {
                        static_cast<mjtNum>(value[0] * M_PI / 180.0),
                        static_cast<mjtNum>(value[1] * M_PI / 180.0),
                        static_cast<mjtNum>(value[2] * M_PI / 180.0),
                    };
                    mju_euler2Quat(model_->geom_quat + 4 * geom_id, euler_rad, "xyz");
                }
                else
                {
                    model_->geom_pos[3 * geom_id + 0] = value[0];
                    model_->geom_pos[3 * geom_id + 1] = value[1];
                    model_->geom_pos[3 * geom_id + 2] = value[2];
                }
                changed = true;
                continue;
            }

            const auto site_ref = site_param_refs_.find(param.get_name());
            if (site_ref != site_param_refs_.end())
            {
                std::vector<double> value;
                if (!parameter_vector3(param, value))
                {
                    result.successful = false;
                    result.reason = param.get_name() + " 需要 3 个数字，例如 [0.0, 0.0, 0.025]";
                    return result;
                }

                const int site_id = site_ref->second.site_id;
                if (site_ref->second.euler_deg)
                {
                    mjtNum euler_rad[3] = {
                        static_cast<mjtNum>(value[0] * M_PI / 180.0),
                        static_cast<mjtNum>(value[1] * M_PI / 180.0),
                        static_cast<mjtNum>(value[2] * M_PI / 180.0),
                    };
                    mju_euler2Quat(model_->site_quat + 4 * site_id, euler_rad, "xyz");
                }
                else
                {
                    model_->site_pos[3 * site_id + 0] = value[0];
                    model_->site_pos[3 * site_id + 1] = value[1];
                    model_->site_pos[3 * site_id + 2] = value[2];
                }
                changed = true;
                continue;
            }

            const auto joint_ref = joint_axis_param_refs_.find(param.get_name());
            if (joint_ref != joint_axis_param_refs_.end())
            {
                std::vector<double> value;
                if (!parameter_vector3(param, value))
                {
                    result.successful = false;
                    result.reason = param.get_name() + " 需要 3 个数字，例如 [1.0, 0.0, 0.0]";
                    return result;
                }

                const double norm = std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
                if (norm <= 1e-12)
                {
                    result.successful = false;
                    result.reason = param.get_name() + " 不能是零向量";
                    return result;
                }

                const int joint_id = joint_ref->second.joint_id;
                model_->jnt_axis[3 * joint_id + 0] = value[0] / norm;
                model_->jnt_axis[3 * joint_id + 1] = value[1] / norm;
                model_->jnt_axis[3 * joint_id + 2] = value[2] / norm;
                changed = true;
            }
        }

        if (changed)
        {
            mj_forward(model_, data_);
        }
        return result;
    }

    /**
     * @brief 按名字找一个传感器，返回它在 sensordata 里的起点
     *
     * @param name 模型里 <sensor> 的 name
     * @param expect_dim 期望的分量个数，对不上说明找错了
     * @return int 起点下标；模型里没有这个名字时返回 -1
     */
    int find_sensor(const char* name, int expect_dim) const
    {
        for (int i = 0; i < model_->nsensor; ++i)
        {
            const char* sensor_name = mj_id2name(model_, mjOBJ_SENSOR, i);
            if (sensor_name == nullptr || std::strcmp(sensor_name, name) != 0)
            {
                continue;
            }
            if (model_->sensor_dim[i] != expect_dim)
            {
                throw std::runtime_error(std::string("sim: 传感器 ") + name + " 的分量个数不是期望的那几个");
            }
            return model_->sensor_adr[i];
        }
        return -1;
    }

    /**
     * @brief 力矩超时：某一路超过 cmd_timeout_s_ 没收到新指令就把它清零
     *
     * @note 没收到过指令的路不碰（启动阶段 ctrl 本来就是 0）
     */
    void enforce_command_timeout()
    {
        if (cmd_timeout_s_ <= 0.0)
        {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < cmd_seen_.size(); ++i)
        {
            if (cmd_seen_[i] && std::chrono::duration<double>(now - cmd_time_[i]).count() > cmd_timeout_s_)
            {
                data_->ctrl[i] = 0.0;
            }
        }
    }

    /**
     * @brief 按名字前缀接一路 IMU：找 <prefix>_quat/_gyro/_accel 三个传感器，齐了才发
     *
     * @param prefix 模型里传感器名前缀，例如 "imu" / "gimbal_imu"
     * @param topic 发布话题
     * @param frame_id 消息 header 里的 frame_id
     */
    void add_imu_channel(const std::string& prefix, const std::string& topic, const std::string& frame_id)
    {
        ImuChannel channel;
        channel.quat_adr  = find_sensor((prefix + "_quat").c_str(), 4);
        channel.gyro_adr  = find_sensor((prefix + "_gyro").c_str(), 3);
        channel.accel_adr = find_sensor((prefix + "_accel").c_str(), 3);
        channel.frame_id  = frame_id;

        if (channel.quat_adr >= 0 && channel.gyro_adr >= 0 && channel.accel_adr >= 0)
        {
            channel.pub = create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS());
            imu_channels_.push_back(std::move(channel));
            RCLCPP_INFO(get_logger(), "IMU 就绪：%s_* 发到 %s", prefix.c_str(), topic.c_str());
        }
        else
        {
            RCLCPP_WARN(get_logger(), "模型里的 IMU 传感器不全（需要 %s_quat / %s_gyro / %s_accel），不发 %s",
                        prefix.c_str(), prefix.c_str(), prefix.c_str(), topic.c_str());
        }
    }

    void setup_camera_renderer(CameraChannel& channel)
    {
        channel.camera_id = mj_name2id(model_, mjOBJ_CAMERA, channel.name.c_str());
        if (channel.camera_id < 0)
        {
            RCLCPP_WARN(get_logger(), "模型里找不到摄像头 %s，不发布 %s", channel.name.c_str(), channel.topic.c_str());
            return;
        }
        if (channel.width <= 0 || channel.height <= 0)
        {
            RCLCPP_WARN(get_logger(), "摄像头图像尺寸非法 %dx%d，不发布 %s", channel.width, channel.height, channel.topic.c_str());
            return;
        }

        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        channel.window = glfwCreateWindow(channel.width, channel.height, channel.name.c_str(), nullptr, nullptr);
        if (channel.window == nullptr)
        {
            RCLCPP_WARN(get_logger(), "创建隐藏 OpenGL 相机窗口失败，不发布 %s", channel.topic.c_str());
            return;
        }
        glfwMakeContextCurrent(channel.window);

        mjv_defaultScene(&channel.scene);
        mjr_defaultContext(&channel.context);
        mjv_makeScene(model_, &channel.scene, 20000);
        mjr_makeContext(model_, &channel.context, mjFONTSCALE_100);
        mjr_resizeOffscreen(channel.width, channel.height, &channel.context);

        const std::size_t bytes = static_cast<std::size_t>(channel.width) * static_cast<std::size_t>(channel.height) * 3U;
        channel.rgb.resize(bytes);
        channel.rgb_flipped.resize(bytes);
        channel.ready = true;
        RCLCPP_INFO(get_logger(), "摄像头就绪：%s -> %s (%dx%d)",
                    channel.name.c_str(), channel.topic.c_str(), channel.width, channel.height);
    }

    void shutdown_camera_renderer(CameraChannel& channel)
    {
        if (channel.ready)
        {
            glfwMakeContextCurrent(channel.window);
            mjr_freeContext(&channel.context);
            mjv_freeScene(&channel.scene);
            channel.ready = false;
        }
        if (channel.window != nullptr)
        {
            glfwDestroyWindow(channel.window);
            channel.window = nullptr;
        }
    }

    void publish_camera_image(CameraChannel& channel)
    {
        if (!channel.ready || !channel.pub)
        {
            return;
        }

        glfwMakeContextCurrent(channel.window);

        mjvCamera cam;
        mjv_defaultCamera(&cam);
        cam.type = mjCAMERA_FIXED;
        cam.fixedcamid = channel.camera_id;

        mjrRect viewport {0, 0, channel.width, channel.height};
        mjv_updateScene(model_, data_, &option_, nullptr, &cam, mjCAT_ALL, &channel.scene);
        mjr_setBuffer(mjFB_OFFSCREEN, &channel.context);
        mjr_render(viewport, &channel.scene, &channel.context);
        mjr_readPixels(channel.rgb.data(), nullptr, viewport, &channel.context);

        const std::size_t row_bytes = static_cast<std::size_t>(channel.width) * 3U;
        for (int y = 0; y < channel.height; ++y)
        {
            const std::size_t src = static_cast<std::size_t>(channel.height - 1 - y) * row_bytes;
            const std::size_t dst = static_cast<std::size_t>(y) * row_bytes;
            std::copy_n(channel.rgb.data() + src, row_bytes, channel.rgb_flipped.data() + dst);
        }

        sensor_msgs::msg::Image msg;
        msg.header.stamp = rclcpp::Time(static_cast<std::int64_t>(data_->time * 1.0e9));
        msg.header.frame_id = channel.name;
        msg.height = static_cast<std::uint32_t>(channel.height);
        msg.width = static_cast<std::uint32_t>(channel.width);
        msg.encoding = "rgb8";
        msg.is_bigendian = 0;
        msg.step = static_cast<std::uint32_t>(row_bytes);
        msg.data = channel.rgb_flipped;
        channel.pub->publish(msg);
    }

    /**
     * @brief 把这一拍的各路 IMU 发出去：姿态四元数（w,x,y,z）、机体系角速度、机体系线加速度
     *
     * @note 时间戳跟关节状态一样用 data->time
     */
    void publish_imu()
    {
        for (const ImuChannel& channel : imu_channels_)
        {
            sensor_msgs::msg::Imu msg;
            msg.header.stamp    = rclcpp::Time(static_cast<std::int64_t>(data_->time * 1.0e9));
            msg.header.frame_id = channel.frame_id;

            const mjtNum* quat = data_->sensordata + channel.quat_adr;
            msg.orientation.w  = quat[0];
            msg.orientation.x  = quat[1];
            msg.orientation.y  = quat[2];
            msg.orientation.z  = quat[3];

            const mjtNum* gyro = data_->sensordata + channel.gyro_adr;
            msg.angular_velocity.x = gyro[0];
            msg.angular_velocity.y = gyro[1];
            msg.angular_velocity.z = gyro[2];

            const mjtNum* accel = data_->sensordata + channel.accel_adr;
            msg.linear_acceleration.x = accel[0];
            msg.linear_acceleration.y = accel[1];
            msg.linear_acceleration.z = accel[2];

            channel.pub->publish(msg);
        }
    }

    /**
     * @brief 把自由根节点的世界系线速度发给策略观测。
     */
    void publish_base_velocity()
    {
        if (root_dof_adr_ < 0)
        {
            return;
        }

        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = rclcpp::Time(static_cast<std::int64_t>(data_->time * 1.0e9));
        msg.header.frame_id = "world";
        msg.vector.x = data_->qvel[root_dof_adr_ + 0];
        msg.vector.y = data_->qvel[root_dof_adr_ + 1];
        msg.vector.z = data_->qvel[root_dof_adr_ + 2];
        base_velocity_pub_->publish(msg);
    }

    /**
     * @brief 把这一拍的关节状态发出去
     *
     * @note 时间戳用 data->time 而不是 now()：仿真时间跟墙上时钟会被拖慢时对不上，
     *       下游要的是"物理走了多少"
     */
    void publish_joint_state()
    {
        // 只诊断实际执行链，不添加底盘 IMU，也不改变 ctrl 的写入行为。
        const int big = mj_name2id(model_, mjOBJ_ACTUATOR, "big_yaw");
        const int small = mj_name2id(model_, mjOBJ_ACTUATOR, "small_yaw");
        if (big >= 0 && small >= 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - yaw_diagnostic_time_).count() >= 0.5)
        {
            const auto now = std::chrono::steady_clock::now();
            yaw_diagnostic_time_ = now;
            const double big_age = cmd_seen_[big] ? std::chrono::duration<double>(now - cmd_time_[big]).count() : -1.0;
            const double small_age = cmd_seen_[small] ? std::chrono::duration<double>(now - cmd_time_[small]).count() : -1.0;
            // 独立于命令数值检查 ROS 图，定位残留控制进程争抢同一执行器。
            const std::size_t big_publishers = static_cast<std::size_t>(big) < cmd_subs_.size() ? count_publishers(cmd_subs_[big]->get_topic_name()) : 0;
            const std::size_t small_publishers = static_cast<std::size_t>(small) < cmd_subs_.size() ? count_publishers(cmd_subs_[small]->get_topic_name()) : 0;
            RCLCPP_DEBUG(get_logger(),
                "yaw执行链 t=%.6f run=%d | 收到=(%.5f, %.5f) ctrl=(%.5f, %.5f) 实际执行器力=(%.5f, %.5f) Nm | 指令年龄秒=(%.4f, %.4f) 发布者 big/small=%zu/%zu",
                data_->time, sim_->run ? 1 : 0, cmd_received_[big], cmd_received_[small],
                data_->ctrl[big], data_->ctrl[small],
                data_->actuator_force[big], data_->actuator_force[small], big_age, small_age,
                big_publishers, small_publishers);
        }
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = rclcpp::Time(static_cast<std::int64_t>(data_->time * 1.0e9));

        const std::size_t count = joints_.size();
        msg.name.resize(count);
        msg.position.resize(count);
        msg.velocity.resize(count);
        msg.effort.resize(count);

        for (std::size_t i = 0; i < count; ++i)
        {
            const JointRef& joint = joints_[i];
            msg.name[i]     = joint.name;
            msg.position[i] = data_->qpos[joint.qpos_adr];
            msg.velocity[i] = data_->qvel[joint.dof_adr];
            msg.effort[i]   = data_->qfrc_actuator[joint.dof_adr];
        }

        joint_state_pub_->publish(msg);
    }
};
