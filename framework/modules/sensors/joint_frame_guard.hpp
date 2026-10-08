/**
 * @file joint_frame_guard.hpp
 * @brief 关节整帧体检：识别疑似陈旧的"贴限位且全停"帧
 */

#pragma once

#include "framework/algorithm/math/scalar.hpp"

#include <array>
#include <cmath>

namespace modules::sensors {

/**
 * @brief 识别"髋膝四个关节同时贴下限、且六路速度近似为 0"的疑似陈旧整帧。
 *
 * 仿真/桥接启动时偶发把上一轮倒地姿态当新帧吐出来；正常摔倒过程一般带非零速度，
 * 不会命中这个判据，所以它只挡这一种帧，不当作通用的摔倒检测用。
 */
class JointFrameGuard
{
public:
    /**
     * @brief 判据参数。
     */
    struct Config
    {
        double hip_min_rad {0.0};            // 髋关节硬下限，rad
        double knee_min_rad {0.0};           // 膝关节硬下限，rad
        double position_eps_rad {2.0e-4};    // 认为"贴限位"的位置容差，rad
        double velocity_eps_radps {1.0e-5};  // 认为"停住"的速度容差，rad/s
    };

    /**
     * @brief 一帧六路关节反馈，下标固定为 轮L / 轮R / 髋L / 髋R / 膝L / 膝R。
     */
    struct Frame
    {
        std::array<double, 6> position_rad {};   // 关节位置，rad
        std::array<double, 6> velocity_radps {}; // 关节角速度，rad/s
    };

    /**
     * @brief 用判据参数构造。
     *
     * @param config 判据参数
     */
    explicit JointFrameGuard(const Config& config) : config_(config) {}

    /**
     * @brief 这一帧是不是疑似陈旧帧。
     *
     * @param frame 本帧六路反馈
     * @return bool 四个髋膝都贴下限且六路速度都近似 0 时返回 true
     */
    bool looks_stale(const Frame& frame) const
    {
        const bool at_lower_limits =
            algorithm::math::near(frame.position_rad[2], config_.hip_min_rad,  config_.position_eps_rad) &&
            algorithm::math::near(frame.position_rad[3], config_.hip_min_rad,  config_.position_eps_rad) &&
            algorithm::math::near(frame.position_rad[4], config_.knee_min_rad, config_.position_eps_rad) &&
            algorithm::math::near(frame.position_rad[5], config_.knee_min_rad, config_.position_eps_rad);

        const bool stopped =
            std::abs(frame.velocity_radps[0]) < config_.velocity_eps_radps &&
            std::abs(frame.velocity_radps[1]) < config_.velocity_eps_radps &&
            std::abs(frame.velocity_radps[2]) < config_.velocity_eps_radps &&
            std::abs(frame.velocity_radps[3]) < config_.velocity_eps_radps &&
            std::abs(frame.velocity_radps[4]) < config_.velocity_eps_radps &&
            std::abs(frame.velocity_radps[5]) < config_.velocity_eps_radps;

        return at_lower_limits && stopped;
    }

private:
    Config config_ {};   // 判据参数
};

} // namespace modules::sensors
