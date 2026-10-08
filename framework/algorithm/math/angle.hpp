/**
 * @file angle.hpp
 * @author qingyu
 * @brief 角度工具：弧度归一化和最短角差
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <cmath>

namespace algorithm::math {

/**
 * @brief 弧度归一化到 [-pi, pi]
 *
 * @param radian 任意弧度
 * @return double 归一化后的弧度
 *
 * @note 对齐 Dust_SentinelRobot-main::normalize_pi
 */
inline double normalize_radian_pm_pi(double radian)
{
    radian = std::fmod(radian, 2.0 * M_PI);
    if (radian > M_PI)
    {
        radian -= 2.0 * M_PI;
    }
    else if (radian < -M_PI)
    {
        radian += 2.0 * M_PI;
    }
    return radian;
}

/**
 * @brief 返回 current - target 的最短角差
 *
 * @param current_radian 当前角
 * @param target_radian 目标角
 * @return double 落在 [-pi, pi] 的角差
 *
 * @note 对齐 Dust_SentinelRobot-main::CalcYawError
 */
inline double radian_error_pm_pi(double current_radian, double target_radian)
{
    double error = current_radian - target_radian;
    while (error > M_PI)
    {
        error -= 2.0 * M_PI;
    }
    while (error < -M_PI)
    {
        error += 2.0 * M_PI;
    }
    return error;
}

} // namespace algorithm::math
