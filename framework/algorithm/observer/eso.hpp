/**
 * @file eso.hpp
 * @author qingyu
 * @brief 三阶线性扩张状态观测器：估计位置、速度和集总扰动加速度
 * @version 0.1
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <cmath>
#include <stdexcept>

namespace algorithm::observer {

/**
 * @brief 单输入单输出三阶线性 ESO
 *
 * 连续形式：
 *   e  = z1 - y
 *   z1_dot = z2 - beta1 * e
 *   z2_dot = z3 - beta2 * e + b * u
 *   z3_dot =     - beta3 * e
 *
 * 离散实现用前向欧拉，适合当前 1 kHz 控制拍和较低观测器带宽。
 * 若 bandwidth_rad_s 开很高，应改成离散极点配置，而不是继续套连续 beta。
 *
 * @tparam Scalar 标量类型
 */
template <typename Scalar = double>
class Eso3
{
public:
    struct State
    {
        Scalar z1 {0};  // 估计位置
        Scalar z2 {0};  // 估计速度
        Scalar z3 {0};  // 估计集总扰动加速度
    };

    Eso3() = default;

    /**
     * @brief 配置 ESO
     *
     * @param period_s 控制周期，单位 s
     * @param input_gain b，有效输入增益；模型 qdd = b * u + d
     * @param bandwidth_rad_s 观测器连续带宽 omega_o，单位 rad/s
     */
    void configure(Scalar period_s, Scalar input_gain, Scalar bandwidth_rad_s)
    {
        if (!(period_s > Scalar{0}))
        {
            throw std::invalid_argument("ESO period_s must be positive");
        }
        if (!(std::abs(input_gain) > Scalar{0}))
        {
            throw std::invalid_argument("ESO input_gain must be non-zero");
        }
        if (!(bandwidth_rad_s > Scalar{0}))
        {
            throw std::invalid_argument("ESO bandwidth_rad_s must be positive");
        }

        period_s_   = period_s;
        input_gain_ = input_gain;

        const Scalar w = bandwidth_rad_s;
        beta1_ = Scalar{3} * w;
        beta2_ = Scalar{3} * w * w;
        beta3_ = w * w * w;
        configured_ = true;
    }

    /**
     * @brief 重置观测器状态
     */
    void reset(Scalar position, Scalar velocity = Scalar{0}, Scalar disturbance_accel = Scalar{0})
    {
        state_.z1 = position;
        state_.z2 = velocity;
        state_.z3 = disturbance_accel;
    }

    /**
     * @brief 推进一拍
     *
     * @param measured_position 实测位置
     * @param saturated_input 实际送进对象的输入，必须是限幅后的输入
     */
    const State& update(Scalar measured_position, Scalar saturated_input)
    {
        if (!configured_)
        {
            throw std::logic_error("ESO has not been configured");
        }

        const Scalar e  = state_.z1 - measured_position;
        const Scalar dz1 = state_.z2 - beta1_ * e;
        const Scalar dz2 = state_.z3 - beta2_ * e + input_gain_ * saturated_input;
        const Scalar dz3 = -beta3_ * e;

        state_.z1 += period_s_ * dz1;
        state_.z2 += period_s_ * dz2;
        state_.z3 += period_s_ * dz3;

        return state_;
    }

    const State& state() const { return state_; }

private:
    bool configured_ {false};

    Scalar period_s_    {Scalar{0.001}};
    Scalar input_gain_  {Scalar{1}};

    Scalar beta1_       {Scalar{0}};
    Scalar beta2_       {Scalar{0}};
    Scalar beta3_       {Scalar{0}};

    State  state_ {};
};

} // namespace algorithm::observer
