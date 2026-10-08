/**
 * @file pid.hpp
 * @author qingyu
 * @brief 位置式 PID：微分先行、变速积分、积分分离、微分低通及前馈
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @note 移植自 Dust_Zephyr_Tree/framework/algorithm/controller/pid。
 *       保留原算法计算顺序，适配当前纯头文件算法层；不依赖 ROS 或 Zephyr。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace algorithm::controller
{

/**
 * @brief 微分先行：关闭时对误差微分，开启时对测量值微分
 */
enum class DerivativeFirst : std::uint8_t
{
    Disable = 0,
    Enable
};

/**
 * @brief 位置式 PID，误差采用 target - measurement
 * @tparam Scalar 浮点标量，默认 double
 * @note 前馈沿用原算法：kf * 本拍目标增量，不除以控制周期。
 *       死区内目标会对齐测量值；上一拍输出饱和或误差反向时清积分。
 */
template <typename Scalar = double>
class Pid final
{
    static_assert(std::is_floating_point_v<Scalar>, "PID requires a floating-point scalar");

public:
    /**
     * @brief 参数：限幅和分离阈值为零时关闭对应功能
     */
    struct Params
    {
        Scalar kp {0.0};
        Scalar ki {0.0};
        Scalar kd {0.0};
        Scalar kf {0.0};
        Scalar integral_limit {0.0};  // 积分输出限幅，沿用原算法的积分前限幅时序
        Scalar output_limit {0.0};
        Scalar dt {0.001};            // 控制周期，s
        Scalar dead_zone {0.0};
        Scalar integral_speed_low {0.0};
        Scalar integral_speed_high {0.0}; // 达到上阈值时停止本拍积分并清零
        Scalar integral_separation {0.0}; // 达到分离阈值时清零积分
        DerivativeFirst derivative_first {DerivativeFirst::Disable};
        Scalar derivative_lpf_hz {0.0};
    };

    /**
     * @brief 运行状态，供日志读取；角度模式的位置量单位为 rad
     */
    struct State
    {
        Scalar target {0.0};
        Scalar measurement {0.0};
        Scalar output {0.0};
        Scalar previous_measurement {0.0};
        Scalar previous_target {0.0};
        Scalar previous_output {0.0};
        Scalar previous_error {0.0};
        Scalar integral_error {0.0};
        Scalar d_filtered {0.0};
    };

    Pid() = default;

    /**
     * @brief 配置控制器，初始历史状态为零，与原 PID 一致
     */
    explicit Pid(const Params& params)
    {
        configure(params);
    }

    /**
     * @brief 更新参数并重算微分滤波系数，不自动清除历史状态
     */
    void configure(const Params& params)
    {
        validate_parameters(params);
        params_ = params;
        d_lpf_alpha_ = params_.derivative_lpf_hz > 0.0 ? Scalar(1) / (Scalar(1) + Scalar(2) * std::acos(Scalar(-1)) * params_.derivative_lpf_hz * params_.dt) : Scalar(0);
    }

    /**
     * @brief 重置历史，可用当前测量和目标避免启动微分及前馈跳变
     */
    void reset(Scalar measurement = Scalar(0), Scalar target = Scalar(0))
    {
        state_ = State {};
        state_.measurement = measurement;
        state_.target = target;
        state_.previous_measurement = measurement;
        state_.previous_target = target;
        state_.previous_error = target - measurement;
    }

    /**
     * @brief 普通位置式计算，参数顺序与当前 LQR 的测量/目标顺序一致
     */
    Scalar update(Scalar measurement, Scalar target)
    {
        state_.measurement = measurement;
        state_.target = target;
        return compute(target - measurement);
    }

    /**
     * @brief 角度位置式计算，输入为 rad，按最短角度差处理任意多圈输入
     * @note 微分及目标增量前馈沿用原算法；编码器跨圈时建议调用方先展开测量值。
     */
    Scalar update_angle(Scalar measurement, Scalar target)
    {
        state_.measurement = measurement;
        state_.target = target;
        const Scalar pi = std::acos(Scalar(-1));
        Scalar error = std::fmod(target - measurement, Scalar(2) * pi);
        if (error > pi)
        {
            error -= Scalar(2) * pi;
        }
        else if (error < -pi)
        {
            error += Scalar(2) * pi;
        }
        return compute(error);
    }

    /**
     * @brief 设置积分误差，单位为误差单位乘秒
     */
    void set_integral_error(Scalar value)
    {
        state_.integral_error = value;
    }

    const Params& parameters() const
    {
        return params_;
    }

    const State& state() const
    {
        return state_;
    }

private:
    Params params_ {};
    State state_ {};
    Scalar d_lpf_alpha_ {0.0};

    /**
     * @brief 拒绝非有限参数、非法周期、负限幅以及颠倒的变速积分阈值
     */
    static void validate_parameters(const Params& params)
    {
        const Scalar values[] {
            params.kp, params.ki, params.kd, params.kf, params.dt,
            params.integral_limit, params.output_limit, params.dead_zone,
            params.integral_speed_low, params.integral_speed_high,
            params.integral_separation, params.derivative_lpf_hz
        };
        for (const Scalar value : values)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument("PID parameters must be finite");
            }
        }
        if (!(params.dt > 0.0) || params.integral_limit < 0.0 || params.output_limit < 0.0 ||
            params.dead_zone < 0.0 || params.integral_speed_low < 0.0 || params.integral_speed_high < 0.0 ||
            params.integral_separation < 0.0 || params.derivative_lpf_hz < 0.0 ||
            ((params.integral_speed_low != 0.0 || params.integral_speed_high != 0.0) &&
             !(params.integral_speed_high > params.integral_speed_low)))
        {
            throw std::invalid_argument("PID period, limits or integral thresholds are invalid");
        }
    }

    /**
     * @brief 公共计算：死区、积分策略、PID、目标增量前馈、输出限幅及历史更新
     */
    Scalar compute(Scalar error)
    {
        // 误差
        Scalar abs_error = std::abs(error);

        // 死区
        if (abs_error < params_.dead_zone)
        {
            state_.target   = state_.measurement;
            error     = 0.0;
            abs_error  = 0.0;
        }
        else if (error > 0.0)
        {
            error    -= params_.dead_zone;
            abs_error  = std::abs(error);
        }
        else if (error < 0.0)
        {
            error    += params_.dead_zone;
            abs_error  = std::abs(error);
        }

        // 变速积分（|error| ≤ Lo → 全速；|error| ≥ Hi → 停积分 + 清零；[Lo, Hi] → 线性降速）
        Scalar speed_ratio = 1.0;
        if (params_.integral_speed_low != 0.0 || params_.integral_speed_high != 0.0)
        {
            if (abs_error <= params_.integral_speed_low)
            {
                speed_ratio = 1.0;
            }
            else if (abs_error >= params_.integral_speed_high)
            {
                speed_ratio = 0.0;
                state_.integral_error = 0.0;          // 超上限 → 清零防饱和
            }
            else
            {
                Scalar denom = params_.integral_speed_high - params_.integral_speed_low;
                if (denom > 0.0)
                {
                    speed_ratio = (params_.integral_speed_high - abs_error) / denom;
                }
            }
        }

        // P项
        const Scalar p_out = params_.kp * error;

        // I项
        Scalar i_out = 0.0;

        // 积分限幅 (防除零)
        if (params_.integral_limit != 0.0 && params_.ki != 0.0)
        {
            Scalar i_clamp = params_.integral_limit / std::abs(params_.ki);
            state_.integral_error = std::clamp(state_.integral_error, -i_clamp, i_clamp);
        }

        // 防卡角: 输出饱和或误差反向时清零
        Scalar abs_out = std::abs(state_.output);
        if ((params_.output_limit != 0.0 && abs_out >= params_.output_limit) || (state_.previous_error > 0.0 && error < 0.0) || (state_.previous_error < 0.0 && error > 0.0))
        {
            state_.integral_error = 0.0;
        }

        // 积分分离（与 integral_speed_high 功能重叠，关闭一个即可）
        if (params_.integral_separation == 0.0 || abs_error < params_.integral_separation)
        {
            state_.integral_error += speed_ratio * params_.dt * error;
            i_out = params_.ki * state_.integral_error;
        }
        else
        {
            state_.integral_error = 0.0;
        }

        // D项
        Scalar d_raw;
        if (params_.derivative_first == DerivativeFirst::Enable)
        {
            d_raw = -params_.kd * (state_.measurement - state_.previous_measurement) / params_.dt;
        }
        else
        {
            d_raw = params_.kd * (error - state_.previous_error) / params_.dt;
        }

        Scalar d_out = d_raw;
        if (d_lpf_alpha_ > 0.0)
        {
            // 一阶低通：y(n) = α · y(n-1) + (1-α) · x(n)
            // 化简为一次乘法：y(n) = x(n) + α · (y(n-1) - x(n))
            d_out = d_raw + d_lpf_alpha_ * (state_.d_filtered - d_raw);
            state_.d_filtered = d_out;
        }

        // 前馈
        const Scalar f_out = params_.kf * (state_.target - state_.previous_target);

        // 输出
        state_.output = p_out + i_out + d_out + f_out;
        if (params_.output_limit != 0.0)
        {
            state_.output = std::clamp(state_.output, -params_.output_limit, params_.output_limit);
        }

        // 状态保持
        state_.previous_measurement    = state_.measurement;
        state_.previous_target = state_.target;
        state_.previous_output    = state_.output;
        state_.previous_error  = error;

        return state_.output;
    }
};

} // namespace algorithm::controller
