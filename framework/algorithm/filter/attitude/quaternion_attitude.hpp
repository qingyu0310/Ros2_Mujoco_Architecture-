/**
 * @file quaternion_attitude.hpp
 * @author qingyu
 * @brief 四元数姿态解算：陀螺积分加加速度计倾角校正
 * @version 0.1
 * @date 2026-09-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace algorithm::filter {

/**
 * @brief 圆周率，按标量类型取。
 *
 * @tparam Scalar 标量类型
 * @return Scalar 圆周率
 */
template <typename Scalar>
constexpr Scalar pi()
{
    return Scalar(3.141592653589793238462643383279502884L);
}

/**
 * @brief 四元数转 ZYX（yaw-pitch-roll）顺序的欧拉角，只做换算不做滤波。
 *
 * @tparam Scalar 标量类型
 * @param q 姿态四元数，内部先归一化
 * @return Eigen::Matrix<Scalar, 3, 1> (roll, pitch, yaw)，单位 rad
 */
template <typename Scalar>
Eigen::Matrix<Scalar, 3, 1> quaternion_to_rpy_rad(const Eigen::Quaternion<Scalar>& q)
{
    const Eigen::Quaternion<Scalar> normalized = q.normalized();

    const Scalar sinr_cosp = Scalar(2) * (normalized.w() * normalized.x() + normalized.y() * normalized.z());
    const Scalar cosr_cosp = Scalar(1) - Scalar(2) * (normalized.x() * normalized.x() + normalized.y() * normalized.y());
    const Scalar roll      = std::atan2(sinr_cosp, cosr_cosp);

    const Scalar sinp  = Scalar(2) * (normalized.w() * normalized.y() - normalized.z() * normalized.x());
    const Scalar pitch = std::abs(sinp) >= Scalar(1) ? std::copysign(pi<Scalar>() / Scalar(2), sinp) : std::asin(sinp);

    const Scalar siny_cosp = Scalar(2) * (normalized.w() * normalized.z() + normalized.x() * normalized.y());
    const Scalar cosy_cosp = Scalar(1) - Scalar(2) * (normalized.y() * normalized.y() + normalized.z() * normalized.z());
    const Scalar yaw       = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Matrix<Scalar, 3, 1>(roll, pitch, yaw);
}

/**
 * @brief 陀螺积分 + 加速度计倾角校正的四元数姿态解算。
 *
 * predict() 用陀螺角速度右乘积分，correct_accel() 用加速度计测到的天向与估计天向的
 * 叉乘做小角度旋转校正；两处都保持单位四元数。
 *
 * @tparam Scalar 标量类型，默认 double
 */
template <typename Scalar = double>
class QuaternionAttitude {
public:
    using Vector3    = Eigen::Matrix<Scalar, 3, 1>;    // 三维向量
    using Quaternion = Eigen::Quaternion<Scalar>;      // 姿态四元数，机体系到世界系

    /**
     * @brief 默认构造，姿态为单位四元数。
     */
    QuaternionAttitude() = default;

    /**
     * @brief 用给定姿态构造，构造时就归一化。
     *
     * @param initial 初始姿态
     */
    explicit QuaternionAttitude(const Quaternion& initial) : q_(initial.normalized()) {}

    /**
     * @brief 重置姿态，默认归到单位四元数。
     *
     * @param q 目标姿态
     */
    void reset(const Quaternion& q = Quaternion::Identity()) { q_ = q.normalized(); }

    /**
     * @brief 设置加速度计校正增益，负值按 0 处理。
     *
     * @param gain 每拍收掉的倾角误差比例
     */
    void set_accel_correction_gain(Scalar gain) {
        accel_correction_gain_ = std::max(Scalar(0), gain);
    }

    /**
     * @brief 用陀螺角速度右乘积分一拍姿态，零输入直接返回。
     *
     * @param gyro_radps 机体系角速度，rad/s
     * @param dt_s 采样周期，s
     */
    void predict(const Vector3& gyro_radps, Scalar dt_s) {
        if (dt_s <= Scalar(0)) {
            return;
        }

        const Scalar angle = gyro_radps.norm() * dt_s;
        if (angle <= Scalar(0)) {
            return;
        }

        const Vector3 axis = gyro_radps.normalized();
        q_ = (q_ * Quaternion(Eigen::AngleAxis<Scalar>(angle, axis))).normalized();
    }

    /**
     * @brief 用加速度计测到的天向做一次小角度旋转校正。
     *
     * @param accel_mps2 机体系加速度计读数，m/s^2
     */
    void correct_accel(const Vector3& accel_mps2) 
    {
        const Scalar norm = accel_mps2.norm();
        if (norm <= Scalar(0)) {
            return;
        }

        // 加速度计静止时读的是比力 +g（沿世界天向），所以这两个向量的物理含义都是"天向"
        // 在机体系里的分量，姿态对时两者相同。叉乘顺序就是反馈符号，别调过来：
        // estimated × measured 是正反馈，姿态会一路翻到 ±180° 才停在那儿（那里叉乘也是 0）。
        const Vector3 measured_up = accel_mps2 / norm;
        const Vector3 estimated_up = q_.inverse() * world_up();
        const Vector3 error = measured_up.cross(estimated_up);

        if (error.norm() <= Scalar(0)) {
            return;
        }

        const Vector3 correction = accel_correction_gain_ * error;
        const Scalar angle = correction.norm();
        if (angle <= Scalar(0)) {
            return;
        }

        q_ = (q_ * Quaternion(Eigen::AngleAxis<Scalar>(angle, correction / angle))).normalized();
    }

    /**
     * @brief 一拍完整更新：先陀螺积分，再加速度计校正。
     *
     * @param gyro_radps 机体系角速度，rad/s
     * @param accel_mps2 机体系加速度计读数，m/s^2
     * @param dt_s 采样周期，s
     */
    void update(const Vector3& gyro_radps, const Vector3& accel_mps2, Scalar dt_s) {
        predict(gyro_radps, dt_s);
        correct_accel(accel_mps2);
    }

    /**
     * @brief 当前姿态四元数。
     */
    Quaternion quaternion() const { return q_; }

    /**
     * @brief 把姿态转成 ZYX（yaw-pitch-roll）顺序的欧拉角。
     *
     * @return Vector3 (roll, pitch, yaw)，单位 rad
     */
    Vector3 euler_rpy_rad() const
    {
        return quaternion_to_rpy_rad(q_);
    }

    /**
     * @brief 世界天向在机体系里的分量，单位向量（倾角判断/重力补偿用）
     */
    Vector3 up_body() const { return q_.inverse() * world_up(); }

private:
    Quaternion q_{Quaternion::Identity()};       // 当前姿态，机体系到世界系
    Scalar accel_correction_gain_{Scalar(0.1)};  // 每拍收掉的倾角误差比例

    // 世界天向：gz/SDF 的世界是 Z 朝上，天向就是 +Z（重力是 -9.8 沿 Z，别搞反）
    static Vector3 world_up()    { return Vector3(Scalar(0), Scalar(0), Scalar(1)); }
};

} // namespace algorithm::filter
