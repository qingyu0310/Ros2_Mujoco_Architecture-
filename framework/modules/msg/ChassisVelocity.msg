# 底盘实测轮速经运动学反解得到的平面速度估计，不是控制目标。
# header.stamp 使用轮速反馈采样时间；header.frame_id 为 base_link。
# 坐标约定：+X 向前，+Y 向左，绕 +Z 逆时针为正。
# 打滑或离地时，轮速估计可能偏离底盘真实运动。
std_msgs/Header header
float64 vx_m_s       # 底盘前向线速度，m/s
float64 vy_m_s       # 底盘左向线速度，m/s
float64 wz_rad_s     # 底盘绕自身 Z 轴的角速度，rad/s
