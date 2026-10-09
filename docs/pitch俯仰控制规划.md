# Pitch 俯仰控制规划

> 基于 2026-10-08 当前源码重写。本文是后续设计与验证计划，不代表控制效果已经验收。
> 保持 `YawNode`、`PitchNode` 分节点，由 `project/node/gimbal/gimbal.cpp` 统一启动。
> 不新增、不订阅底盘 IMU；底盘运动由云台 IMU 与大小 yaw/pitch 编码器按刚体运动学解析，具体见 yaw 规划第 4.3 节。
> 先闭合坐标、模型、输入和数据链路；大小 yaw 惯性耦合补偿与 pitch 改进并行规划，再接入残差 ESO，最后验收组合工况。

## 1. 当前实现与边界

| 项目 | 当前源码事实 | 尚需完成 |
|---|---|---|
| pitch 节点 | `ros2_layer/node/gimbal/pitch.hpp`，独立 `Lqr<2,1>` | 模型参数核对、世界俯仰模式、ESO 动态验收 |
| pitch 反馈 | `/joint_states` 中的 `pitch_pitch_joint` 位置和速度 | 消息长度、有限值、时间戳及超时检查 |
| pitch 输出 | `/motor/pitch/cmd_force`，最终力矩限幅 | 原始力矩、限幅占比和数据有效性记录 |
| 输入 | `/keyboard` 的归一化 `mouse_dy`，映射为目标角速度再积分 | 输入超时归零、参考速度与反馈一致 |
| 重力前馈 | `sign * mass * g * com_x * cos(q)` | 从实际模型核对质量一阶矩，支持倾斜底盘 |
| 云台 IMU | pitch 轴处已有 site，整车已有 quat/gyro/accel sensor | 坐标核对及 pitch 订阅 |
| IMU 解算 | `/gimbal/imu/euler_rad`、`/gimbal/imu/angular_velocity` 等已发布 | 时间同步、安装外参、欧拉角速度转换 |
| yaw | 大轴关节角 LQR、小轴 IMU 世界角 LQR，均为 `Lqr<2,1>` | 本规划不重构 yaw 分配律 |
| ESO | `framework/algorithm/observer/eso.hpp` 已有 `Eso3` | 已接入残差 ESO，动态效果待验证 |

当前 pitch 已实现“关节角 LQR + 简化重力前馈”，并非从零开始。编译通过、模型可加载与静态/运动工况验收分别记录。

yaw 的硬边界：小轴所需目标在软限内时保持历史大轴参考；只有分配超限部分才移动大轴参考。pitch 的 IMU 误差、ESO 或补偿不得写入 yaw 的历史参考，不改变 yaw 的 Q/R 接口或反馈结构。

## 2. 两种控制目标

### 2.1 关节角模式：保留当前基线

```text
q = pitch 关节相对父体的角度
q_ref = 用户期望关节角
```

此模式用于确认电机方向、重力补偿、惯量和单轴动态。底盘俯仰时，枪管随底盘倾斜属于该模式的正常结果。

### 2.2 世界俯仰模式：后续用于坡道与越野

取枪管局部前向单位向量 `ex=[1,0,0]`，安装外参校正后的姿态为 `R_world_barrel`：

```text
f_world = R_world_barrel * ex
alpha = atan2(f_world.z, sqrt(f_world.x² + f_world.y²))
```

`alpha` 是枪管相对世界水平面的仰角，向上为正。用户目标记为 `alpha_ref`，与关节角 `q_ref` 分开命名。

局部 +Y 关节按右手定则旋转，水平父体下 `q>0` 会让局部 +X 朝 -Z 转动，因此 `alpha≈-q`。不能把 IMU 欧拉 pitch、关节角和仰角直接同号使用。

实现路线：世界仰角外环生成限速的关节参考，原关节 LQR 执行。先在水平底盘验证，再扩展倾斜底盘。机械行程限制世界指向的可达范围，目标不可达时记录 `unreachable`，停止向限位外积分。

不在第一阶段直接以世界角替换关节状态、沿用相同 `B`；其输入增益的符号和姿态相关性需要重新推导。

## 3. 坐标、IMU 与限位

模型事实以 XML 为准：

- `mujoco/models/gimbal/pitch.xml`：关节轴 `0 1 0`，硬限 `[-35°,35°]`，阻尼 `0.002`，armature `0.0018`。
- 枪管沿 pitch 局部 +X；整机通过 attach 形成 `pitch_pitch_joint` 与 `pitch_pitch_link`。
- 云台 IMU 在 pitch 局部原点，但该点并不位于整车所有 yaw 轴上；小陀螺仍会产生平移加速度。
- 当前软限为对称 `±25°`。保留机械几何，不因控制实现改变安装位置或布局。

`ImuNode::publish_raw_solution()` 原样转发原始角速度；`/gimbal/imu/angular_velocity` 的名字不表示世界系，也不表示欧拉角导数。仿真 gyro 是传感器局部坐标，需要核对 site 朝向并应用外参。

```text
R_world_barrel = R_world_imu * R_imu_barrel
omega_barrel = transpose(R_imu_barrel) * omega_imu
```

标准 ZYX 欧拉角中，若机体系角速度为 `[p,r_y,r_z]`、欧拉 roll 为 `phi`：

```text
theta_dot = cos(phi)*r_y - sin(phi)*r_z
```

因此 gyro.y 只有在相应姿态条件下才近似等于欧拉 pitch 导数。世界仰角速度优先由 `f_dot = omega_world × f_world` 推导，或使用带时间戳和滤波的仰角差分交叉检查。接近竖直指向时需处理水平投影退化。

仿真可用 orientation 真值验证；实车姿态融合受加速度和安装误差影响，不能将仿真验收直接当成实车验收。

### 3.1 pitch 运动对 yaw IMU 反馈的影响

IMU 安装在 pitch 上，俯仰会使 gyro 的测量轴旋转。当前 yaw 将局部 gyro.z 直接作为世界 yaw 速率近似，pitch 非零时会产生投影误差，影响小 yaw 的速度反馈以及底盘加速度/惯性前馈估计。

标准 ZYX 约定下 `psi_dot=(sin(roll)*gyro_y+cos(roll)*gyro_z)/cos(pitch)`，其中 gyro 必须先变换到枪管坐标。底盘水平、纯 yaw 运动时，25° pitch 对直接使用 gyro.z 产生约 9.4% 的低估。不能把此误差归为“大小 yaw 惯量不准”后用惯量参数补救。

底盘倾斜后 `yaw_world≈base_yaw+q_big+q_small` 也不再是精确关系，必须检查完整旋转链。安装外参、航向导数、样本同步、偏移加速度和专项工况见 [大小 yaw 联动规划](大小yaw联动规划.md) 第 2.1 节。

此修正同时涉及 pitch 状态和 yaw 测量接口，应单独实施并回归软限分配行为；世界方向稳定与电机相对角跟踪分别验收。


## 4. 单轴模型与 LQR

以关节坐标写动力学，统一区分外力矩与补偿力矩：

```text
J_eff*qdd = tau_motor - D*dq + tau_gravity_external + tau_coupling_external + tau_residual
```

`J_eff` 包含随 pitch 运动的刚体折算惯量和 pitch joint armature。当前参数 `j_pitch=0.00035` 与 XML `armature=0.0018` 的关系尚未核对；不能忽略后者后宣称输入增益准确。

核对步骤：从加载模型提取相关质量、质心、惯量与质量矩阵，在固定其他自由度的条件下确认 pitch 等效惯量；其他关节自由运动时，使用完整动力学响应检查耦合影响。单个质量矩阵对角元不自动等于自由耦合系统的有效惯量。

连续近似：

```text
x = [q, dq]^T
Ac = [[0, 1], [0, -D/J_eff]]
Bc = [[0], [1/J_eff]]
```

当前代码使用前向欧拉、无阻尼模型：

```text
A = [[1,T],[0,1]]
B = [[0],[T/j_pitch]]
```

后续在模型核对后选择精确零阶保持离散化或经过误差检查的欧拉离散化。无阻尼时精确 `B=[T²/(2J_eff), T/J_eff]^T`。修改离散化和惯量后重新求解 LQR，记录求解收敛情况与闭环特征值。

反馈定义：

```text
error = [q-q_ref, dq-dq_ref]^T
tau_lqr = -K * error
```

当前速度反馈直接使用 `dq`；移动参考时需显式给出 `dq_ref`，避免位置参考在动、速度反馈却要求停住。机械有限行程内位置误差直接相减；越界或异常多圈读数进入数据故障处理，不以 wrap 掩盖。

## 5. 重力补偿

### 5.1 水平父体的解析基线

设全部随 pitch 转动部件的质量一阶矩为：

```text
Sx = sum(m_i*x_i)
Sz = sum(m_i*z_i)
```

质心坐标均相对 pitch 轴、表达在零位 pitch 坐标中。父体水平时：

```text
tau_gravity_external = g*(Sx*cos(q) + Sz*sin(q))
tau_gravity_ff = -tau_gravity_external
```

对 +Y 关节、前向偏心 `Sx>0`，水平时补偿应为负力矩。仍用静态实验核对模型轴、执行器 gear 与实际响应，不凭画面直觉改符号。

此前 `pitch_gravity_comp_mass=0.09` 时，静态日志显示前馈过大、LQR 反向抵消。随后使用等效质量 `0.05`、`com_x=0.065`，水平与负角度保持日志中 LQR 力矩约为 `0.00002 N·m`，角度误差约为日志显示精度 `0.001°`。这是当前会话的静态实验结果，不代表模型总质量，也不代表正角度、倾斜底盘和运动跟踪全部通过；实际生效参数仍以 YAML 为准。

统计对象包括 pitch 座、枪管、两只摩擦轮及附属几何。当前弹丸预览虽然关闭接触，但仍是带 `mass=0.0032` 的 geom，会参与刚体质量和惯量；非接触不等于无质量。site/camera 不计质量。最终使用加载后的模型数据核对，避免只加注释里的数字。

### 5.2 倾斜底盘下的通用表达

仅依赖 `cos(q)` 的公式默认父体水平，爬坡或侧倾时不再充分。后续计算：

```text
tau_gravity_external = sum(axis_world · (r_i_world × (m_i*g_world)))
tau_gravity_ff = -tau_gravity_external
```

节点中使用确认过的集中质量/质心参数和姿态外参；离线 MuJoCo 工具负责提供对照真值，不在控制循环里解析 XML。若所有相关部件可合并为单刚体，则在 barrel 坐标下使用 `ey · (r_com × m*g_barrel)` 简化。

先在平地做 `q=0,±10°,±20°` 静态保持，再测试底盘前倾、后倾和侧倾。记录补偿与实际保持力矩，而非通过放大质心参数掩盖 ESO、符号或模型误差。

## 6. ESO：只估计前馈后的残余扰动

复用 `framework/algorithm/observer/eso.hpp` 的 `Eso3<double>`。现已接入 pitch 残差 ESO：`pitch_eso_enable` 开关、`pitch_eso_bandwidth` 带宽（默认 30 rad/s）。在新关节采样上使用前一区间的 `tau_sent - tau_gravity_ff` 更新，重复时间戳不推进，时间回退或间隔超过 0.1 s 时重置。补偿为 `-j_pitch*z3`，叠加后统一限幅。运行效果待验证。

定义已知前馈：

```text
tau_ff = tau_gravity_ff + tau_coupling_ff
qdd = b*(tau_applied-tau_ff) + d_res
b = 1/J_eff
```

此定义下 ESO 的输入应为 `tau_applied-tau_ff`，不是未限幅指令，也不是直接把总力矩当作残差模型输入。否则 ESO 会把已补偿的重力重新估入 `z3`，再次抵消时产生重复补偿。

控制律：

```text
tau_raw = tau_lqr + tau_ff - z3/b
tau_applied = clamp(tau_raw, -limit, limit)
u_observer = tau_applied - tau_ff
```

这里 `tau_applied` 表示最终发布的力矩；若仿真执行器还存在 gear、额外限幅或延迟，必须核对实际施加力矩与指令的对应关系。

离散时序必须对齐：观测器收到第 k 拍位置时，使用此前区间实际保持的 `u_observer[k-1]` 更新，再用估计状态计算第 k 拍命令。保存前一拍的已限幅力矩和同区间前馈；不能以尚未施加的当前命令解释过去的运动。

先取关节位置作为观测量，以关节模型闭合 ESO。世界仰角外环仍独立存在。ESO 不在未经推导时混用世界角位置与关节输入增益。

接入前先保存无 ESO 基线。低带宽起步，记录 `z1/z2/z3`、观测误差和限幅；不预先宣称某一带宽稳定。停用、数据超时或重置参考模式后需明确重置/冻结策略，恢复时重新初始化，避免旧扰动估计造成力矩跳变。

## 7. yaw 与底盘运动耦合

### 7.1 大小 yaw 之间的惯性耦合补偿：必须单独实施

大轴承载小轴和全部 pitch/发射机构。小轴加速的反作用影响大轴，大轴加速又会扰动上层世界指向，不能只写“交给 ESO”。

在同轴、底盘水平、pitch 固定的简化条件下：

```text
M_yaw = [[Jb+Js+Ab, Js], [Js, Js+As]]
h_base = [Jb+Js, Js]^T
tau_yaw_ff = M_yaw*q_yaw_dd_ref + h_base*base_yaw_dd + D_yaw*q_yaw_dot_ref
tau_yaw_raw = tau_yaw_lqr + tau_yaw_ff
```

`Js` 必须包含 pitch、枪管、摩擦轮等全部上层负载绕 yaw 轴的惯量；`Ab/As` 是对应关节 armature，不加入交叉项。`Js` 随 pitch 姿态可能变化，先在固定 pitch 工况核对模型，再决定姿态调度范围。

重点记录 `Js*qs_dd_ref` 对大轴的补偿，以及 `Js*qb_dd_ref` 对小轴的补偿。前馈使用连续、限加速度且相互一致的参考，包含小陀螺基座项；不能直接差分 wrap 后的角度。

仍保持两个 `Lqr<2,1>`，补偿在 `YawNode` 内合成力矩后统一限幅。小轴需求未超软限时，大轴参考不变；但大轴可以输出抵消小轴反作用的保持力矩。保持参考不动不等于电机力矩为零。

先完成大/小轴分别起停的前馈开关对照，再测试底盘自转和 pitch 同时运动。具体模型、实现步骤和验收见 [大小 yaw 联动与惯性耦合补偿规划](大小yaw联动规划.md) 第 3～7 节。

### 7.2 yaw 运动对 pitch 的耦合补偿

以下内容处理 yaw→pitch 的力矩影响，不能替代上一节大小 yaw 之间的惯性补偿。


不预先假设 `k_accel*yaw_accel*cos(q)` 一定代表真实 pitch 力矩。理想对称机构的某些 yaw 加速度到 pitch 投影可以为零；实际项取决于惯量积、轴偏置、姿态、摩擦轮转动和底盘运动。

实施顺序：

1. pitch 静态重力与单轴动态通过后，分别记录云台 yaw 起停、底盘自转、底盘爬坡冲击。
2. 从姿态与角速度计算正确坐标下的运动量，再检查扰动力矩或 pitch 误差与候选项的相关性。
3. 用完整 MuJoCo 动力学作离线对照，区分重力、速度相关偏置与接触冲击；`qfrc_bias` 含重力和速度相关项，不能整体称为重力。
4. 只有证据支持时才引入参数化前馈，给出单位、符号、适用姿态和辨识数据。
5. 独立测试集验证收益，再与 ESO 合用，避免只在一个方向或单一速度下有效。

原规划的 yaw 加速度/速度平方/交叉项系数不作为已确认模型，也不要求强行调出非零值。接触冲击、运动不可达和传感器延迟分别诊断。

保留当前 yaw 结构。若 pitch 倾斜后发现 yaw 速率坐标问题，单独记录证据并规划修改，不在 pitch 实现中顺带重构 yaw。

## 8. 输入、参数与代码结构

### 8.1 输入语义

键盘节点已将原始鼠标位移乘灵敏度并限幅到 `[-1,1]`。pitch 当前再次乘 `mouse_pitch_sensitivity`，不能把此字段当像素再乘“度/像素”。保留现有速率语义：

```text
input = clamp(mouse_dy*sensitivity, -1, 1)
dq_ref = input*rate_limit
q_ref_next = clamp(q_ref+dq_ref*T, soft_min, soft_max)
```

到软限后，有效参考速度应由实际参考增量计算，避免仍给限位外速度。鼠标停止时参考保持；输入断流时速率归零。固定参数目标与鼠标积分模式分开定义，当前 `pitch_target_deg` 只用于启动初值，尚不是运行时持续跟踪目标。

WASD、Shift 和 yaw 鼠标映射由各自节点负责，本规划不重写整车输入协议。

### 8.2 参数

`project/params/gimbal.yaml` 保持 `/yaw`、`/pitch` 两段。当前有效参数以 `PitchNode::read_parameters()` 为准：

| 参数组 | 当前参数 |
|---|---|
| 周期和输入 | `control_period_s`、`mouse_pitch_sensitivity` |
| 参考 | `pitch_target_deg`、`pitch_target_rate_limit_deg`、`pitch_soft_limit_deg` |
| LQR | `j_pitch`、`pitch_q_pos`、`pitch_q_vel`、`pitch_r`、`pitch_ctrl_limit` |
| 重力 | `pitch_gravity_comp_enable`、`pitch_gravity_comp_mass`、`pitch_gravity_comp_com_x`、`pitch_gravity_comp_sign` |
| 接口 | `pitch_joint`、`keyboard_topic`、`joint_states_topic`、`motor_topic_prefix` |

后续新增参数候选：模式选择、输入/状态超时、IMU 话题及外参、质心 Z、armature 或明确包含 armature 的等效惯量、ESO 开关及带宽、日志采样配置。实现时逐项声明并验证，未实现参数不提前写入生效 YAML。

角度用 `_deg`，内部转 rad；角速度命名应明确 `deg_s` 或 `rad_s`，质量 kg、质心 m、惯量 kg·m²、力矩 N·m。现有 `pitch_target_rate_limit_deg` 是历史角速度参数名，兼容期间注明单位为 deg/s，迁移不能静默改变语义。

### 8.3 函数划分

继续在 `pitch.hpp` 内按“结构体 + 函数块”组织：

```text
read_parameters / validate_parameters
configure_lqr / configure_eso
on_keyboard / on_joint_states / on_gimbal_imu
check_data_freshness
update_reference
compute_gravity_feedforward
compute_coupling_feedforward
update_observer_from_previous_interval
compute_torque / publish_torque / log_state
```

数据组建议为 `PitchSample`、`ImuSample`、`PitchReference`、`ModelParams`、`GravityCompParams`、`ObserverState`、`TorqueCommand`。只增加实际需要的字段，不恢复合并式 `GimbalNode`。

控制周期使用一致的时间基准。当前 wall timer 与固定积分步长在暂停、仿真变速和消息延迟下可能不一致；验证阶段记录实际采样间隔，确定按仿真时间驱动或固定步长同步策略后再评价动态指标。

## 9. 分阶段交付

| 阶段 | 修改范围 | 通过条件 |
|---|---|---|
| 0 基线 | 不改控制架构，记录现状与参数快照 | 可重复启动；日志能区分目标、反馈、前馈和最终力矩 |
| 1 数据与坐标 | 索引/有限值/新鲜度检查、IMU 外参、pitch 对 yaw 速率投影及倾斜旋转链核对 | 正负小力矩、关节角、枪管仰角方向一致；断流后不继续积分 |
| 2 模型 | 核对质量一阶矩、惯量、armature、阻尼和离散化 | 静态保持力矩及小信号响应与模型对照一致，误差有记录 |
| 3 关节 LQR + 重力 | 修正参考速度、重力前馈 | 多个角度保持与双向跟踪通过，无持续振荡和长期饱和 |
| 4 世界俯仰 | 姿态到仰角、外环到关节参考、可达性约束 | 底盘倾斜时世界仰角稳定；不可达目标不积累参考 |
| 5 ESO | 残差输入、正确时序、初始化与恢复 | 相同扰动下误差降低，重力不被重复补偿，无新增高频振荡 |
| 6a 大小 yaw 惯性补偿 | 在 YawNode 内核对两轴惯量并增加参考逆动力学前馈、基座项 | 大/小轴分别起停、自转正反向对照；大轴参考分配不变 |
| 6b yaw→pitch 耦合辨识 | 分离 yaw/自转/越障数据，必要时增加 pitch 前馈 | 独立测试数据支持补偿收益；正反向均有效 |
| 7 回归 | 平地、坡道、台阶、自转和组合工况 | pitch 指标通过，yaw 分配行为未回退 |

每阶段分别标记“已实现”“已验证”“待验证”。不以一次编译或画面观察代替动态验收。

## 10. 日志与验收

结构化日志至少包含：

```text
t_sim, dt, mode, joint_age, imu_age, input_age
q, dq, q_ref, dq_ref, alpha, alpha_ref, alpha_rate
error_joint, error_world, soft_limited, unreachable
tau_lqr, tau_gravity_ff, tau_coupling_ff, tau_eso, tau_raw, tau_sent
eso_z1, eso_z2, eso_z3, observer_input_prev, saturated
```

打印节流日志用于现场诊断，RMS、峰值和短瞬态从高频记录计算；100 ms 一次的屏幕日志不足以可靠计算起停冲击峰值。记录参数快照、模型版本、工况、采样率与指标统计区间。

建议初始验收线如下，测试前固定，若需调整必须说明理由：

| 工况 | 指标与建议目标 |
|---|---|
| 平地静态，关节目标 0/±10/±20° | 稳态关节角 RMS <0.5°，分别统计每个目标 |
| 双向参考跟踪 | 记录峰值误差、超调、稳定时间及两方向差异，目标速率保持一致 |
| 世界模式坡道保持 | 可达且进入稳态后世界仰角 RMS <1°，同时报告底盘姿态范围 |
| 小陀螺起停 | 世界仰角峰值 <3°；若未达到，补偿相对基线收益单独报告，不混称绝对指标通过 |
| 小陀螺匀速 | 至少 3 s 稳态区间，世界仰角 RMS <1°，同时报告实际转速均值/波动 |
| ESO/耦合前馈对照 | 相同输入、初值和速度下比较开关效果，记录误差及力矩代价 |
| 所有工况 | 无持续振荡；报告力矩峰值、限幅占比、持续时间与余量 |
| yaw 回归 | 小轴需求在软限内时大轴参考保持；超限随动行为与原基线一致 |

7 N·m 是当前执行器限幅，不能当作持续额定输出。当前文档不新增未经模型确认的持续力矩数字；验收同时检查电机模型和连续输出约束。

台阶碰撞段单独统计峰值与恢复时间，不能混入稳态 RMS 后掩盖冲击，也不能要求任何幅度的碰撞都满足静态保持阈值。

## 11. 下一步

保存当前关节 LQR 与已测静态重力补偿基线，核对 pitch 质量一阶矩、armature、鼠标参考和 IMU 坐标/时间链路。与此同时按大小 yaw 专项规划实施两轴惯性前馈，先固定底盘和 pitch，再加入底盘自转。

pitch 后续依次验证世界俯仰外环与残差 ESO。yaw→pitch 的额外耦合前馈根据数据辨识，与大小 yaw 已明确需要建模的交叉惯性项分开推进。没有日志前，不将任何动态补偿标为已验收。
