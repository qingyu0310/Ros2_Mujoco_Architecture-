# 大小 yaw 联动与惯性耦合补偿规划

> 基于 2026-10-08 当前源码。当前实现与后续计划分开记录。
> 大 yaw 关节角 LQR、小 yaw 世界 yaw LQR，保留两个 `Lqr<2,1>` 和现有 Q/R 数组接口。
> 已实现固定基座的大小 yaw 惯性耦合前馈；已接入基座运动解析和航向速度补偿，基座加速度力矩补偿待实现。
> 硬要求：不新增、不订阅底盘 IMU。底盘节点通过四轮实测转速反解底盘系 vx/vy/wz，并通过 `/chassis/velocity` 发布给 yaw。

## 1. 当前实现

`project/node/gimbal/gimbal.cpp` 统一启动 `YawNode` 与 `PitchNode`。yaw 实现在 `ros2_layer/node/gimbal/yaw.hpp`，参数在 `project/params/gimbal.yaml` 的 `/yaw` 段。

| 数据 | 用途 |
|---|---|
| `/keyboard` 鼠标横移、左右键 | 生成世界 yaw 目标角速度并积分成角度 |
| `/joint_states` | 两级相对关节角和速度 |
| `/gimbal/imu/euler_rad` | 枪管世界 yaw |
| `/gimbal/imu/angular_velocity` | 当前取局部 gyro.z 作为 yaw 速率近似，倾斜工况待修正 |
| `/motor/big_yaw/cmd_force` | 大轴力矩指令 |
| `/motor/small_yaw/cmd_force` | 小轴力矩指令 |

当前没有 yaw ESO。已增加固定基座的分配参考惯性前馈：完整两轴惯量矩阵、自惯量/交叉惯量分项及参考阻尼项。开启前馈时，大轴速度误差使用滤波参考速度；小轴仍闭环世界航向。

参数 `yaw_inertia_ff_enable` 控制开关，`yaw_inertia_reference_lpf_hz` 控制参考导数低通。前馈只读原分配参考，不修改软限或历史大轴参考。参考速度由世界目标速度与底盘速度解析分配后低通，加速度由滤波参考速度差分；角度参考本身仍沿用原分配，并非已完成所有参考整形。

已接入 chassis 四轮实测转速反解话题，以及平面基座角速度补偿。Shift / Ctrl 时速度补偿保持有效；固定基座加速度前馈仍关闭，反解航向速度绝对值超过 0.01 rad/s 时也关闭该固定基座前馈，避免把运动基座当作固定基座。起转/刹停的基座加速度力矩项尚未接入。

已在当前 MuJoCo 模型的固定基座测试中取得改善，完整 ROS/GUI 闭环、小陀螺和倾斜姿态仍待验证。

## 2. 世界目标与软限分配

定义 `psi_ref` 为世界指向，`qb/qs` 为大、小轴相对关节角，`qb_ref` 为保存的大轴参考。底盘近似水平时：

```text
psi = beta + qb + qs
beta_est = wrap(psi_imu - qb - qs)
theta_body = wrap(psi_ref - beta_est)
small_required = wrap(theta_body - qb_ref)
small_allocated = clamp(small_required, -L_soft, L_soft)
excess = small_required - small_allocated
qb_ref = wrap(qb_ref + excess)
```

当前小轴软限 ±60°，物理限 ±120°；分配限制的是目标需求，实际瞬态超软限需另外监测。

硬要求：

- `excess=0` 时保持历史 `qb_ref`，不能用实际 `qb` 每拍覆盖参考。
- 反馈纠偏、ESO 扰动或前馈力矩不得写回大轴分配参考。
- 补偿可以让大轴输出保持力矩，但不能凭空生成大轴转动目标。
- 不通过小轴撞物理限位推动大轴，不把机械限位当作正常联动机制。
- 当前分配没有小轴主动回中；后续如需要，应单独设计，不能混入惯性补偿。

当前反馈：

```text
x_big   = [wrap(qb-qb_ref), qb_dot]
x_small = [wrap(psi_imu-psi_ref), yaw_rate_feedback-psi_ref_dot]
tau_fb_big   = -K_big*x_big
tau_fb_small = -K_small*x_small
```

小轴始终闭环最终世界指向，不直接闭环 `small_allocated`。

### 2.1 IMU 对 yaw 闭环、分配和惯性补偿的影响

IMU 是世界指向的反馈来源，测量坐标或时间不一致会同时影响小轴力矩、大轴分配及底盘运动估计。这一部分必须在惯性前馈启用前验证。

当前数据链：`SimNode` 发布 pitch 上的 `/gimbal/imu`，`ImuNode` 输出姿态欧拉角和角速度，`YawNode` 取欧拉角 z 与 gyro.z。`ImuNode::publish_raw_solution()` 原样转发 gyro，没有将其转为世界 yaw 角速度；变量 `yaw_rate_world` 的名字不代表坐标转换已经完成。

#### 安装外参与 pitch 运动

IMU 随 pitch 俯仰，gyro 轴也跟着转。必须先校正 IMU 到枪管的旋转外参：

```text
R_world_barrel = R_world_imu * R_imu_barrel
omega_barrel = transpose(R_imu_barrel) * omega_imu
```

采用标准 ZYX 欧拉角 `roll=phi, pitch=theta, yaw=psi`，枪管机体系角速度为 `[p,q,r]` 时：

```text
psi_dot = (sin(phi)*q + cos(phi)*r) / cos(theta)
```

底盘水平、roll=0、纯世界 yaw 旋转时，局部 `gyro.z≈psi_dot*cos(theta)`。因此 pitch 抬起后直接使用 gyro.z 会低估 yaw 速度；例如 |theta|=25° 时约低估 9.4%。这会改变小轴 LQR 的速度反馈，也会污染差分得到的 yaw 加速度。

不能简单把 `omega_world.z` 当作所有姿态下的欧拉 yaw 导数，两者在有 roll/pitch 运动时也不同。可改用枪管前向向量计算航向及航向速度：

```text
f = R_world_barrel * [1,0,0]^T
omega_world = R_world_barrel * omega_barrel
f_dot = omega_world × f
psi = atan2(f_y, f_x)
psi_dot = (f_x*f_dot_y - f_y*f_dot_x) / (f_x² + f_y²)
```

欧拉角法与前向向量法在相同约定及非退化姿态下应一致。前向水平投影接近零时航向不可定义，需限制适用范围并标记无效，不能靠巨大速率补偿硬撑。

#### 底盘倾斜与两级关节角

`psi=beta+qb+qs` 是竖直同轴、底盘水平工况的关系。底盘倾斜后，两级 yaw 轴随底盘倾斜，不能继续把欧拉 yaw 与关节角简单加减当作精确坐标变换。

倾斜工况规划采用完整旋转链：

```text
R_world_barrel = R_world_base * R_base_barrel(qb,qs,q_pitch,安装外参)
R_world_base = R_world_barrel * transpose(R_base_barrel)
```

从恢复的底盘姿态提取底盘航向，再通过完整方向变换求相对目标与可达性；仅恢复 beta 后继续机械套用水平加法仍不充分。需增加 pitch 编码器输入并在 yaw 数据链中解析底盘姿态；不引入底盘 IMU，当前 yaw 实现尚未完成该三维反解。

保留软限分配原则：正确坐标变换得到需求后才分配。IMU 世界误差反馈不作为额外积分量混入历史大轴参考。

#### 安装位置、加速度与姿态融合

理想刚体上安装位置不改变角速度；安装朝向会改变 gyro 分量。pitch 轴上的 IMU 也不一定处于公共 yaw 轴线上，安装偏移会产生：

```text
a_offset = angular_acceleration × r + omega × (omega × r)
```

它影响 accelerometer，实车融合时可能被误认为重力而污染 roll/pitch，进而影响 yaw 速率坐标变换。纯 gyro+重力观测不能确定绝对 yaw，长期世界 yaw 保持还取决于零偏与外部航向参考。

仿真直接使用 orientation 真值时，偏移加速度不会自动污染该真值；不能将上述实车融合问题误报为当前仿真已发生的问题，也不能以仿真真值稳定证明实车不漂移。

#### 时间、噪声和数据有效性

- 角度、gyro、两轴与 pitch 编码器要按时间戳对齐，记录各自年龄及对齐误差。异步相减会将关节运动误认为底盘运动，误触发大轴分配。
- 当前 yaw 只拒绝旧姿态时间戳，尚无完整超时及角度/速度配对检查；需增加断流、重复帧、仿真时间回退与恢复初始化策略。
- 底盘角速度不能用未经姿态转换的 gyro.z 减去两关节速度。即使在水平近似下，也需统一时间和方向；倾斜时使用完整轴投影关系。
- `beta_dd` 由速度估计差分会放大噪声；先低通、限制异常值并量化相位延迟，参考加速度与实测加速度分开记录。
- 模型惯性前馈会把加速度估计误差直接变成力矩误差，不能靠提高前馈增益抵消 IMU 延迟。

#### 专项验收

| 工况 | 检查内容 |
|---|---|
| 静止、pitch 为 0/±20/±25° | yaw 不应仅因 pitch 改变产生虚假漂移；安装外参和静态偏差有记录 |
| 相同世界 yaw 速度、不同 pitch | 对比 raw gyro.z、转换后 psi_dot 与仿真真值/展开航向差分；验证投影修正 |
| 底盘前倾、侧倾，再转 yaw | 完整旋转链与仿真 body 姿态对照；验证水平加法的适用边界 |
| 大/小轴分别起停 | 同步前后 beta_est 的误差、噪声与分配变化；不应产生额外大轴参考跳变 |
| 小陀螺起转和刹停 | 记录 beta_dot/beta_dd、传感器延迟、世界 yaw 误差与前馈力矩 |
| 暂停、断流、恢复 | 不使用旧样本持续计算惯性前馈；恢复时无异常加速度尖峰 |

专项数据通过后再启用第 4 节基座惯性补偿。记录 raw gyro、校正 gyro、世界角速度、航向速度和各样本时间戳，不能只保留名为 `world_rate` 的单一字段。

## 3. 大小 yaw 惯性耦合模型

### 3.1 惯量边界

- `Jb`：只随大轴转动、位于小轴之前的刚体惯量。
- `Js`：小轴及其全部上层负载绕共同 yaw 轴的刚体惯量，包括 pitch、枪管、摩擦轮和附属几何。
- `Ab/As`：大、小轴关节 armature，按 XML 折算到对应关节。
- `Db/Ds`：关节阻尼。
- `beta`：底盘世界 yaw；`qb/qs` 均为相对关节角。

`Js` 必须使用平行轴定理统计，不能只算小 yaw 圆柱。同轴两轴在 pitch 固定时不会仅因 `qs` 改变就改变绕公共轴的惯量；pitch 运动则可能使 `Js` 改变。底盘倾斜、pitch 自由运动及其他轴耦合属于完整模型核对范围。

### 3.2 同轴、竖直、pitch 固定时的简化模型

刚体动能：

```text
E = 0.5*Jb*(beta_dot+qb_dot)²
  + 0.5*Js*(beta_dot+qb_dot+qs_dot)²
  + 0.5*Ab*qb_dot² + 0.5*As*qs_dot²
```

对应关节力矩：

```text
M = [[Jb+Js+Ab, Js],
     [Js,       Js+As]]
h_base = [Jb+Js, Js]^T
D = diag(Db, Ds)

tau = M*[qb_dd, qs_dd]^T + h_base*beta_dd + D*[qb_dot,qs_dot]^T
```

armature 加在对应关节对角线上，不加入刚体交叉项或 `h_base`。更完整的反射转子动力学需要另建模型，不能混用。

静止基座、暂不计阻尼与 armature 时：

```text
tau_big   = (Jb+Js)*qb_dd + Js*qs_dd
tau_small = Js*qb_dd + Js*qs_dd
```

含义：小轴加速时，大轴需要相应力矩抵消反作用以保持位置；大轴加速时，小轴也需要相应力矩控制上层机构的运动。这就是需要显式处理的惯性耦合。

仅把 `Js` 加进大轴 LQR 的等效惯量，只处理了对角近似，没有消除交叉项。

## 4. 补偿实现路线

### 4.1 第一阶段：参考运动逆动力学前馈

保留现有两路 LQR 反馈。先生成相互一致且经过速度、加速度约束的参考：

```text
qb_ref + qs_ref = psi_ref - beta_est
qb_dd_ref + qs_dd_ref = psi_dd_ref - beta_dd_est
```

等式在连续展开角度域成立。软限交接时参考应平滑，不直接差分 wrap 后的角度，更不能把 ±pi 跳变当作加速度。若参考整形暂时不能同时满足限制，记录参考偏差和可达性，不隐瞒。

前馈按完整简化矩阵计算：

```text
tau_ff = M*q_dd_ref + h_base*beta_dd_est + D*q_dot_ref
tau_raw_big   = tau_fb_big   + tau_ff_big
tau_raw_small = tau_fb_small + tau_ff_small
tau_sent = clamp_each(tau_raw, motor_limits)
```

这是“参考运动所需力矩 + 跟踪误差反馈”。原 LQR 没有参考加速度前馈，因此此路线不重复叠加已有加速度力矩。阻尼前馈只补参考速度所需力矩，反馈中的速度阻尼作用仍保留。

前馈不保证把实际耦合完全消掉：参考与实际加速度不同、惯量不准或碰撞时仍有残差，必须用对照日志评估。

### 4.2 交叉项与对角项分别记录

```text
tau_ff_big_self  = (Jb+Js+Ab)*qb_dd_ref
tau_ff_big_cross = Js*qs_dd_ref
tau_ff_small_cross = Js*qb_dd_ref
tau_ff_small_self  = (Js+As)*qs_dd_ref
```

再加底盘加速度和参考阻尼项。不能只叠交叉项后宣称世界系小轴已经精确解耦；世界角反馈与关节坐标模型之间的关系也要验证。

需要更强的实际耦合抵消时，可进一步研究由实际/观测加速度或电机力矩估计反作用。但必须推导输入映射、验证滤波相位与稳定性，禁止将两路“最终力矩”互相直接依赖造成代数环，也不直接差分噪声编码器再高增益前馈。

### 4.3 小陀螺影响抵消专项

#### 控制目标与当前缺口

按住 Shift 只让底盘自转，不改变 `psi_ref`。无鼠标输入时，枪管保持原世界方向；有鼠标输入时，枪管跟随用户给定的世界方向，底盘自转不能叠加到用户目标上。

当前已有世界角反馈和软限分配，但大轴速度状态仍为 `qb_dot`，目标为零；小陀螺超软限后，大轴实际上需要持续反转，因此现有速度反馈会抑制其必要跟随。仅加一项加速度力矩不能解决这个速度参考不一致问题。

专项分三层：先抵消运动学影响，再补偿已知动力学力矩，最后由世界角反馈纠正残差。保留两个 LQR、Q/R 接口和现有软限分配，不新增角度限位，不改变用户配置的交接角度。

#### 第一层：底盘运动进入参考速度和加速度

在底盘水平、竖直同轴近似下：

```text
psi = beta + qb + qs
psi_ref_dot = 用户世界目标角速度
relative_rate_ref = psi_ref_dot - beta_dot_measured
relative_accel_ref = psi_ref_dd - beta_dd_estimated
```

无输入时 `psi_ref_dot=0`，两级相对转速之和应为 `-beta_dot`。

分配仍先计算 `theta_body=psi_ref-beta_est`，再相对历史大轴参考判断小轴需求是否超限：

| 区域 | 大轴参考速度 | 小轴相对参考速度 |
|---|---|---|
| 小轴所需目标在软限内 | 0，保持历史大轴参考 | relative_rate_ref |
| 继续向软限外运动，需要大轴承担 | relative_rate_ref | 0，保持该侧软限参考 |
| 反向离开软限，需求重新进入限内 | 0，保持当时大轴参考 | relative_rate_ref |

边界参考根据实际分配序列生成连续速度/加速度，不能仅凭 `abs(qs)` 判断模式，也不能在边界产生不一致的参考跳变。沿连续展开的角度差分参考，并确认：

```text
qb_dot_ref + qs_dot_ref = relative_rate_ref
qb_dd_ref + qs_dd_ref = relative_accel_ref
```

大轴 LQR 速度状态规划改为 `qb_dot-qb_dot_ref`；小轴仍使用转换后的世界航向速度误差 `psi_dot-psi_ref_dot`，不重复减底盘速度。任何参考整形带来的滞后需记录，不能把反馈误差积分到大轴历史参考。

例如底盘以 +360°/s 匀速旋转且世界目标固定：软限内由小轴以约 -360°/s 补偿；需求越限后，大轴相对底盘以约 -360°/s 转动，小轴相对角停留在软限附近，二者共同维持世界指向。这里是参考关系，实际速度必须从日志验证。

#### 第二层：起转和刹停的惯性力矩补偿

参考运动与基座运动一起代入第 3 节模型：

```text
tau_spin_ff = M*q_dd_ref + h_base*beta_dd_estimated + D*q_dot_ref
tau_raw = tau_lqr + tau_spin_ff
```

该式就是第 4.1 节完整前馈的小陀螺工况实现，不另叠一份相同的 `M*q_dd_ref` 或基座项，避免重复补偿。日志把自惯量、交叉惯量、基座和阻尼项分别列出。

世界目标不动时，两轴参考加速度之和为 `-beta_dd`。单独添加 `h_base*beta_dd` 而不考虑相对参考加速度会补错；单独添加 `M*q_dd_ref` 而漏掉基座项同样不成立。

理想竖直同轴、世界上层机构不动的条件下，某些刚体惯性项会互相抵消；电枢惯量、相对关节阻尼和实际跟踪偏差仍可能要求电机出力。不能因为底盘转速高就硬加与速度平方成比例的 yaw 补偿；理想模型匀速时没有这项必然需求。

匀速段重点是正确的反向参考速度及已确认的摩擦/阻尼补偿；起停段重点是参考加速度和基座加速度的相位一致性。接触抖动和倾斜姿态相关残差另行建模，不凭经验统一归为离心力矩。

#### 第三层：世界角反馈及数据链

- 云台姿态/gyro 使用第 2.1 节的外参和航向速度转换；不得直接把局部 gyro.z 当作所有姿态下的世界 yaw 导数。
- 不新增、不订阅底盘 IMU，不使用 Shift 指令转速替代实际底盘速度。
- 主链路：chassis 使用全部实测轮速联合反解底盘系速度，经 `/chassis/velocity` 转发，yaw 订阅 wz_rad_s；不使用底盘自转指令冒充反馈。
- 云台 IMU 与三轴关节刚体运动学反解可作后续交叉检查，当前 yaw 已移除该速度反解实现。轮速主链路在打滑、离地时会存在估计偏差。
- `beta_dd` 使用同一时间基准下滤波后的实际速度差分，记录滤波延迟；角度展开、仿真暂停、时间回退和恢复分别处理。
- 数据缺失时明确禁用对应前馈并标记原因，不能默认为零后宣称小陀螺补偿已经有效。

#### 可选交叉检查：不使用底盘 IMU 的云台运动学解析（当前未启用）

输入为云台 IMU 姿态与角速度、`qb/qs/qp` 和三关节速度、固定安装外参。所有量先同步到同一时刻，关节速度取编码器实测值。

水平底盘、pitch 为零时的简化检查式：

```text
beta_dot = psi_dot_gimbal - qb_dot - qs_dot
```

pitch 或底盘倾斜时，先在统一世界坐标下计算完整角速度链：

```text
R_world_base = R_world_barrel * transpose(R_base_barrel(qb,qs,qp))
a_yaw_world = R_world_base * [0,0,1]^T
a_pitch_world = R_world_base * R_base_pitch_parent(qb,qs) * [0,1,0]^T
omega_gimbal_world = R_world_barrel * omega_barrel
omega_base_world = omega_gimbal_world
                 - a_yaw_world*(qb_dot+qs_dot)
                 - a_pitch_world*qp_dot
omega_base_body = transpose(R_world_base) * omega_base_world
```

上式针对当前同轴两级 yaw、局部 Y pitch 的机械链；固定安装旋转由实际 XML 关节坐标变换带入。不能漏掉 pitch 速度项，也不能在不同坐标系中直接相减分量。

若 `R_world_base` 的标准 ZYX roll/pitch 为 `phi_b/theta_b`，底盘世界航向速度为：

```text
beta_dot = (sin(phi_b)*omega_base_body.y + cos(phi_b)*omega_base_body.z) / cos(theta_b)
```

平面模型中底盘竖直轴与世界 Z 对齐，`beta_dot=omega_base_body.z`。倾斜时完整动力学应使用正确的轴向角速度与姿态，不能将世界欧拉航向导数机械代入竖直同轴模型并宣称精确补偿。航向退化姿态记录不可用，不新增角度限位来掩盖。

解析的只是角速度坐标与运动分解，不消除 IMU 零偏、编码器噪声和时间误差。保持世界指向时，云台角速度可能接近零，但反向关节速度仍能正确反推出底盘自转，不会因为云台稳住就认为底盘不转。

轮速交叉检查沿当前底盘几何：

```text
r_i*dq_i = ux_i*vx + uy_i*vy + m_i*wz
m_i = -ux_i*y_i + uy_i*x_i
H_i = [ux_i,uy_i,m_i]
[vx,vy,wz]^T = least_squares(H, [r_i*dq_i])
```

当前主链路使用全部轮速联合最小二乘反解并检查矩阵秩，不在平移时简单取轮速平均；残差诊断尚待加入。比较轮速 `wz` 与云台刚体运动学反解的底盘局部 z 角速度；两者不是倾斜工况下必然等于世界航向速度的量。仿真真值仅用于离线验收，不作为控制输入，不增加底盘传感器。

建议数据结构与函数块：

```text
BaseYawSample：运动学反解的底盘姿态/角速度、时间戳及有效性，不含底盘 IMU 输入
SpinCompState：底盘速度/加速度、分配参考速度/加速度、各前馈项
update_base_yaw_state()
update_allocated_reference_motion()
compute_yaw_inertia_feedforward()
log_spin_compensation_state()
```

小陀螺按键只影响底盘模式，不应作为云台世界保持生效的唯一开关；即使外力带着底盘转动，正确的测量反馈与运动补偿也应响应。

#### 对照实验和验收

保存三组配置，逐步确认收益：A 为当前基线；B 为正确 IMU 速率与参考速度；C 为 B 加完整惯性前馈。A→B 评估坐标/参考修正，B→C 才用于判断惯性前馈收益，不能混算。

固定 pitch 为 0，正反两个方向分别测试：起转、至少 3 s 匀速、刹停，以及需求跨软限交接。随后在 pitch=±20°、鼠标移动世界目标、底盘跟随模式退出进入自转等工况回归。

```text
base_spin_request, beta_dot_measured, beta_dd_estimated
psi_ref, psi_imu, psi_ref_dot, psi_dot_corrected, yaw_world_error
small_required, allocation_excess, qb_ref, qs_ref
qb_dot_ref, qs_dot_ref, qb_dd_ref, qs_dd_ref
big_lqr, small_lqr, big_base_ff, small_base_ff
big_inertia_ff, small_inertia_ff, big_damping_ff, small_damping_ff
big_sent, small_sent, saturation, sample_age, source_valid
```

建议验收目标，作为待验证指标而非额外限位：

| 工况 | 指标 |
|---|---|
| 起转、刹停 | 世界 yaw 峰值误差 <3°；同时报告 B→C 峰值变化 |
| 匀速 | 世界 yaw RMS <1°，记录实际转速及波动 |
| 软限交接 | 世界角无阶跃、大轴参考只接走超限部分；记录小轴实际超调 |
| 补偿代价 | 比较力矩峰值、限幅占比与振荡，不能靠长期饱和换取低误差 |
| 正反向与 pitch 倾斜 | 两方向均满足，IMU 投影误差不被惯量参数掩盖 |

未达到绝对目标时，如实记录相对改善及剩余原因，不把“画面看着稳”作为通过依据。

## 5. 模型参数和接口

当前 `/yaw` 的 `j_big`、`j_small`、`armature_big`、`armature_small` 是现有控制参数，尚未完成完整模型辨识。核对加载模型的质量、惯量和质量矩阵，再确认是否作为前馈模型共用参数。

LQR 的惯量、Q/R 与前馈模型若需调整，保存原配置并逐项说明，不一次性调参掩盖模型错误。

新增参数候选：惯性前馈开关、基座补偿开关、参考加速度上限、加速度估计滤波配置及诊断采样率。尚未实现的参数不提前写入生效 YAML。

角度 deg/rad、角速度 deg/s 或 rad/s、加速度 deg/s² 或 rad/s²、惯量 kg·m²、力矩 N·m 分别注明；“所有参数都是度数”的旧约定删除。

## 6. ESO 与 pitch 的职责

`framework/algorithm/observer/eso.hpp` 已存在，但 yaw 尚未接入。ESO 是后续残差处理，不替代已知惯性前馈。

若以后增加 yaw ESO，需选择与两轴耦合输入矩阵一致的观测模型；不能仅取 `diag(M^-1)` 的标量 ESO 输入就称为完整两轴模型。对已知前馈的扣除与执行时序必须与残差动力学定义一致，使用实际限幅后的命令。

大小 yaw 交叉惯性项与 yaw→pitch 耦合是两件事：前者在 `YawNode` 输出两轴力矩中处理，后者在 `PitchNode` 中处理。详见 [Pitch 俯仰控制规划](pitch俯仰控制规划.md) 第 7 节。

## 7. 实施与验收

| 阶段 | 工作 | 验证 |
|---|---|---|
| 0 | 保存现有世界角 LQR 基线，记录原参数与模型 | 可复现限内、超限和自转行为 |
| 1 | 核对 Jb/Js、armature、阻尼及 pitch 姿态影响 | 固定其他自由度的模型矩阵与解析矩阵对照；再测自由耦合响应 |
| 2 | IMU 坐标专项、连续参考、速度/加速度、同步与滤波 | 跨 ±pi 无加速度尖峰；软限交接无不合理参考跳变 |
| 3 | 静止底盘下加入两路惯性前馈 | 分别测大轴起停、小轴起停，正反方向对照 |
| 4a | 云台 IMU 与三关节编码器解析底盘角速度，生成正确参考速度，保留软限分配 | A→B 对照，特别检查大轴持续反转与小轴世界速率反馈 |
| 4b | 完整小陀螺惯性前馈，避免与已有前馈重复 | B→C 对照，起转、匀速、刹停和软限交接，正反方向验收 |
| 5 | pitch 倾斜及越野回归 | 世界 yaw 速率坐标正确；模型适用范围和残差有记录 |

高频记录：

```text
t_sim, dt, beta_est, beta_dot, beta_dd, data_age
psi_ref, psi_imu, yaw_world_error
qb, qs, qb_ref, qs_ref, small_required, allocation_excess
qb_dot_ref, qs_dot_ref, qb_dd_ref, qs_dd_ref
big_fb, small_fb, big_self_ff, big_cross_ff, small_self_ff, small_cross_ff
big_base_ff, small_base_ff, big_raw, small_raw, big_sent, small_sent, saturation
```

固定输入、初值和实际运动速度，对比开关补偿的：世界 yaw 峰值/RMS、稳定时间、大轴保持误差、小轴软限越界幅度、力矩峰值与限幅占比。建议耦合冲击误差比基线下降 50%，同时报告绝对误差，不能单凭相对改善宣称整体通过。

必须满足：

- 小轴目标需求在软限内，大轴参考保持不变；小轴动作时允许大轴输出反作用补偿力矩。
- 大轴参考固定时实际大轴偏移应减小；大轴主动起停时世界指向误差应减小。
- 超限跟随时只分配必要差额，不因前馈/反馈新增大轴参考。
- 无持续振荡，不依赖长期力矩饱和维持表现。
- 正反方向均有效，跨 ±pi 不反向甩圈。

## 8. 本次固定基座实现与验证记录

源码 `YawNode::update_inertia_feedforward()` 实现参考导数及两轴前馈；`control_tick()` 在原 LQR 输出后叠加并沿用原电机力矩限幅。Q/R、惯量参数值和角度交接参数未调整；原实现备份在本次会话 `/tmp/dust_yaw_before_inertia.hpp`，正式 Git 差异也可审阅。

无 GUI 对照使用当前 MJCF，经 MuJoCo spec 临时删除底盘 freejoint 固定基座；pitch 用保持力矩维持零位附近，正式模型文件未修改。使用实际控制器增益和前馈函数，测试驱动直接读取仿真关节状态、合成力矩并推进 1 ms 物理步长，不经过 ROS 传输与定时器。

两组测试各运行 6 s，前 4 s 使用平滑余弦往返参考，后 2 s 保持。大轴初始参考为 0，小轴交接角仅在测试中设为 60°以覆盖交接；不修改用户 YAML 的交接参数。对比开关同时包含惯性前馈及大轴参考速度修正，不将结果单独归因于交叉项。

| 测试 | 配置 | 世界 yaw 峰值 / RMS | 大轴跟踪峰值 | 原始力矩峰值 |
|---|---|---|---|---|
| 28.6°，限内小轴动作 | 关闭 | 0.03583° / 0.01974° | 0.00329° | 0.00290 N·m |
| 同上 | 开启 | 0.00160° / 0.00054° | 0.00084° | 0.00323 N·m |
| 114.6°，跨软限交接 | 关闭 | 0.13657° / 0.07646° | 3.32711° | 0.16381 N·m |
| 同上 | 开启 | 0.03457° / 0.00326° | 0.59854° | 0.76199 N·m |

限内测试同时断言大轴参考保持零；跨 ±pi 导数与自转禁用/清零逻辑单独检查。以上证明固定基座模型测试中的收益，未证明自由底盘、小陀螺或实车效果，也未完成 IMU 速率坐标专项、时间同步或 ESO。

仍需验证交接附近力矩峰值增大是否在用户实际操作中可接受，以及启动/急停、用户实际软限、pitch 非零和消息延迟的影响。


## 9. 底盘轮速话题补偿实现记录

主链路改为 `joint_states 实测轮速 → ChassisNode → /chassis/velocity → YawNode`。不增加底盘 IMU，不用小陀螺指令代替真实反馈。yaw 中原来的云台 IMU 三轴刚体反解已移除，也不再读取 pitch 编码器；云台 IMU 仍用于世界角反馈和正确的航向速率计算。

底盘轮速目标分配与测量反解共用 `ux/uy/m` 几何：

```text
r*dq_i = ux_i*vx + uy_i*vy + m_i*wz
[vx,vy,wz] = least_squares(H, r*dq_measured)
```

启动时检查 H 满列秩并预计算逆映射。每帧全部轮速有效才发布，缺轮或非有限值不拼接历史值。消息为 `framework/msg/ChassisVelocity`，`header.frame_id=base_link`，保留关节消息时间戳；`vx_m_s/vy_m_s` 为底盘系线速度，`wz_rad_s` 为底盘局部 Z 角速度。

配置：chassis 的 `velocity_topic` 与 yaw 的 `base_velocity_topic` 默认均为 `/chassis/velocity`；`yaw_base_rate_comp_enable` 控制补偿，`yaw_base_rate_timeout_s=0.1` 检查接收间隔及与云台姿态样本的时间差。无有效速度时停用该前馈，保留世界角反馈。陈旧、重复时间戳或错误坐标系不更新速度。

yaw 使用 `body_ref_rate=world_target_rate-wz_measured`。按原交接规则将速度分给大轴，并按 `body_ref_rate-dq_big_measured` 补偿小轴相对关节阻尼。位置分配、Q/R 与用户软限配置保持不变。固定基座惯性前馈在 Shift / Ctrl、检测到基座转动或底盘速度话题无效时退出；基座加速度力矩项尚未实现。

此处采用平面运动学，底盘局部 z 角速度在倾斜时不必然等于世界航向导数。打滑、离地和轮地接触扰动可使轮速估计偏离真实基座运动。完整自由底盘小陀螺动态验收仍待完成。

验证记录：`ros2_layer` 与 `project` 编译通过；实际 chassis 关节回调 → ChassisVelocity 发布 → yaw 订阅链路测试通过，覆盖纯平移、平移与自转混合、正反自转、超时停用及缺轮不发布。以上验证不等同 MuJoCo 自由底盘动态验收。


## 10. 底盘速度链路与抖动修复

用户运行日志显示 `yaw基座 sample_valid=0 active=0`，但 `yaw惯性 active=1`；参考加速度达到 1004.50839 rad/s²，小轴自身惯性项达到 4.01803 N·m。源码确认底盘测速匹配 `wheel_names` 原值，而实际关节名为该名字加 `_joint`，与原轮速控制读取规则不一致，导致速度话题无输出。之前构造链路测试也错误使用了无后缀名字，未覆盖真实 MJCF 命名，这项测试结论不足。

修复测速匹配为 `<wheel_name>_joint`。惯性前馈不再对由测量姿态计算的分配角做二次差分，改用解析目标速度：大轴取超限分配速度，小轴取 `body_reference_rate-big_reference_rate`；通过原低通再求参考加速度。阻尼前馈中的小轴实际大轴速度抵消仍独立保留。启用底盘速度补偿但话题无效时，固定基座惯性前馈清零退出，避免误把未知基座运动当固定基座。

保留 Q/R、用户软限与力矩限幅。先验证实际 MJCF 关节名、ROS 速度链路、无话题退出与静止参考下测量抖动不产生惯性前馈，再进行用户工况回归。第 8 节旧参考差分实现的测试数字仅作历史记录，不能代表本次修复后的效果；自由底盘消抖效果仍需运行数据确认。

修复验证：真实 MJCF 轮关节命名检查、chassis 到 yaw 的 ROS 速度链路、缺轮与超时退出、缺底盘速度时关闭固定基座前馈、1000 次静止参考测量扰动测试均通过；`ros2_layer` 与 `project` 编译通过。


## 11. IMU 姿态与 gyro 坐标不一致导致速度反馈丢失

新日志显示底盘速度链路有效，但 `gyro_z=4.23011 rad/s` 时闭环航向速度约 `0.0004 rad/s`，底盘阻尼前馈仅约 `0.0004 N·m`。实际 MuJoCo + ROS imu/chassis/gimbal 无窗口复现确认：平放状态下发布的 roll 约为 1.5708 rad，原因是 framequat 使用 `objtype="body"` 返回主惯性坐标系姿态，而 gyro site 与 pitch 刚体安装坐标同向，二者不能直接配对旋转。

将整车 `gimbal_imu_quat` 改为 `objtype="xbody"`，读取刚体安装坐标系，保持传感器名称、安装几何、控制器 Q/R 与速度补偿不变。此变换适用于当前 site 与 body 同向的模型；后续改变 site 朝向时需同时核对姿态源。

相同模型、实际 ROS 进程与自由底盘，5 s 静止无窗口对照：修复前 roll 约 90°、yaw 峰值约 135.96°；修复后 roll 约 0.0011°、yaw 峰值约 0.000165°。该测试通过 ROS 力矩话题闭合物理，不是单独数学函数测试；尚不能替代用户 GUI 工况验收。之前只修底盘话题命名与参考差分，并未解决此速度反馈坐标根因。

补充验证：7 s 同一 ROS/物理闭环，1 s 时给小轴 2.5 rad/s 速度扰动，2～4.5 s 发布 Shift 小陀螺请求后释放；世界 yaw 峰值约 1.7277°，未复现原持续发散。测试使用独立 ROS_DOMAIN_ID=188，不运行 GUI、不修改机械几何。


## 12. 小陀螺软限交接速度参考跳变修复

用户日志中，小轴需求反复在 -119.99° 与 -120.35° 一带经过交接边界，大轴速度参考在 0 与约 -6.2 rad/s 间切换；采样日志最大相邻跳变约 6.2806 rad/s，世界 yaw 误差最高约 41.53°。原逻辑按 `small_required <= -soft_limit` 决定是否给速度参考，噪声或消息到达间隔造成一拍进入软限内时，角度参考只暂停一拍，速度反馈却立即要求大轴停止，下一拍又要求高速反转。

保留位置分配、历史大轴参考、Q/R 与软限参数。使用真正更新的 `reference.big` 与更新前值的最短角差计算参考速度，并使用现有 `yaw_inertia_reference_lpf_hz` 做低通；速度状态放入 `ReferenceState.big_rate`。不增加新限位、回中逻辑或交接滞回。大轴参考角在需求限内时仍不改变，速度参考按滤波状态连续衰减，不再由边界布尔条件硬切换。

验证：`ros2_layer` 与 `project` 编译通过。13 s 实际 ROS imu/chassis/gimbal + 自由底盘 MuJoCo，无 GUI，对照旧/新实现；鼠标先生成约 -11.7° yaw 与 -7.45° pitch，2～11 s 持续 Shift 自转。20 ms 采样日志统计：世界 yaw 峰值误差 2.10°→0.93°、全区间 RMS 0.253°→0.136°；大轴峰值跟踪误差 2.56°→3.13°、力矩峰值 0.934→1.504 N·m，均需结合收益评价。实际控制参数未调整。该测试未复现用户 41.53° 极端偏差，不将其视为所有小陀螺工况验收通过；用户窗口工况仍待复测。旧/新测试进程日志保存在本会话 `/tmp/handoff_old_*.log`、`/tmp/handoff_new_*.log`。

## 13. 2026-10-09：高速弹丸物理步长下的小陀螺时序修正

高速弹丸将物理步长改为 0.1 ms 后，日志显示反馈时间戳离散跳变，而 wall timer 仍以 1 ms 控制。此前大轴参考速度按固定 1 ms 差分，并对重复反馈反复滤波，形成速度尖峰和衰减。另外，分别到达的关节、IMU 角度和角速度会被混用，拼出虚假的底盘朝向变化。

本次修正：

- 大轴参考速度使用相邻新 IMU 样本的仿真时间间隔；同时间戳保持参考速度，不重复衰减。
- `JointSample` 保存时间戳。当前仿真链路的关节、IMU 姿态和角速度均源于同一采样时刻，三者时间戳一致才计算控制；等待配对期间保持此前已发布指令。若后续接入非同步实物传感器，需要另外实现时间对齐缓存/插值，不能直接沿用精确相等判据。
- 不修改大小轴角度分配规则、Q/R、软限或力矩限幅。

隔离 ROS 域的合成反馈对照：底盘恒定 6 rad/s、关节与 IMU 每 7.5 ms 更新、控制定时器 1 ms。匀速末段大轴指令标准差从旧实现约 1.35 N·m 降至配对实现的数值舍入量级，输出稳定于阻尼前馈 -0.012 N·m。该测试验证时间链路，不等于整车接触动力学验收；实际小陀螺 RMS、峰值和新日志仍待确认。

弹丸池已扩至 200 个独立自由体，装填扫描同步扩容。固定云台、1 ms 速度环、0.1 ms 物理步长的模型测试，枪口轴向速度约 20.69 m/s。200 自由体增加物理开销，需要同时观察实际仿真速率与反馈频率。
