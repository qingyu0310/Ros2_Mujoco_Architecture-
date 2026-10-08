# 大小 yaw 联动控制规划

> 目标：小 yaw 是主控级（快速指向），大 yaw 是随动级（承担大范围转动）。
> 小 yaw 在限位内自主转动；相对角度顶到限位后，超出部分由大 yaw 承担，实现"小 yaw 带动大 yaw"。

> **单位约定**：所有参数（yaml、模型 XML）一律用**度数**（参数带 `_deg` 后缀；XML 有 `angle="degree"`）；
> 节点内部进参数就转弧度，控制、计算、wrap 全部用 **rad**。

## 1. 需求拆解

定义（角度都是相对上一级 body 的关节角，rad）：

- `q_big`：大 yaw 相对底盘的转角
- `q_small`：小 yaw 相对大 yaw 的转角
- `θ_des`：期望指向（相对底盘坐标系，来自遥控/视觉/上位机）
- `L_soft`：小 yaw **软约束**（控制器限幅用，±60° = ±1.0472）
- `L_hard`：小 yaw **物理约束**（模型 joint range，±120° ≈ ±2.0944），`L_hard > L_soft`
- `δ = L_hard - L_soft`：软约束允许被短暂超出的小幅余量

行为要求：

| 情况 | 小 yaw | 大 yaw |
|---|---|---|
| 期望指向在软约束内可达 | 自主转到期望方向 | 不动 |
| 期望指向超出软约束 | 顶到软约束边，允许瞬态小幅越界（见 §2.3） | 跟着转，承担超出部分 |
| 期望指向超出物理约束 | 被 joint range 硬挡住 | 承担到物理边界为止 |
| 指向回中 | 回到软约束内 | 尽量停在当时位置；若小 yaw 会超软约束，则继续承担必要差额 |

小 yaw 实际指向（相对底盘）= `q_big + q_small`，控制目标就是让它跟上 `θ_des`。

## 2. 双层约束设计

### 2.1 软约束（控制器内限幅）

联动分配时小 yaw 目标被 clamp 在 `±L_soft` 内，正常情况下永远不碰物理边界，无冲击。

### 2.2 物理约束（模型 joint range）

`small_yaw_joint` 在 MJCF 里加 `range="-L_hard L_hard"`（`autolimits` 已开，写了 range 自动 limited）。
它是最后一道保险：控制器失效、外部撞击、初始化错位等异常情况下硬挡住，防止机械上不可能的转角。

### 2.3 允许瞬态越过软约束

软约束不是铁丝网：目标分配 clamp 在 `±L_soft`，但**实际角度允许在动态过程中小幅越界**（惯性冲过、被外力推过、大 yaw 加速时的耦合反拖），只要不碰 `L_hard`。
越界后的处理：分配律检测到 `|q_small| > L_soft` 时，把小 yaw 目标拉回 `±L_soft`、同时把差值继续转嫁给大 yaw——即越界是"被允许的瞬态"，系统会自己收敛回软约束内，而不是报错或急停。

```
e           = θ_des - q_big
q_small_des = clamp(e, -L_soft, +L_soft)          # 目标分配用软约束
θ_big_des   = θ_des - q_small_des
# q_small 实际值允许短暂进入 (L_soft, L_hard) 区间；分配律天然把它往回拉
```

## 3. 两条实现路线

### 3.1 路线 A：控制器限幅（推荐）

联动分配逻辑在控制节点里算；同时给小 yaw 加物理 joint range（`L_hard`）作保险，形成"软约束 + 物理约束"双层结构（见 §2）：

```
e           = θ_des - q_big            # 期望方向相对大 yaw 的角度
q_small_des = clamp(e, -L_soft, +L_soft)   # 小 yaw 目标：软约束内吃掉全部
θ_big_des   = θ_des - q_small_des      # 大 yaw 目标：承担超出的部分
```

两个关节各自用 LQR+ESO 跟踪自己的目标（见 §4，输出力矩写 `/motor/big_yaw/cmd_force`、`/motor/small_yaw/cmd_force`）。

- 软约束内：`e` 没超限时 `θ_big_des = q_big`，大 yaw 目标就是它当前位置 → 不动；小 yaw 独自跟踪。
- 超限时：`q_small_des` 被 clamp 在软约束上，差额自然落到 `θ_big_des` → 大 yaw 启动跟随。
- 瞬态越过软约束：分配律把小 yaw 目标拉回 `±L_soft`，越界部分自动收敛（§2.3）。

优点：行为精确可调（限位是软参数，随时改）、大 yaw 跟随速度可控、没有硬接触冲击。
缺点：大 yaw 是"主动跟随"而不是被物理拖着走（行为等价，但语义不是"带动"）。

### 3.2 路线 B：物理限位硬接触

给 `small_yaw_joint` 加 `range="-L L"`（MJCF joint range），大 yaw 电机只给阻尼/弱回中甚至不给力。
小 yaw PD 跟踪 `θ_des`，顶到限位后 MuJoCo 的约束反力通过限位接触把扭矩传给大 yaw，物理上拖着它走。

优点：真·"带动"，逻辑极简。
缺点：限位是模型参数不是软参数；接触冲击大、易抖动；大 yaw 行为不可控（跟多远、停不停都由接触力学决定）；调试成本高。

**建议走路线 A**，B 只作为对比验证存在。

## 4. 控制律：云台动力学建模 + LQR + ESO

### 4.0 云台动力学建模（LQR 的 A/B 从这里来）

大小 yaw 是串联两自由度机械臂结构（大 yaw 扛着小 yaw 整体转），动力学：

```
M(q)·q̈ + C(q,q̇)·q̇ + g(q) = τ + d
```

- `q = [q_big, q_small]ᵀ`，`τ = [τ_big, τ_small]ᵀ`
- `M(q)`：2×2 关节空间惯量矩阵，**非对角元不为零**——小 yaw 的惯量会耦合进大 yaw（而且 M₁₁ 随 q_small 变化，小 yaw 上指示棒偏心时更明显）
- `C(q,q̇)q̇`：科氏/离心力矩（大 yaw 转得快时小 yaw 受牵连）
- `g(q)`：重力矩（质心不在转轴上时才有）
- `d`：摩擦、模型误差等剩余扰动（ESO 管）

**M(q) 的来源：解析式，控制器自己算**——不订阅仿真话题。实车上没有"发惯量矩阵的人"，模型在建模时就是定的，仿真/实车用同一份解析式：

两级 yaw 的转轴都是竖直 Z 轴且同轴（small_yaw_link 的 pos xy 都是 0），几何关系让 M 特别简单：

```
M = [ J_big + J_small   J_small ]        # 两轴同轴：小 yaw 全部惯量都背在大 yaw 上
    [ J_small           J_small ]

J_big   = 大 yaw 本级绕 Z 的惯量（圆柱 ≈ ½mr²）
J_small = 小 yaw 整体绕 Z 的惯量
```

两个 J 都是**常数**，从 MJCF 的质量和尺寸直接算出来填进 `gimbal.yaml`（改模型尺寸时同步改，公式写在参数注释里）。
不完美的地方——指示棒让小 yaw 惯量随 q_small 轻微不对称、质心微小偏移——量级很小，归 ESO 的集总扰动。

**线性化**（供 LQR）：取状态 `x = [q_big, q_small, dq_big, dq_small]ᵀ`，输入 `u = τ`：

```
q̈ = M(q)⁻¹·(u - C·q̇ - g + d)
```

把非线性项全归进扰动，得到 LTI 模型 `ẍ = A_c x + B_c u`：

```
A_c = [0 I; 0 0]（4×4）,   B_c = [0; M(q)⁻¹]（4×2）
```

M 是**常数矩阵**（两个 J 都是常数），所以 LQR 增益在节点启动时解一次就固定了，没有增益调度问题。
两路独立的"双积分器 + ESO"（b = 1/J 对角近似）降级为对照方案，不再是主线。

### 4.1 LQR 反馈（复用 framework）

framework 已有离散 LQR 模板 `framework/algorithm/controller/lqr.hpp`
（`algorithm::controller::Lqr<4, 2>`，迭代解 DARE，控制律 `u = -K(x - target)`），
gimbal 节点直接复用，不自己解 Riccati：

- 按控制周期 T 离散化：`A = I + A_c·T`，`B = B_c·T`（前向欧拉，1kHz 下足够）
- 节点启动时（或 M 更新时）`configure(A, B, Q, R)` + `solve()`，之后每拍 `update(state, target)`
- Q 对角 4×4（位置/速度权重），R 对角 2×2（两个通道力矩权重），全部走参数
- target 里角度来自分配律（§4.5），**速度目标不是 0**：θ_des 是键盘积分的运动目标，θ̇_des 已知
  （`input × max_turns_per_second × 2π`），按分配比例拆给两级喂进 target，否则跟踪运动目标有恒定滞后

### 4.2 ESO（扩张状态观测器，framework 新建）

**ESO 目前 framework 里没有，需要新建 `framework/algorithm/observer/eso.hpp`**——
和 `controller/lqr.hpp`、`filter/kalman/ekf.hpp` 平级，纯头文件模板（`algorithm::observer` 命名空间），
风格照 lqr.hpp：模板化维数/标量、构造配置、update() 单拍推进。framework 是 INTERFACE 库，
加头文件即可，CMake 不用动。

耦合模型里 M 的非对角项由 LQR 管，ESO 负责剩下的：**C(q,q̇)·q̇ 科氏/离心项、g(q) 重力矩、
摩擦、M 摄动、外部扰动**——两通道各一个三阶线性 ESO，离散化实现（与节点控制周期一致）：

```
e  = z1 - q
z1 += T·(z2 - β1·e)
z2 += T·(z3 - β2·e + b_eff·τ)
z3 += T·(-β3·e)
```

`b_eff` 用 `M⁻¹` 的对角元（常数，启动时算一次），带宽参数化：β1=3ωo, β2=3ωo², β3=ωo³，只调一个 `ωo`。
**ωo 有上限**：1kHz 采样下奈奎斯特 500Hz，ωo 给到 40 rad/s 时连续公式算出的 β 在离散积分里已贴稳定边界；
带宽要高就换离散极点配置（β 直接按离散特征方程算），这条写进 eso.hpp 的注释里。
**ESO 输入必须用 clamp 后的 τ**（抗饱和，见 §4.5 第 5 步），否则力矩饱和时 z2/z3 按假输入积分，观测器发散。
输出力矩：

```
τ = u_lqr - M(q)·ẑ3      # u_lqr 是 LQR 给出的 2 维力矩；ẑ3 是每通道扰动加速度，乘回 M 变成力矩抵消
```

### 4.3 期望指向生成：键盘 → θ_des

**来源**：键盘节点发左右方向键（范围 -1.0 ~ 1.0），gimbal 节点订阅后积分成 `θ_des`。

**转换**：参数 `max_turns_per_second` 表示满偏（|input|=1）时期望 1 秒转多少圈。控制周期 1kHz（0.001s），每拍增量：

```
Δθ      = input × max_turns_per_second × 2π × control_period_s
θ_des  += Δθ
```

例：`max_turns_per_second = 0.5`、满偏时每拍转 0.5 × 2π × 0.001 ≈ 0.00314 rad。

**角度缠绕（越 0 / 越 π 区间）**：`θ_des` 维护在 (-π, π]，积分后做 wrap：

```
θ_des = wrap_pi(θ_des)    # wrap_pi(a) = atan2(sin a, cos a)
```

缠绕必须贯穿整条链路，所有"角度差"都要用最短路径差，禁止直接相减：

```
err(a, b) = wrap_pi(a - b)     # 结果 ∈ (-π, π]
```

具体落点：

- 分配律：`e = wrap_pi(θ_des - q_big)`
- LQR 误差：`q_err = wrap_pi(q - q_des)`（状态 q 不做 wrap，误差做）
- ESO 的 `z1 - q` 不跨界（z1 跟 q 同域），不用 wrap

这样 θ_des 从 +π 积到 -π（或反向越 0）时，跟踪走最短方向，不会出现"目标从 π 跳到 -π、云台反向甩一整圈"。

### 4.4 节点接口

| 方向 | 话题 | 内容 |
|---|---|---|
| 订阅 | `/joint_states` | 读 `big_yaw_joint`、`small_yaw_joint` 的 position/velocity（按名字查，不按索引） |
| 订阅 | `/keyboard`（framework/msg/KeyboardState，已确认：`left`/`right` 两个 bool） | input = right - left ∈ -1~1 |
| 发布 | `/motor/big_yaw/cmd_force`、`/motor/small_yaw/cmd_force` | std_msgs/Float64 力矩指令（SimNode 已按 actuator 名自动建好） |

### 4.5 每拍计算流程（固定周期，1kHz）

1. 从 joint_states 取 `q_big`、`q_small`、`dq_big`、`dq_small`；M 是常数，启动时算好 M⁻¹
2. θ_des 积分：`ω_des = input × max_turns_per_second × 2π`（再做速率限制），`θ_des = wrap_pi(θ_des + ω_des × control_period_s)`
3. 分配律：`e = wrap_pi(θ_des - q_big)`，`q_small_des = clamp(e, ±L_soft)`，`θ_big_des = θ_des - q_small_des`；
   速度目标按分配比例拆：`q̇_small_des = ω_des`（未限幅时），限幅时 `q̇_small_des = 0`、`θ̇_big_des = ω_des`
4. LQR 算 u_lqr，**target 带速度**：`[θ_big_des, q_small_des, θ̇_big_des, q̇_small_des]`（A/B 启动时用 M 离散化好，角度误差用 wrap_pi）
5. 合成 `τ_raw = u_lqr - M·ẑ3`，clamp 到各自 ctrlrange 得 τ；**clamp 后的 τ 回喂 ESO**（抗观测器饱和）
6. 两通道 ESO 更新（用 clamp 后的 τ、当前 q、b_eff = diag(M⁻¹)），发布 τ

### 4.6 参数文件 `project/params/gimbal.yaml`（风格照 keyboard.yaml：数值带小数点）

```yaml
/gimbal:
  ros__parameters:
    control_period_s: 0.001
    max_turns_per_second: 0.5       # 键盘满偏时 θ_des 的转速（圈/秒）
    # 单位约定：参数一律度数（_deg 后缀），节点内部转弧度再控制；模型 XML 同理（angle="degree"）
    target_rate_limit_deg: 180.0    # ω_des 速率上限（°/s），防键盘满偏太猛
    small_yaw_soft_limit_deg: 60.0  # 软约束 L_soft；物理约束 L_hard 在模型 joint range（±120°）
    j_big: 0.00032                  # 大 yaw 本级绕 Z 惯量：圆柱 ½mr² = ½×0.15×0.065²（改模型同步改）
    j_small: 0.00002                # 小 yaw 绕 Z 惯量：½×0.05×0.03² + 指示棒修正（量级小，残余归 ESO）
    q_pos: [10.0, 20.0]             # LQR Q 位置权重（大、小 yaw）
    q_vel: [1.0, 1.0]               # LQR Q 速度权重
    r: [1.0, 1.0]                   # LQR R 力矩权重（大、小 yaw）
    eso_bandwidth_big: 20.0         # ωo，离散实现下必须 ≪ 奈奎斯特（500 Hz），大带宽换离散极点配置
    eso_bandwidth_small: 40.0
```

`θ_des` 由键盘左右键积分生成（§4.3），验证阶段也可以 `ros2 topic pub` 键盘输入话题手动给值。

## 5. 执行阶段

### 5.1 阶段 1：模型与数据通路（先让数据流起来）

1. 模型：给 `small_yaw_joint` 加物理约束 `range="-120 120"`（±120°，`mujoco/robot/square_chassis.xml`）✅ 已完成
   **坑**：`<compiler angle="degree"/>` 下 range 按**度数**解析，写弧度数值会被当成度数（±2.09°），小 yaw 卡死在 4.6°
2. ~~SimNode 发布惯量矩阵~~（**已否**：实车没有惯量话题，M 用解析式，两个常数 J 填进 gimbal.yaml，见 §4.0）
3. 键盘接口已确认：`/keyboard`，framework/msg/KeyboardState 六个 bool，`input = right - left` ✅ 已完成

**完成判据**：模型带 joint range 能加载（已验证）；`ros2 topic echo /keyboard` 能看到左右键。

### 5.2 阶段 2：gimbal 节点骨架（能订阅、能发力矩）

4. `project/node/gimbal/gimbal.cpp`：节点骨架，订阅 `/joint_states`（按名字建索引）、`/keyboard`；发布 `/motor/big_yaw/cmd_force`、`/motor/small_yaw/cmd_force`
5. `project/params/gimbal.yaml` 参数文件（§4.6）
6. CMake 加 `gimbal` 可执行目标（照 keyboard 的写法），params 已在 install 规则里
7. `project/launch/sim.launch.py` 增加 gimbal 节点（launch 里先只起 sim + gimbal + keyboard 说明）

**完成判据**：launch 起来后键盘左右键能让两个 yaw 按联动分配转动，joint_states 正常回读。

### 5.3 阶段 3：联动分配律（大小 yaw 行为正确）

8. 实现 θ_des 积分 + wrap_pi（§4.3）、分配律 clamp（§4.5 第 2~3 步）
9. 双层约束联调：软约束 clamp、物理 joint range、瞬态越界收敛（§2）

**完成判据**：过验证 §5 的 1~4 项（软约束内 / 超软约束 / 越 π 缠绕 / 超物理约束）。

### 5.4 阶段 4：LQR + ESO 控制律（性能和抗扰）

10. 新建 `framework/algorithm/observer/eso.hpp`：三阶线性 ESO 模板（§4.2），风格照 lqr.hpp
11. gimbal 节点接入两通道 ESO（带宽参数化，b_eff 取 diag(M⁻¹)，启动时算一次）
12. gimbal 节点接入 `algorithm::controller::Lqr<4,2>`（framework 现有模板，§4.1），A/B 启动时用参数里的 j_big/j_small 离散化后 configure+solve
13. 合成 `τ = u_lqr - M·ẑ3`，clamp ctrlrange 后发布

**完成判据**：过验证 §5 全部，重点第 6 项抗扰（鼠标拖动后回位）。

### 5.5 阶段 5：收尾

13. 全量验证 §5，记录调好的参数进 yaml
14. 复盘是否下沉通用层、是否需要大 yaw 慢回中（§6）

## 6. 验证方法

启动 `ros2 launch project sim.launch.py`（含 gimbal 节点）后，θ_des 由键盘左右键积分；以下"发到 x rad"指按住方向键到目标角度后松手（也可以直接 pub 键盘话题给固定输入）：

1. **软约束内**：转到 0.5 rad —— 只有小 yaw 动，大 yaw 不动（joint_states 里 `q_big` 基本为零）
2. **超软约束**：转到 3.0 rad（> L_soft=1.0472）—— 小 yaw 顶在 1.0472 附近（允许瞬态小幅越过，收敛后回到 1.0472），大 yaw 跟到约 1.95，两者之和 ≈ 3.0
3. **越 π 缠绕**：按住方向键让 θ_des 从 +3.0 继续越过 +π（变成 -3.14 附近）—— 云台应继续同方向平滑转过去，不允许反向甩一圈；反向越 0 同理
4. **超物理约束**：θ_des 持续打到边界 —— 小 yaw 被 joint range 硬挡在 ±2.0944 以内，大 yaw 持续跟随，不发散、不穿模
5. **回摆**：反向键转回 0 —— `q_big + q_small ≈ 0`；大 yaw 尽量少动，但若小 yaw 会超软约束，大 yaw 必须继续承担必要差额
6. **抗扰**：仿真界面里用鼠标拖小 yaw（SimNode 界面支持外力扰动）——ESO 应把扰动估掉，松手后回到目标指向
7. 看画面里两根橙色指示棒的指向关系是否符合直觉

判据：各步中 `wrap_pi(q_big + q_small - θ_des)` 的稳态误差 < 0.02 rad，无持续振荡；软约束越界只在瞬态出现且幅度 < L_hard - L_soft。

## 7. 待定项

- 键盘输入话题/字段的最终对齐（已定：`/keyboard` KeyboardState，`input = right - left`）
- 大 yaw 是否需要"慢回中"（当前设计：不需要，停在当时位置）
- L_soft / L_hard / max_turns_per_second / target_rate_limit 的最终取值
- **基座假设**：`<4,2>` 把底盘当不动基座，底盘 yaw 运动对云台是扰动（ESO 兜）；车要边动边稳指向需升级 `<6,2>`（加底盘 yaw 状态、θ_des 改世界系），列为升级路径
- ~~云台建模方式~~（已定：解析 M，两轴同轴 → 常数矩阵，j_big/j_small 从 MJCF 算进参数，见 §4.0）
- ~~LQR 增益~~（已定：M 是常数，启动时 configure+solve 一次固定，无增益调度）
- ~~ESO 放哪~~（已定：新建 `framework/algorithm/observer/eso.hpp`，纯头文件模板）
- 控制节点是否下沉通用层（阶段 5 再议）
