# 兼容旧字段：只表示方向键原始状态，不映射 WASD 或任何控制语义。
bool up
bool down
bool left
bool right
bool equal
bool minus

# 26 个字母键，索引 0=a, 1=b, ..., 25=z；支持多键同时为 true。
# key_down 是长按状态；key_pressed 是本拍按下沿；key_toggle 是每个键各自按下后翻转的锁存状态。
bool[26] key_down
bool[26] key_pressed
bool[26] key_toggle

# 方向键原始状态；旧字段 left/right 仍保留给老控制逻辑。
bool arrow_up
bool arrow_down
bool arrow_left
bool arrow_right

# 修饰键原始状态。
# shift_hold 是长按状态；spin_active 当前等于 shift_hold，用于按住即小陀螺。
# shift_toggle_active 只是按键锁存状态，保留给需要按一下切换模式的订阅者。
bool shift_hold
bool shift_pressed
bool shift_toggle_active
bool ctrl_hold                 # 左右 Ctrl 任一个按住；底盘跟随请求，不锁存
bool spin_active

# 鼠标状态。raw 位移单位是窗口像素；mouse_dx/dy 是乘过 keyboard 节点灵敏度系数并 clamp 到 [-1, 1] 的归一化轴值。
float64 mouse_x
float64 mouse_y
float64 mouse_raw_dx
float64 mouse_raw_dy
float64 mouse_dx
float64 mouse_dy
bool mouse_left
bool mouse_right
bool mouse_middle
bool mouse_left_pressed
bool mouse_right_pressed
bool mouse_middle_pressed
bool mouse_captured
