/**
 * @file keyboard.hpp
 * @author qingyu
 * @brief 输入采集节点：GLFW 窗口读取 26 键、方向键、Shift 和鼠标位移
 * @version 0.2
 * @date 2026-10-07
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这个节点现在不再读 stdin。鼠标必须有窗口焦点才能读到，所以这里开一个很小的
 *       GLFW 输入窗口：26 键、方向键、Shift、鼠标 dx/dy 都从这个窗口出来。
 * @note 这里只采集并发布输入状态，不解释“前后左右 / 云台”等控制语义。
 *       Shift 的 spin_active 是按住即生效的小陀螺请求；WASD 在 key_down 里由订阅者自己解释。
 */

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>

#include <GLFW/glfw3.h>
#include <rclcpp/rclcpp.hpp>

#include "framework/msg/keyboard_state.hpp"

class KeyboardNode : public rclcpp::Node
{
public:
    explicit KeyboardNode(const std::string& node_name = "keyboard", const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), GLFWwindow* shared_window = nullptr) : rclcpp::Node(node_name, options)
    {
        read_parameters();
        if (shared_window != nullptr)
        {
            // 借用摄像头窗口采集输入，不持有窗口和 GLFW 生命周期。
            glfw_.window = shared_window;
            set_mouse_capture(params_.capture_mouse);
        }
        else
        {
            open_window();
        }

        ros_.pub   = create_publisher<framework::msg::KeyboardState>(params_.topic, 10);

        ros_.timer = create_wall_timer(std::chrono::duration<double>(params_.publish_period), std::bind(&KeyboardNode::update, this));

        RCLCPP_INFO(get_logger(),
                    "输入窗口就绪：发布 26 键/方向键/Shift/鼠标状态到 %s，鼠标灵敏度=(%.4f, %.4f)",
                    params_.topic.c_str(), params_.mouse_sensitivity_x, params_.mouse_sensitivity_y);
    }

    ~KeyboardNode() override
    {
        if (glfw_.initialized && glfw_.window != nullptr)
        {
            glfwMakeContextCurrent(glfw_.window);
            glfwDestroyWindow(glfw_.window);
            glfw_.window = nullptr;
        }
        if (glfw_.initialized)
        {
            glfwTerminate();
            glfw_.initialized = false;
        }
    }

private:
    static constexpr int kLetterCount = 26;

    struct Params
    {
        std::string topic {"/keyboard"};
        std::string window_title {"Dust Teleop"};
        double publish_period {0.001};
        int    window_width {640};
        int    window_height {360};
        bool   capture_mouse {false};
        double mouse_sensitivity_x {0.01};
        double mouse_sensitivity_y {0.01};
    };

    struct GlfwState
    {
        GLFWwindow* window {nullptr};
        bool initialized {false};
        bool fullscreen {false};
        int window_x {0};
        int window_y {0};
        int window_width {640};
        int window_height {360};
    };

    struct KeyState
    {
        std::array<bool, kLetterCount> down {};
        std::array<bool, kLetterCount> prev {};
        std::array<bool, kLetterCount> toggle {};
        bool arrow_up {false};
        bool arrow_down {false};
        bool arrow_left {false};
        bool arrow_right {false};
        bool shift_prev {false};
        bool shift_toggle {false};
        bool f11_prev {false};
        bool escape {false};
    };

    struct MouseState
    {
        bool first_sample {true};
        bool captured {false};
        bool left_prev {false};
        bool right_prev {false};
        bool middle_prev {false};
        double last_x {0.0};
        double last_y {0.0};
    };

    struct RosInterfaces
    {
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<framework::msg::KeyboardState>::SharedPtr pub;
    };

    Params        params_ {};
    GlfwState     glfw_   {};
    KeyState      keys_   {};
    MouseState    mouse_  {};
    RosInterfaces ros_    {};

    void read_parameters()
    {
        params_.topic               = declare_parameter("topic_keyboard_",      "/keyboard");
        params_.publish_period      = declare_parameter("publish_period_s",     0.001);
        params_.window_title        = declare_parameter("window_title",         "Dust Teleop");
        params_.window_width        = declare_parameter("window_width",         640);
        params_.window_height       = declare_parameter("window_height",        360);
        params_.capture_mouse       = declare_parameter("capture_mouse",        false);
        params_.mouse_sensitivity_x = declare_parameter("mouse_sensitivity_x",  0.01);
        params_.mouse_sensitivity_y = declare_parameter("mouse_sensitivity_y",  0.01);
    }

    void open_window()
    {
        if (glfwInit() != GLFW_TRUE)
        {
            throw std::runtime_error("keyboard: GLFW 初始化失败，无法读取鼠标/键盘窗口输入");
        }
        glfw_.initialized = true;

        glfwWindowHint(GLFW_RESIZABLE,  GLFW_FALSE);

        glfw_.window = glfwCreateWindow(params_.window_width, params_.window_height, params_.window_title.c_str(), nullptr, nullptr);
        if (glfw_.window == nullptr)
        {
            throw std::runtime_error("keyboard: GLFW 输入窗口创建失败（没有 DISPLAY/Wayland 会话？）");
        }

        glfwSetWindowUserPointer(glfw_.window, this);
        glfwMakeContextCurrent(glfw_.window);
        glfwSwapInterval(0);
        set_mouse_capture(params_.capture_mouse);
    }

    void set_mouse_capture(bool captured)
    {
        if (glfw_.window == nullptr)
        {
            return;
        }

        mouse_.captured = captured;
        mouse_.first_sample = true;
        glfwGetCursorPos(glfw_.window, &mouse_.last_x, &mouse_.last_y);

        glfwSetInputMode(glfw_.window, GLFW_CURSOR, captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        if (glfwRawMouseMotionSupported() == GLFW_TRUE)
        {
            glfwSetInputMode(glfw_.window, GLFW_RAW_MOUSE_MOTION, captured ? GLFW_TRUE : GLFW_FALSE);
        }

        RCLCPP_INFO(get_logger(), "鼠标%s：按 P 切换，按 ESC 退出仿真", captured ? "锁定" : "释放");
    }

    void update()
    {
        if (glfw_.window == nullptr || glfwWindowShouldClose(glfw_.window))
        {
            rclcpp::shutdown();
            return;
        }

        glfwPollEvents();
        update_fullscreen_toggle();

        framework::msg::KeyboardState msg;
        fill_keyboard_state(msg);
        if (should_exit())
        {
            rclcpp::shutdown();
            return;
        }
        fill_mouse_state(msg);
        fill_legacy_fields(msg);

        ros_.pub->publish(msg);
        // 自建窗口时才清屏；借用模式的绘制完全由摄像头模块负责。
        if (glfw_.initialized)
        {
            glfwMakeContextCurrent(glfw_.window);
            glClearColor(0.06f, 0.08f, 0.10f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            glfwSwapBuffers(glfw_.window);
        }
    }

    void update_fullscreen_toggle()
    {
        const bool f11_down = glfwGetKey(glfw_.window, GLFW_KEY_F11) == GLFW_PRESS;
        if (f11_down && !keys_.f11_prev)
        {
            toggle_fullscreen();
        }
        keys_.f11_prev = f11_down;
    }

    void toggle_fullscreen()
    {
        if (!glfw_.fullscreen)
        {
            GLFWmonitor* monitor = glfwGetPrimaryMonitor();
            const GLFWvidmode* mode = monitor == nullptr ? nullptr : glfwGetVideoMode(monitor);
            if (monitor == nullptr || mode == nullptr)
            {
                RCLCPP_WARN(get_logger(), "找不到主显示器，无法进入全屏");
                return;
            }
            glfwGetWindowPos(glfw_.window, &glfw_.window_x, &glfw_.window_y);
            glfwGetWindowSize(glfw_.window, &glfw_.window_width, &glfw_.window_height);
            glfwSetWindowMonitor(glfw_.window, monitor, 0, 0,
                                 mode->width, mode->height, mode->refreshRate);
            glfw_.fullscreen = true;
        }
        else
        {
            glfwSetWindowMonitor(glfw_.window, nullptr,
                                 glfw_.window_x, glfw_.window_y,
                                 glfw_.window_width, glfw_.window_height,
                                 GLFW_DONT_CARE);
            glfw_.fullscreen = false;
        }
        mouse_.first_sample = true;
        RCLCPP_INFO(get_logger(), "输入窗口%s：按 F11 切换", glfw_.fullscreen ? "已全屏" : "已恢复窗口");
    }

    void fill_keyboard_state(framework::msg::KeyboardState& msg)
    {
        const bool focused = glfwGetWindowAttrib(glfw_.window, GLFW_FOCUSED) == GLFW_TRUE;

        // 多键支持：每拍完整扫描 A-Z，每个键独立维护 down/pressed/toggle，不做互斥选择。
        for (int i = 0; i < kLetterCount; ++i)
        {
            const int glfw_key = GLFW_KEY_A + i;
            const bool down = focused && glfwGetKey(glfw_.window, glfw_key) == GLFW_PRESS;
            const bool pressed = down && !keys_.prev[i];

            keys_.down[i] = down;
            if (pressed)
            {
                keys_.toggle[i] = !keys_.toggle[i];
            }

            msg.key_down[i]    = down;
            msg.key_pressed[i] = pressed;
            msg.key_toggle[i]  = keys_.toggle[i];
            keys_.prev[i]      = down;
        }

        constexpr int kP = GLFW_KEY_P - GLFW_KEY_A;
        if (msg.key_pressed[kP])
        {
            set_mouse_capture(!mouse_.captured);
        }

        keys_.arrow_up    = focused && glfwGetKey(glfw_.window, GLFW_KEY_UP)     == GLFW_PRESS;
        keys_.arrow_down  = focused && glfwGetKey(glfw_.window, GLFW_KEY_DOWN)   == GLFW_PRESS;
        keys_.arrow_left  = focused && glfwGetKey(glfw_.window, GLFW_KEY_LEFT)   == GLFW_PRESS;
        keys_.arrow_right = focused && glfwGetKey(glfw_.window, GLFW_KEY_RIGHT)  == GLFW_PRESS;
        keys_.escape      = focused && glfwGetKey(glfw_.window, GLFW_KEY_ESCAPE) == GLFW_PRESS;

        msg.arrow_up     = keys_.arrow_up;
        msg.arrow_down   = keys_.arrow_down;
        msg.arrow_left   = keys_.arrow_left;
        msg.arrow_right  = keys_.arrow_right;

        const bool shift_down = focused && (glfwGetKey(glfw_.window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(glfw_.window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
        msg.shift_hold    = shift_down;
        msg.shift_pressed = shift_down && !keys_.shift_prev;
        if (msg.shift_pressed)
        {
            keys_.shift_toggle = !keys_.shift_toggle;
        }
        msg.shift_toggle_active = keys_.shift_toggle;
        msg.spin_active = shift_down;  // 小陀螺模式按住即生效；toggle 只保留原始锁存状态给别的订阅者。
        keys_.shift_prev = shift_down;

        // Ctrl 只发布长按状态，窗口失焦时自动释放，不做 toggle。
        msg.ctrl_hold = focused && (glfwGetKey(glfw_.window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                                   glfwGetKey(glfw_.window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS);

        msg.equal = focused && glfwGetKey(glfw_.window, GLFW_KEY_EQUAL) == GLFW_PRESS;
        msg.minus = focused && glfwGetKey(glfw_.window, GLFW_KEY_MINUS) == GLFW_PRESS;
    }

    bool should_exit() const
    {
        return keys_.escape;
    }

    void fill_mouse_state(framework::msg::KeyboardState& msg)
    {
        const bool focused = glfwGetWindowAttrib(glfw_.window, GLFW_FOCUSED) == GLFW_TRUE;

        double x = 0.0;
        double y = 0.0;
        glfwGetCursorPos(glfw_.window, &x, &y);

        msg.mouse_x = x;
        msg.mouse_y = y;
        msg.mouse_captured = mouse_.captured && focused;

        double raw_dx = 0.0;
        double raw_dy = 0.0;

        if (mouse_.first_sample || !focused || !mouse_.captured)
        {
            mouse_.first_sample = false;
        }
        else
        {
            raw_dx = x - mouse_.last_x;
            raw_dy = y - mouse_.last_y;
        }
        mouse_.last_x = x;
        mouse_.last_y = y;

        msg.mouse_raw_dx = raw_dx;
        msg.mouse_raw_dy = raw_dy;
        msg.mouse_dx = std::clamp(raw_dx * params_.mouse_sensitivity_x, -1.0, 1.0);
        msg.mouse_dy = std::clamp(raw_dy * params_.mouse_sensitivity_y, -1.0, 1.0);

        const bool left   = focused && glfwGetMouseButton(glfw_.window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
        const bool right  = focused && glfwGetMouseButton(glfw_.window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
        const bool middle = focused && glfwGetMouseButton(glfw_.window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;

        msg.mouse_left   = left;
        msg.mouse_right  = right;
        msg.mouse_middle = middle;
        msg.mouse_left_pressed   = left   && !mouse_.left_prev;
        msg.mouse_right_pressed  = right  && !mouse_.right_prev;
        msg.mouse_middle_pressed = middle && !mouse_.middle_prev;

        mouse_.left_prev   = left;
        mouse_.right_prev  = right;
        mouse_.middle_prev = middle;
    }

    void fill_legacy_fields(framework::msg::KeyboardState& msg) const
    {
        // 旧字段只承载方向键原始状态，不把 WASD 翻译成方向语义。
        msg.up = msg.arrow_up;
        msg.down = msg.arrow_down;
        msg.left = msg.arrow_left;
        msg.right = msg.arrow_right;
    }
};
