/**
 * @file keyboard.hpp
 * @author qingyu
 * @brief 终端键盘：非阻塞读方向键（Keyboard），加"按住走、松手停"的遥控映射（Teleop）
 * @version 0.1
 * @date 2026-09-26
 *
 * @copyright Copyright (c) 2026
 *
 * @note 这一层没有 ROS、没有 Eigen：键盘就是一路设备输入，跟机器人无关，别的地方也能用
 * @note 为什么不用 getchar / std::cin：节点是定时器驱动的，读键盘不能把那一拍卡住。
 *       所以把终端设成原始模式的非阻塞读（VMIN=0 / VTIME=0），读不到立刻返回"没键"
 * @note 上下左右是转义序列，非阻塞读下三个字节可能落在不同的 poll() 里，所以状态机得跨调用
 *       记住"上一个字节是 ESC [ 还是 ESC O"，不能指望一次读齐。两种前缀都认：
 *       ESC [ A/B/C/D（普通模式，CSI）和 ESC O A/B/C/D（应用光标键模式，SS3 / DECCKM）
 * @note 关行缓冲和回显（ICANON / ECHO）在析构里恢复。进程要是被 SIGKILL 掉就恢复不了，
 *       那个终端得敲一下 reset
 * @note 两个类分工：Keyboard 只管"现在按了什么"，Teleop 管"按着这个键该给多少速度"。
 *       跟 modules::motor 那边一样，模块拿着状态，节点只负责喂时间和把结果发出去
 */

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include "framework/msg/keyboard_state.hpp"

namespace modules::keyboard
{

// 消息类型在本文件里直接写 KeyboardState 用；字段跟原来的结构体一一对应
using KeyboardState = framework::msg::KeyboardState;

/**
 * @brief 认识的那几个键
 *
 * @note kNone 既是"这一拍没按"也是"按了个不认识的键" —— 对遥控来说两者一样：不用改速度
 * @note kEqual / kMinus 是普通字符（不是方向键那种转义序列）：这一层只认键，怎么用是上层的事
 */
enum class Key : std::uint8_t
{
    kNone  = 0,
    kUp    = 1,
    kDown  = 2,
    kLeft  = 3,
    kRight = 4,
    kEqual = 5,   // '='
    kMinus = 6    // '-'
};

/**
 * @brief 终端键盘：原始模式 + 非阻塞读，把方向键还原成 Key
 *
 * @note 非拷贝：它手里攥着终端设置，拷一份出来析构时会把模式恢复两次
 */
class Keyboard
{
public:
    Keyboard() = default;

    /**
     * @brief 析构时恢复终端设置
     */
    ~Keyboard() { close(); }

    Keyboard(const Keyboard&)            = delete;
    Keyboard& operator=(const Keyboard&) = delete;

    /**
     * @brief 把 stdin 切成原始模式、非阻塞
     *
     * @return bool true 成功；false 说明 stdin 不是终端（被重定向到管道/文件了）或者没权限
     *
     * @note 只关 ICANON 和 ECHO，不动 ISIG：Ctrl-C 还得能杀进程
     */
    bool open()
    {
        if (opened_)
        {
            return true;
        }

        // 不是 tty 就直接失败，别装成"读到了键盘"。ros2 launch 里跑、或者被重定向时就是这种
        if (::tcgetattr(STDIN_FILENO, &saved_) != 0)
        {
            return false;
        }

        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);   // 不等回车、按键不回显
        raw.c_cc[VMIN]  = 0;                                    // 没数据就立刻返回，别阻塞
        raw.c_cc[VTIME] = 0;

        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            return false;
        }

        opened_ = true;
        return true;
    }

    /**
     * @brief 恢复终端原来的设置
     *
     * @note 幂等，析构里也会调
     */
    void close()
    {
        if (!opened_)
        {
            return;
        }
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        opened_ = false;
    }

    bool is_open() const { return opened_; }
    void set_key_timeout(double timeout_s) { key_timeout_s_ = timeout_s; }
    bool is_has_keyboard() const {return has_keyboard_; }

    /**
     * @brief 把这一拍能读到的字节全读掉，返回最后一个认出来的键
     *
     * @return Key 这一拍最后认出来的键；没按键、或者按的是不认识的键，返回 Key::kNone
     *
     * @note 一次读一堆而不是一个字节就返回：终端按住不放时按键会连着重复，一次挤进来好几个，
     *       只处理第一个等于把后面的扔了。这里取"最后一个"——按键重复时它们本来也是同一个键
     * @note 转义序列读到一半（只收到 ESC）时返回 kNone：宁可这一拍不动，也别把半个序列当成键
     */
    Key poll()
    {
        if (!opened_)
        {
            return Key::kNone;
        }

        std::uint8_t byte       = 0;
        Key          last_key   = Key::kNone;

        for (std::size_t i = 0; i < kMaxBytesPerPoll; ++i)
        {
            const ssize_t got = ::read(STDIN_FILENO, &byte, 1);
            if (got <= 0)
            {
                break;   // 0：这一拍没键了；-1：出错或被信号打断，都当没键
            }

            const Key decoded = feed(byte);
            if (decoded != Key::kNone) {
                last_key = decoded;
            }
        }

        return last_key;
    }

    /**
     * @brief 轮询键盘 + 超时判定：给出"这一拍还按着的键"
     *
     * @return KeyboardState 这一拍有效的键（至多一个标志置 1）；没有可沿用的键、或者已判成松手时，
     *         六个标志全 0
     *
     * @note "按住走、松手停"整个落在这一层：这一拍读到新键就刷新 last_key_ 和时间戳；
     *       没读到键时，离上次按键还没超过 key_timeout_s_ 就继续返回上一次那个键，
     *       超过了就把状态清掉、返回全 0
     * @note 时钟也在这层取：steady_clock 单调、跟仿真时间无关（键盘是真实时间上的东西）。
     *       调用方不用喂时间，代价是外面塞不进假时间 —— 想在测试里快进只能真 sleep
     */
    KeyboardState updata()
    {
        const double time = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        const Key tick = poll();

        if (tick != Key::kNone)
        {
            last_key_        = tick;      // 有新输入：记下键，时间戳一起刷新
            last_key_time_s_ = time;
        }

        KeyboardState msg {};

        // 没新输入：没超时就继续上一次，超时就当松手、清空
        if (last_key_ == Key::kNone || (time - last_key_time_s_) > key_timeout_s_)
        {
            last_key_ = Key::kNone;
            return msg;
        }

        switch (last_key_)
        {
            case Key::kUp:
                msg.up = 1;
                return msg;
            case Key::kDown:
                msg.down = 1;
                return msg;
            case Key::kLeft:
                msg.left = 1;
                return msg;
            case Key::kRight:
                msg.right = 1;
                return msg;
            case Key::kEqual:
                msg.equal = 1;
                return msg;
            case Key::kMinus:
                msg.minus = 1;
                return msg;
            default:
                return msg;   // 认不出来的键：一个标志都不置
        }
    }

private:
    // 一拍最多吃这么多字节。按键重复最猛也就几十 Hz，64 够用了；同时也是防呆，
    // 万一 stdin 被接到一个一直有数据的管道上，不至于在这一拍里转不出来
    static constexpr std::size_t  kMaxBytesPerPoll = 64;
    static constexpr std::uint8_t kEsc = 0x1b;

    // 转义序列的解析位置：三个字节可能分三次 poll() 到
    enum class Escape : std::uint8_t
    {
        kGround = 0,   // 不在序列里
        kAfterEsc,     // 刚收到 ESC
        kAfterCsi,     // 刚收到 ESC [，等最后一个字节
        kAfterSs3      // 刚收到 ESC O，等最后一个字节
    };
    Escape  escape_{Escape::kGround};

    termios saved_{};

    bool    opened_ {false};
    bool    has_keyboard_  {false};
    double  key_timeout_s_ {0.70};          // 松手判定：这么久没再收到键就算松开

    // "按住走、松手停"的状态：上一次认出来的键，以及它最近一次出现的时间（调用方喂进来的时钟）
    Key     last_key_        {Key::kNone};
    double  last_key_time_s_ {0.0};

    /**
     * @brief 喂一个字节进状态机
     *
     * @param byte 收到的字节
     * @return Key 这个字节凑齐了一个方向键就返回它，否则 kNone
     */
    Key feed(std::uint8_t byte)
    {
        switch (escape_)
        {
            case Escape::kGround:
            {
                if (byte == kEsc)
                {
                    escape_ = Escape::kAfterEsc;
                    return Key::kNone;
                }
                // 等号/减号是普通字符，不跟着方向键走转义序列那条路
                if (byte == '=')
                {
                    return Key::kEqual;
                }
                if (byte == '-')
                {
                    return Key::kMinus;
                }
                return Key::kNone;
            }
            case Escape::kAfterEsc:
            {
                // 方向键有两条路：ESC [ A（普通模式，CSI）和 ESC O A（开了应用光标键之后，
                // SS3，DECCKM）。默认是前者，但终端设成后者的话只认 CSI 就会"方向键没反应"，
                // 所以两个都收。单独的 ESC（按 Esc 键）和别的转义序列都不管
                if (byte == '[')
                {
                    escape_ = Escape::kAfterCsi;
                }
                else if (byte == 'O')
                {
                    escape_ = Escape::kAfterSs3;
                }
                else
                {
                    escape_ = Escape::kGround;
                }
                return Key::kNone;
            }
            case Escape::kAfterCsi:
            case Escape::kAfterSs3:
                escape_ = Escape::kGround;
                return decode_final(byte);
        }

        return Key::kNone;
    }

    /**
     * @brief 转义序列的最后一个字节 -> 方向键
     *
     * @param byte ESC [ 或 ESC O 后面那个字节
     * @return Key 认识的方向键；其它序列（PageUp、Home、F1~F4 之类）返回 kNone
     *
     * @note CSI 和 SS3 的最后一个字节是同一套字母，所以两条路共用这个解码
     */
    static Key decode_final(std::uint8_t byte)
    {
        switch (byte)
        {
            case 'A':
                return Key::kUp;
            case 'B':
                return Key::kDown;
            case 'C':
                return Key::kRight;
            case 'D':
                return Key::kLeft;
            default:
                return Key::kNone;
        }
    }

};

}   // namespace modules::keyboard
