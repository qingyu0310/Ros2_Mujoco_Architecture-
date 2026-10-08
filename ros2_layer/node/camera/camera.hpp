/**
 * @file camera.hpp
 * @author qingyu
 * @brief 摄像头显示节点：订阅 ROS 图像，交给 framework 摄像头模块显示
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @note 单线程 executor 保证 GLFW 的创建、上传、显示和销毁都在主线程执行。
 *       本节点只负责图像适配和显示，不采集控制输入。
 */

#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "framework/modules/camera/camera.hpp"

/**
 * @brief 通用摄像头显示节点，话题和窗口配置由项目参数文件提供
 */
class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const std::string& node_name = "camera", const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : rclcpp::Node(node_name, options)
    {
        read_parameters();
        open_window();
        setup_ros_interfaces();
        start_display_timer();

        RCLCPP_INFO(get_logger(), "摄像头窗口：%s -> %s", params_.image_topic.c_str(), params_.window_title.c_str());
    }

    /**
     * @brief 提供显示窗口给键盘节点，图像订阅仍由本节点负责
     */
    GLFWwindow* window() const
    {
        return window_->window();
    }

private:
    /**
     * @brief 图像话题、窗口尺寸和刷新周期
     */
    struct Params
    {
        std::string image_topic {"/gimbal/camera/image_raw"};
        std::string window_title {"Dust Camera"};
        int window_width {1280};
        int window_height {720};
        double refresh_period_s {0.016};
    };

    /**
     * @brief ROS 图像订阅与窗口刷新定时器
     */
    struct RosInterfaces
    {
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::TimerBase::SharedPtr timer;
    };

    Params params_ {};
    RosInterfaces ros_ {};
    std::unique_ptr<modules::camera::Camera> window_;
    std::vector<std::uint8_t> rgb_;

    /**
     * @brief 读取图像和显示参数
     */
    void read_parameters()
    {
        params_.image_topic      = declare_parameter("image_topic",      "/gimbal/camera/image_raw");
        params_.window_title     = declare_parameter("window_title",     "Dust Camera");
        params_.window_width     = declare_parameter("window_width",     1280);
        params_.window_height    = declare_parameter("window_height",    720);
        params_.refresh_period_s = declare_parameter("refresh_period_s", 0.016);

        if (!(params_.refresh_period_s > 0.0))
        {
            throw std::runtime_error("camera: refresh_period_s 必须为正");
        }
    }

    /**
     * @brief 创建 framework 图像显示窗口
     */
    void open_window()
    {
        window_ = std::make_unique<modules::camera::Camera>(params_.window_title, params_.window_width, params_.window_height);
    }

    /**
     * @brief 订阅配置的图像话题
     */
    void setup_ros_interfaces()
    {
        ros_.image_sub = create_subscription<sensor_msgs::msg::Image>(
            params_.image_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(*msg); });
    }

    /**
     * @brief 启动独立显示定时器
     */
    void start_display_timer()
    {
        ros_.timer = create_wall_timer(std::chrono::duration<double>(params_.refresh_period_s), std::bind(&CameraNode::update, this));
    }

    /**
     * @brief 处理窗口事件，关闭窗口时退出当前显示进程
     */
    void update()
    {
        if (!window_->update())
        {
            rclcpp::shutdown();
        }
    }

    /**
     * @brief 校验 rgb8 图像并按行步长复制，去掉行尾填充后上传显示
     * @param msg ROS 图像消息
     */
    void on_image(const sensor_msgs::msg::Image& msg)
    {
        const std::size_t row = static_cast<std::size_t>(msg.width) * 3;
        if (msg.encoding != "rgb8" || !msg.width || !msg.height || msg.step < row ||
            msg.width  > static_cast<unsigned>(std::numeric_limits<int>::max())   ||
            msg.height > static_cast<unsigned>(std::numeric_limits<int>::max())   ||
            static_cast<std::size_t>(msg.height - 1) * msg.step + row > msg.data.size())
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "忽略非法图像或不支持的编码（需要 rgb8）");
            return;
        }

        rgb_.resize(row * msg.height);
        for (std::size_t y = 0; y < msg.height; ++y)
        {
            std::copy_n(msg.data.data() + y * msg.step, row, rgb_.data() + y * row);
        }
        window_->set_rgb(rgb_.data(), static_cast<int>(msg.width), static_cast<int>(msg.height));
    }
};
