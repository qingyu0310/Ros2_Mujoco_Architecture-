/**
 * @file camera.hpp
 * @author qingyu
 * @brief 摄像头显示模块：管理 GLFW 窗口与 RGB 图像纹理，不依赖 ROS
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026
 *
 * @note 由调用方在同一线程创建、更新和销毁；每个显示进程持有一个实例。
 */

#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

#include <GLFW/glfw3.h>

namespace modules::camera
{
/**
 * @brief 通用 RGB 图像窗口
 */
class Camera
{
public:
    /**
     * @brief 创建指定尺寸的图像窗口
     */
    Camera(const std::string& title, int width, int height)
    {
        if (width <= 0 || height <= 0 || !glfwInit())
        {
            throw std::runtime_error("camera: 窗口参数非法或 GLFW 初始化失败");
        }
        window_ = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (!window_)
        {
            glfwTerminate();
            throw std::runtime_error("camera: 创建图像窗口失败");
        }
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(0);
    }

    /**
     * @brief 在窗口上下文中释放纹理和 GLFW 资源
     */
    ~Camera()
    {
        glfwMakeContextCurrent(window_);
        if (texture_)
        {
            glDeleteTextures(1, &texture_);
        }
        glfwDestroyWindow(window_);
        glfwTerminate();
    }

    /**
     * @brief 提供窗口句柄，供独立输入模块复用，不转移所有权
     */
    GLFWwindow* window() const
    {
        return window_;
    }

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    /**
     * @brief 上传连续排列的 RGB 图像，调用方负责验证数据尺寸
     */
    void set_rgb(const std::uint8_t* data, int width, int height)
    {
        glfwMakeContextCurrent(window_);
        if (!texture_)
        {
            glGenTextures(1, &texture_);
        }
        glBindTexture(GL_TEXTURE_2D, texture_);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        image_width_ = width;
        image_height_ = height;
    }

    /**
     * @brief 处理事件并等比例绘制图像，关闭窗口或 ESC 时返回 false
     */
    bool update()
    {
        glfwPollEvents();
        if (glfwWindowShouldClose(window_) || glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        {
            return false;
        }
        glfwMakeContextCurrent(window_);
        int width {0};
        int height {0};
        glfwGetFramebufferSize(window_, &width, &height);
        glViewport(0, 0, width, height);
        glClearColor(0.02f, 0.025f, 0.03f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        if (texture_ && width > 0 && height > 0)
        {
            // 等比例显示，窗口与图像比例不同时留黑边。
            const double ratio = static_cast<double>(image_width_) * height / (image_height_ * static_cast<double>(width));
            const double x = ratio < 1.0 ? ratio : 1.0;
            const double y = ratio > 1.0 ? 1.0 / ratio : 1.0;
            glEnable(GL_TEXTURE_2D);
            glBindTexture(GL_TEXTURE_2D, texture_);
            glColor3f(1, 1, 1);
            glBegin(GL_QUADS);
            glTexCoord2f(0, 1);
            glVertex2d(-x, -y);
            glTexCoord2f(1, 1);
            glVertex2d(x, -y);
            glTexCoord2f(1, 0);
            glVertex2d(x, y);
            glTexCoord2f(0, 0);
            glVertex2d(-x, y);
            glEnd();
            glDisable(GL_TEXTURE_2D);
        }
        glfwSwapBuffers(window_);
        return true;
    }

private:
    GLFWwindow* window_ {nullptr};
    GLuint texture_ {0};
    int image_width_ {0};
    int image_height_ {0};
};
}
