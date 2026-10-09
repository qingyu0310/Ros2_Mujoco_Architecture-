cmake_minimum_required(VERSION 3.8)
project(project)

find_package(ament_cmake REQUIRED)
# 仿真节点把物理放在另一个线程（界面在主线程）
find_package(Threads REQUIRED)
# 不写 find_package(Eigen3)：Eigen 由 framework 经 ament_export_dependencies 透传上来
find_package(framework REQUIRED)
# ros2_layer 的通用节点（ImuNode 等）从这里来
find_package(ros2_layer REQUIRED)
# mujoco 层的通用仿真节点（SimNode）从这里来，libmujoco 也随它透传
find_package(mujoco REQUIRED)
# SimNode 自己在进程里开 GLFW 窗口（画面和物理同一份），所以要 glfw
find_package(glfw3 REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# libmujoco 不装在系统前缀下（默认 ~/.local/mujoco-3.14.0/lib），安装时 CMake 会把构建期的
# rpath 剥掉，结果 ros2 run 起不来（error while loading shared libraries: libmujoco.so）。
# 打开这个开关，链接进去的、不在构建树里的库目录会自动写进安装后的 rpath。
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
# 键盘节点用它找 share/project/params/keyboard.yaml
find_package(ament_index_cpp REQUIRED)

# 节点可执行目标加在这里（node/ 是源码，不进 install）：
#   add_executable(<node> node/<方向>/<文件>.cpp)
#   ament_target_dependencies(<node> rclcpp ...)
#   target_link_libraries(<node> framework::framework)
#   install(TARGETS <node> DESTINATION lib/${PROJECT_NAME})

# 链 ros2_layer::ros2_layer 就够了：rclcpp / 消息包 / framework（含 Eigen）都由它透传下来。
# 要单独用 framework 的算法头时再写 framework::framework（上面已经 find_package 过）。
add_executable(imu node/imu/imu.cpp)
target_link_libraries(imu ros2_layer::ros2_layer)
install(TARGETS imu DESTINATION lib/${PROJECT_NAME})

# 仿真：MuJoCo 节点。跑物理、开界面、收左右轮力矩、发 /joint_states，顶替原来 Gazebo 的位置。
# 界面是 MuJoCo 自带的 simulate，而它**不以库的形式发布**，官方集成（ros-controls/mujoco_ros2_control）
# 的做法也是"直接从本地 MuJoCo 安装编它的源码"，这里跟着走：下面这几个 .cc 就是 bin/simulate 那套。
# find_package(mujoco) 走的是 install 里的 config，拿不到 SDK 源码位置，所以这里自己找那个目录。
# lodepng 只有 simulate.cc 存截图用，但编就得有头（apt: liblodepng-dev）
find_path(MUJOCO_SIMULATE_DIR
    NAMES simulate.cc glfw_adapter.cc
    HINTS "$ENV{MUJOCO_DIR}/simulate" "$ENV{HOME}/.local/mujoco-3.14.0/simulate")
if(NOT MUJOCO_SIMULATE_DIR)
    message(FATAL_ERROR "找不到 MuJoCo 的 simulate 源码目录：设 MUJOCO_DIR，或把 SDK 装在 ~/.local/mujoco-3.14.0")
endif()
get_filename_component(MUJOCO_SDK_DIR "${MUJOCO_SIMULATE_DIR}" DIRECTORY)

add_executable(sim
    node/sim/sim.cpp
    ${MUJOCO_SIMULATE_DIR}/simulate.cc
    ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc
    ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
    ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc)
target_include_directories(sim PRIVATE ${MUJOCO_SDK_DIR})
target_link_libraries(sim ros2_layer::ros2_layer mujoco::mujoco glfw lodepng Threads::Threads)
install(TARGETS sim DESTINATION lib/${PROJECT_NAME})

# 键盘遥控：读终端方向键，发按键状态给底盘。要自己在有终端的窗口里前台跑
add_executable(keyboard node/keyboard/keyboard.cpp)
target_link_libraries(keyboard ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp glfw OpenGL::GL)
install(TARGETS keyboard DESTINATION lib/${PROJECT_NAME})

# 摄像头显示：framework 图像窗口 + ros2_layer 图像订阅适配。
add_executable(camera node/camera/camera.cpp)
target_link_libraries(camera ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp glfw OpenGL::GL)
install(TARGETS camera DESTINATION lib/${PROJECT_NAME})

# 云台入口：同一进程里启动 yaw / pitch 两个 ROS 节点。
add_executable(gimbal node/gimbal/gimbal.cpp)
target_link_libraries(gimbal ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp)
install(TARGETS gimbal DESTINATION lib/${PROJECT_NAME})

# 底盘：第一人称云台视角驾驶，枪管指哪往哪走。节点本体在 ros2_layer 的
# ros2_layer/node/chassis/chassis.hpp
add_executable(chassis node/chassis/chassis.cpp)
target_link_libraries(chassis ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp)
install(TARGETS chassis DESTINATION lib/${PROJECT_NAME})

# 发射（摩擦轮）：左键长按送弹。节点本体在 ros2_layer/node/shooter/shooter.hpp
add_executable(shooter node/shooter/shooter.cpp)
target_link_libraries(shooter ros2_layer::ros2_layer ament_index_cpp::ament_index_cpp)
install(TARGETS shooter DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY params launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
