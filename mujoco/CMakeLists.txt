cmake_minimum_required(VERSION 3.8)
project(mujoco)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# MuJoCo 官方 release tarball 不带 CMake config，用本包自带的 FindMujoco 找 SDK
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Mujoco REQUIRED)

# 跟 ros2_layer 一样做成 INTERFACE 库：头文件式实现，本包不产二进制，
# 消费方（project）写
#   #include "mujoco/node/sim/sim.hpp"
#   rclcpp::spin(std::make_shared<SimNode>());
# rclcpp / 消息包 / libmujoco 都从这里透传下去，project 不用自己 find_package。
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<BUILD_INTERFACE:${Mujoco_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${Mujoco_INCLUDE_DIR}>)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_17)
# libmujoco 这里给的是**绝对库路径**而不是 imported target Mujoco::Mujoco：
# 导出的 target 里如果引用 Mujoco::Mujoco，消费端没有这个 target，find_package(mujoco) 会直接报错。
# 代价是路径写死在构建产物里——开发机上认 SDK 装在 MUJOCO_DIR 就行。
target_link_libraries(${PROJECT_NAME} INTERFACE
    "${Mujoco_LIBRARY}"
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS})

# 资源只安装当前实际存在且有内容的目录，避免清理模型文件后还导出旧路径。
# 注意别在下面加 FILES_MATCHING：filter 会把 .xml 全滤掉，装完 share 里什么都没有。
install(DIRECTORY models robot worlds
    DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_targets)
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

ament_export_targets(${PROJECT_NAME}_targets)
ament_export_dependencies(rclcpp sensor_msgs std_msgs)

ament_package()
