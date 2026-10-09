cmake_minimum_required(VERSION 3.8)
project(framework)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
# 消息生成：modules/msg/ 下的 .msg 编成 C++ 类型。接口放本层是定的（原先"framework 不依赖
# ROS"在这条上让位）。rosidl 判"接口类别"用的是文件紧邻那层目录的名字（见 rosidl_generate_interfaces.cmake），
# 这里紧邻的是 msg，所以身份仍是标准的 framework/msg/KeyboardState
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 纯模板头，没有 .cpp，所以做成 INTERFACE 库：本包不产二进制，只导出 include 路径和 Eigen。
# 这里刻意不叫 ${PROJECT_NAME}：rosidl 生成的 .so 是按"目标名"命名的，而运行时按
# lib<包名>__rosidl_typesupport_<实现>.so 去找，找不到就 abort（"Type support not from this
# implementation"）。所以 rosidl 那边必须占着 ${PROJECT_NAME}，本库改叫 framework_lib，
# 对外仍导出成 framework::framework（靠 EXPORT_NAME），消费方一行都不用改
add_library(framework_lib INTERFACE)
set_target_properties(framework_lib PROPERTIES EXPORT_NAME framework)
target_include_directories(framework_lib INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>)
target_compile_features(framework_lib INTERFACE cxx_std_17)
target_link_libraries(framework_lib INTERFACE Eigen3::Eigen)

# 装成 include/framework/algorithm/... 和 include/framework/modules/...，配合下面的
# ament_export_include_directories(include)，消费方写 #include "framework/modules/sensor/imu.hpp"。
# 这两处必须配对，只改一处就找不到头文件。
# FILES_MATCHING 只挑 .hpp：modules/sensor/ 下的 imu.cpp 是空占位，不会装出去
install(DIRECTORY algorithm modules
    DESTINATION include/framework
    FILES_MATCHING PATTERN "*.hpp")

install(TARGETS framework_lib EXPORT ${PROJECT_NAME}_targets)
install(EXPORT ${PROJECT_NAME}_targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

# 键盘状态与底盘速度消息。目标名必须是包名：生成的 .so 就叫 libframework__rosidl_typesupport_*.so，
# 跟运行时按包名去找的名字一致（上面 INTERFACE 库因此改了名）
rosidl_generate_interfaces(${PROJECT_NAME}
    "modules/msg/KeyboardState.msg"
    "modules/msg/ChassisVelocity.msg"
    DEPENDENCIES std_msgs)

# 把消息的类型支持挂到 INTERFACE 库上：消费方链 framework::framework 就同时拿到消息的头和序列化
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(framework_lib INTERFACE ${cpp_typesupport_target})

# 顺序有讲究，rosidl 那段必须排在 ament_export_targets 之前：生成的 frameworkConfig.cmake 是按
# 下面这个登记顺序挨个 include 导出文件的，而 framework_targetsExport.cmake 里引用了
# framework::framework_interfaces__rosidl_typesupport_cpp —— 那个目标得先被定义出来，
# 否则消费方 find_package(framework) 会直接失败：referenced, but are missing
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}_targets)
ament_export_dependencies(Eigen3 std_msgs rosidl_default_runtime)

ament_package()
