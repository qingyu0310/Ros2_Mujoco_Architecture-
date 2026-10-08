# MuJoCo 官方 release tarball 不带 CMake config，这里手工找头文件和库并建 imported target。
#
# 根目录按下面的顺序取（第一个存在的生效）：
#   1. CMake 变量 MUJOCO_DIR 或环境变量 MUJOCO_DIR
#   2. $ENV{HOME}/.local/mujoco-<版本>，版本由 MUJOCO_VERSION 给，默认 3.14.0
#
# 定义：Mujoco_FOUND、Mujoco::Mujoco

set(MUJOCO_VERSION "3.14.0" CACHE STRING "MuJoCo 版本，用于拼默认安装目录")

if(NOT MUJOCO_DIR)
    if(DEFINED ENV{MUJOCO_DIR})
        set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
    else()
        set(MUJOCO_DIR "$ENV{HOME}/.local/mujoco-${MUJOCO_VERSION}")
    endif()
endif()

find_path(Mujoco_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS "${MUJOCO_DIR}/include")

find_library(Mujoco_LIBRARY
    NAMES mujoco
    HINTS "${MUJOCO_DIR}/lib")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mujoco
    REQUIRED_VARS Mujoco_LIBRARY Mujoco_INCLUDE_DIR
    VERSION_VAR MUJOCO_VERSION)

if(Mujoco_FOUND AND NOT TARGET Mujoco::Mujoco)
    add_library(Mujoco::Mujoco SHARED IMPORTED)
    set_target_properties(Mujoco::Mujoco PROPERTIES
        IMPORTED_LOCATION "${Mujoco_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Mujoco_INCLUDE_DIR}")
endif()

mark_as_advanced(Mujoco_INCLUDE_DIR Mujoco_LIBRARY)
