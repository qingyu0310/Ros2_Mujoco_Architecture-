# 仿真启动：SimNode 从 model 参数拿模型绝对路径，这里指向 install 里 mujoco 包导出的整机文件。
# 整机引用 worlds/empty.xml 地形及 models 下的部件；相对 include 按整机文件路径解析。
# 用法：ros2 launch project sim.launch.py
#       ros2 launch project sim.launch.py robot:=square_chassis.xml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='square_chassis.xml',
        description='mujoco 包 share/robot 下的整机模型文件名',
    )

    model_path = PathJoinSubstitution([
        get_package_share_directory('mujoco'), 'robot', LaunchConfiguration('robot'),
    ])

    sim = Node(
        package='project',
        executable='sim',
        name='sim',
        output='screen',
        parameters=[{'model': model_path}],
    )

    # 云台 IMU 解算：吃 SimNode 发的原始帧 /gimbal/imu，解出姿态再发到 /gimbal/imu/*。
    # gimbal 节点订的是解算结果，不再自己解析四元数。
    imu = Node(
        package='project',
        executable='imu',
        name='imu_gimbal',
        output='screen',
        parameters=[{
            'raw_topic': '/gimbal/imu',
            'output_prefix': '/gimbal/imu',
            'frame_id': 'pitch_link',
        }],
    )

    gimbal = Node(
        package='project',
        executable='gimbal',
        output='screen',
    )

    chassis = Node(
        package='project',
        executable='chassis',
        name='chassis',
        output='screen',
    )

    keyboard = Node(
        package='project',
        executable='keyboard',
        output='screen',
    )

    # Teleop 与 pitch 画面共用窗口；俯视画面独立显示。
    overhead_camera = Node(
        package='project', executable='camera', name='overhead_camera', output='screen',
    )

    # keyboard 是 GLFW 输入窗口；按 ESC 后 keyboard 节点退出。
    # launch 监听到 keyboard 退出后发 Shutdown，让 sim/gimbal/chassis 一起收掉。
    shutdown_on_keyboard_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=keyboard,
            on_exit=[
                EmitEvent(event=Shutdown(reason='keyboard exited')),
            ],
        )
    )

    return LaunchDescription([robot_arg, sim, imu, gimbal, chassis, keyboard, overhead_camera, shutdown_on_keyboard_exit])
