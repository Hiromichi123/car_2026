"""
一键启动航点导航系统 (C++ 版)，launch 后自动开始导航。

启动节点:
  1. hardware_bridge_node  — 串口桥接 (/cmd_vel → 电机驱动板)
  2. waypoint_nav          — 航点导航 C++版 (VIO定位 + PID → /cmd_vel)

使用:
  ros2 launch line_follower navigate.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/car_serial"
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200"
    )

    # 硬件桥接节点 (ros2_tools 包)
    hardware_bridge_node = Node(
        package="ros2_tools",
        executable="hardware_bridge_node",
        name="hardware_bridge_node",
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baud_rate": LaunchConfiguration("baud_rate"),
        }],
    )

    # 航点导航节点 (C++ 版)
    waypoint_nav_node = Node(
        package="line_follower",
        executable="waypoint_nav",
        name="waypoint_navigator",
        output="screen",
        parameters=[{
            "kp_angular": 1.8,
            "kp_linear": 0.6,
            "max_linear_speed": 0.4,
            "max_angular_speed": 0.8,
            "slow_down_dist": 0.30,
            "reach_dist": 0.08,
            "lost_timeout": 1.0,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        hardware_bridge_node,
        waypoint_nav_node,
    ])
