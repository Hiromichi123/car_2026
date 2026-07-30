"""
一键启动航点导航系统，等待地面站 /mission/command 后开始一圈任务。

启动节点:
  1. hardware_bridge_node  — 串口桥接 (/cmd_vel → 电机驱动板)
  2. waypoint_nav_py       — 比赛胶囊路线任务导航 (VIO定位 + PID → /cmd_vel)

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

    # 航点导航节点 (Python 任务版)
    waypoint_nav_node = Node(
        package="line_follower",
        executable="waypoint_nav_py",
        name="waypoint_navigator",
        output="screen",
        parameters=[{
            "mission_command_topic": "/mission/command",
            "carrier_pose_topic": "/carrier/lidar_pose",
            "car_status_topic": "/car/status",
            "field_start_yaw": 1.57079632679,
            "field_position_yaw_offset": 1.57079632679,
            "field_position_scale_x": 1.0,
            "field_position_scale_y": 1.0,
            "auto_position_alignment": False,
            "reset_origin_on_start": True,
            "waypoint_reach_dist": 8.0,
            "final_reach_dist": 5.0,
            "kp_angular": 2.4,
            "kp_linear": 0.35,
            "max_linear_speed": 0.4,
            "max_angular_speed": 0.8,
            "slow_down_dist": 30.0,
            "lost_timeout": 1.0,
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        hardware_bridge_node,
        waypoint_nav_node,
    ])
