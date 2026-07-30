"""
一键启动 MID360 航点导航系统，等待地面站 /mission/command 后开始一圈任务。

启动节点:
  1. livox_ros_driver2     — MID360 驱动
  2. point_lio             — LiDAR/IMU 里程计
  3. lidar_data_node       — Odometry → LidarPose
  4. hardware_bridge_node  — 串口桥接 (/cmd_vel → 电机驱动板)
  5. waypoint_nav_py       — 比赛胶囊路线任务导航 (MID360定位 + PID → /cmd_vel)

使用:
  ros2 launch line_follower navigate.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/car_serial"
    )
    baud_rate_arg = DeclareLaunchArgument(
        "baud_rate", default_value="115200"
    )
    real_robot_odom_topic_arg = DeclareLaunchArgument(
        "real_robot_odom_topic", default_value="/Odometry"
    )

    livox_ros_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            )
        ])
    )

    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_link", "livox_frame"],
    )

    slam = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("point_lio"),
                        "launch",
                        "point_lio.launch.py",
                    ])
                ]),
                launch_arguments={"rviz": "False"}.items(),
            )
        ],
    )

    lidar_data_node = Node(
        package="ros2_tools",
        executable="lidar_data_node",
        name="lidar_data_node",
        output="screen",
        parameters=[{
            "use_simulation": False,
            "simulation_odom_topic": "/absolute_pose",
            "real_robot_odom_topic": LaunchConfiguration("real_robot_odom_topic"),
        }],
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
            "pose_topic": "/lidar_data",
            "pose_source": "lidar_pose",
            "carrier_pose_topic": "/carrier/lidar_pose",
            "car_status_topic": "/car/status",
            "field_start_yaw": 1.57079632679,
            "field_position_yaw_offset": 1.57079632679,
            "field_position_scale_x": 1.0,
            "field_position_scale_y": 1.0,
            "camera_forward_offset_m": 0.0,
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
        real_robot_odom_topic_arg,
        livox_ros_driver,
        tf,
        slam,
        lidar_data_node,
        hardware_bridge_node,
        waypoint_nav_node,
    ])
