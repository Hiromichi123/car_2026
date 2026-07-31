"""
Start the MID360 localization chain without car motion control.

Nodes:
  1. livox_ros_driver2   - MID360 driver, publishes /livox/lidar and /livox/imu
  2. point_lio           - LiDAR/IMU odometry, publishes /aft_mapped_to_init
  3. lidar_data_node     - Odometry -> /lidar_data
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    real_robot_odom_topic_arg = DeclareLaunchArgument(
        "real_robot_odom_topic", default_value="/aft_mapped_to_init"
    )
    point_lio_delay_arg = DeclareLaunchArgument(
        "point_lio_delay", default_value="5.0"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="False"
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
        arguments=["0.135", "0", "0", "0", "0", "0", "1", "base_link", "livox_frame"],
    )

    point_lio = TimerAction(
        period=LaunchConfiguration("point_lio_delay"),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("point_lio"),
                        "launch",
                        "point_lio.launch.py",
                    ])
                ]),
                launch_arguments={"rviz": LaunchConfiguration("rviz")}.items(),
            )
        ],
    )

    lidar_data_node = Node(
        package="ros2_tools",
        executable="lidar_data_node",
        name="car_lidar_data_node",
        output="screen",
        remappings=[
            ("lidar_data", "/car/lidar_data"),
        ],
        parameters=[{
            "use_simulation": False,
            "simulation_odom_topic": "/absolute_pose",
            "real_robot_odom_topic": LaunchConfiguration("real_robot_odom_topic"),
            "lidar_forward_offset_m": 0.135,
            "reset_origin_on_start": True,
            "max_valid_z_m": 0.5,
        }],
    )

    return LaunchDescription([
        real_robot_odom_topic_arg,
        point_lio_delay_arg,
        rviz_arg,
        livox_ros_driver,
        tf,
        point_lio,
        lidar_data_node,
    ])
