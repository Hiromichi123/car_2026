# 小车控制方案
在小车中心前方17cm有一个导航计，/camera/camera/vio_100hz是他自己的位置话题，你需要订阅这个话题来获得小车的位置，注意17cm的位置查，然后可以发布一个小车的pose话题
你可以通过cmd_vel来控制小车速度，注意是差速底盘，控制指令可以如下图所示
你现在需要根据地图写一个控制器，新创建一个功能包就可以，使小车可以按照轨迹跑一圈，小车中心在A点上电，并以A点为原点，坐标系均为右手系，即前为x，左为y
我需要控制小车走D题地图的轨迹，单位均为cm,从A（0,0）到B（150,0），再过一个半径75cm的半圆，到C（150,-150），到D（0,-150），再过半径75cm的半圆回到C（0,0），停车
# 启动
 ros2 run ros2_tools hardware_bridge_node
# 前进 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 50

# 后退 0.2 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 50

# 原地左转 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 50

# 原地右转 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}" -r 50

# 前进+左转（画弧线）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" -r 50

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1


# 1. Source 环境
source ~/AIC_car_2025/install/setup.bash

# 2. 一键启动（串口桥接 + 航点导航），自动开始导航
ros2 launch line_follower navigate.launch.py
