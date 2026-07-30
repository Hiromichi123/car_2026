cmake_minimum_required(VERSION 3.8)
project(ros2_tools)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LidarPose.msg"
  "srv/SERVO.srv"
)

add_executable(bsp_node src/bsp_node.cpp)
add_executable(hardware_bridge_node src/hardware_bridge_node.cpp)
add_executable(lidar_data_node src/lidar_data_node.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bsp_node ${cpp_typesupport_target})
target_link_libraries(lidar_data_node ${cpp_typesupport_target})

ament_target_dependencies(bsp_node rclcpp geometry_msgs)
ament_target_dependencies(hardware_bridge_node rclcpp geometry_msgs nav_msgs std_msgs)
ament_target_dependencies(lidar_data_node rclcpp geometry_msgs tf2 tf2_geometry_msgs nav_msgs)

install(TARGETS bsp_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS hardware_bridge_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS lidar_data_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srv DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(PROGRAMS src/pwm.py DESTINATION lib/${PROJECT_NAME} RENAME pwm_node)

ament_package()
