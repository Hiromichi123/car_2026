/*
 * 功能:
 * 订阅里程计数据（实机或仿真），处理后发布雷达位姿信息
 * 兼容PointLIO里程计消息格式
 * 发布消息类型: ros2_tools::msg::LidarPose
 * 订阅消息类型: nav_msgs::msg::Odometry
 * 配置参数:
 * - use_simulation (bool): 是否使用仿真里程计，默认true
 * - simulation_odom_topic (string): 仿真里程计话题，默认"/absolute_pose"
 * - real_robot_odom_topic (string): 实机里程计话题，默认"/aft_mapped_to_init"
 * - lidar_forward_offset_m (double): 雷达相对车体中心沿车体X正方向偏置，默认0.135m
 * - reset_origin_on_start (bool): 将首帧车体中心位姿作为局部原点，默认true
 */
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ros2_tools/msg/lidar_pose.hpp"

class LidarDataNode : public rclcpp::Node {
public:
    LidarDataNode() : Node("lidar_data_node") {
        // 参数声明
        this->declare_parameter<bool>("use_simulation", false);
        this->declare_parameter<std::string>("simulation_odom_topic", "/absolute_pose");
        this->declare_parameter<std::string>("real_robot_odom_topic", "/aft_mapped_to_init");
        this->declare_parameter<double>("lidar_forward_offset_m", 0.135);
        this->declare_parameter<bool>("reset_origin_on_start", true);
        // 获取参数值
        using_gazebo_ = this->get_parameter("use_simulation").as_bool();
        std::string sim_topic = this->get_parameter("simulation_odom_topic").as_string();
        std::string real_topic = this->get_parameter("real_robot_odom_topic").as_string();
        lidar_forward_offset_m_ = this->get_parameter("lidar_forward_offset_m").as_double();
        reset_origin_on_start_ = this->get_parameter("reset_origin_on_start").as_bool();
        
        // 设置QoS，某些情况下
        //rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 雷达数据发布
        lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "创建发布 lidar_data");

        // 订阅 PointLIO 里程计（实机模式）
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            real_topic, 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ LidarDataNode::odomCallback(msg);});
        RCLCPP_INFO(this->get_logger(), "创建实机odom订阅: %s", real_topic.c_str());

        // 订阅仿真里程计（仿真模式）
        local_position_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            sim_topic, 10, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ LidarDataNode::odomCallback(msg);});
        RCLCPP_INFO(this->get_logger(), "创建仿真odom订阅: %s", sim_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达前向偏置补偿: %.3f m", lidar_forward_offset_m_);
        RCLCPP_INFO(this->get_logger(), "启动原点重置: %s", reset_origin_on_start_ ? "true" : "false");
    }

    // 兼容版odom回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        msgDispose(msg->pose.pose);
    }

    // 处理并发布LidarPose消息
    void msgDispose(const geometry_msgs::msg::Pose &pose) {
        // 提取位置和姿态
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;
        tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 归一化到0~2π
        if (roll < 0) roll += 2 * M_PI;
        if (pitch < 0) pitch += 2 * M_PI;
        if (yaw < 0) yaw += 2 * M_PI;

        // Point-LIO输出的是雷达中心位姿。雷达位于车体中心X正方向时，
        // 导航需要将位置换算回车体中心。
        const double base_x = x - lidar_forward_offset_m_ * std::cos(yaw);
        const double base_y = y - lidar_forward_offset_m_ * std::sin(yaw);
        if (reset_origin_on_start_ && !origin_ready_) {
            origin_x_ = base_x;
            origin_y_ = base_y;
            origin_z_ = z;
            origin_ready_ = true;
            RCLCPP_INFO(
                this->get_logger(),
                "记录车体中心局部原点: (%.3f, %.3f, %.3f)",
                origin_x_, origin_y_, origin_z_);
        }

        const double out_x = reset_origin_on_start_ ? base_x - origin_x_ : base_x;
        const double out_y = reset_origin_on_start_ ? base_y - origin_y_ : base_y;
        const double out_z = reset_origin_on_start_ ? z - origin_z_ : z;

        // 填充并发布车体中心LidarPose消息
        lidar_pose.x = out_x;
        lidar_pose.y = out_y;
        lidar_pose.z = out_z;
        lidar_pose.roll = roll;
        lidar_pose.pitch = pitch;
        lidar_pose.yaw = yaw;

        lidar_pub->publish(lidar_pose);

        log(out_x, out_y, out_z, roll, pitch, yaw); // 低频打印日志
    }

    // 低频打印当前位姿日志
    void log(double x, double y, double z, double roll, double pitch, double yaw) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000, // 5秒打印一次
            "Position=(%.2f, %.2f, %.2f), Orientation=(%.2f, %.2f, %.2f) rad",
            x, y, z, roll, pitch, yaw);
    }

private:
    bool using_gazebo_; // 仿真开关
    double lidar_forward_offset_m_;
    bool reset_origin_on_start_;
    bool origin_ready_ = false;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.0;
    
    rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_position_sub;

    ros2_tools::msg::LidarPose lidar_pose;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarDataNode>();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lidar_data_node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
