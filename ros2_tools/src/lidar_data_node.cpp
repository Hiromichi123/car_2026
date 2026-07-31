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
 * - max_valid_z_m (double): 地面车定位最大允许Z漂移，超过则丢弃，默认0.5m
 */
#include <chrono>
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
        this->declare_parameter<double>("max_valid_z_m", 0.5);
        // 获取参数值
        using_gazebo_ = this->get_parameter("use_simulation").as_bool();
        std::string sim_topic = this->get_parameter("simulation_odom_topic").as_string();
        std::string real_topic = this->get_parameter("real_robot_odom_topic").as_string();
        lidar_forward_offset_m_ = this->get_parameter("lidar_forward_offset_m").as_double();
        reset_origin_on_start_ = this->get_parameter("reset_origin_on_start").as_bool();
        max_valid_z_m_ = this->get_parameter("max_valid_z_m").as_double();
        
        // 设置QoS，某些情况下
        //rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 雷达数据发布
        lidar_pub = this->create_publisher<ros2_tools::msg::LidarPose>("lidar_data", 10);
        RCLCPP_INFO(this->get_logger(), "创建发布 lidar_data");

        selected_odom_topic_ = using_gazebo_ ? sim_topic : real_topic;
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            selected_odom_topic_, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg){ LidarDataNode::odomCallback(msg);});
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { logOdomWatchdog(); });
        RCLCPP_INFO(
            this->get_logger(),
            "创建%sodom订阅: %s",
            using_gazebo_ ? "仿真" : "实机",
            selected_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "雷达前向偏置补偿: %.3f m", lidar_forward_offset_m_);
        RCLCPP_INFO(this->get_logger(), "启动原点重置: %s", reset_origin_on_start_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "地面车Z漂移保护: max_valid_z_m=%.2f", max_valid_z_m_);
    }

    // 兼容版odom回调
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        received_odom_ = true;
        last_odom_time_ = this->get_clock()->now();
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
        if (reset_origin_on_start_ && std::abs(out_z) > max_valid_z_m_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "丢弃异常Point-LIO位姿: z=%.2f 超过 %.2f，raw=(%.2f, %.2f, %.2f)",
                out_z,
                max_valid_z_m_,
                x,
                y,
                z);
            return;
        }

        // 填充并发布车体中心LidarPose消息
        lidar_pose.x = out_x;
        lidar_pose.y = out_y;
        lidar_pose.z = out_z;
        lidar_pose.roll = roll;
        lidar_pose.pitch = pitch;
        lidar_pose.yaw = yaw;

        lidar_pub->publish(lidar_pose);
        published_pose_ = true;
        last_publish_time_ = this->get_clock()->now();

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

    void logOdomWatchdog() {
        const rclcpp::Time now = this->get_clock()->now();
        if (!received_odom_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "等待Point-LIO里程计: 未收到 %s",
                selected_odom_topic_.c_str());
            return;
        }

        const double odom_age = (now - last_odom_time_).seconds();
        if (odom_age > 1.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "Point-LIO里程计断流: %.1fs 未收到 %s",
                odom_age,
                selected_odom_topic_.c_str());
            return;
        }

        const double publish_age = published_pose_ ? (now - last_publish_time_).seconds() : 1e9;
        if (publish_age > 1.0) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "有效小车定位断流: %.1fs 未发布 /car/lidar_data，通常是Point-LIO姿态被Z漂移保护过滤",
                publish_age);
        }
    }

private:
    bool using_gazebo_; // 仿真开关
    bool received_odom_{false};
    bool published_pose_{false};
    double lidar_forward_offset_m_;
    double max_valid_z_m_;
    bool reset_origin_on_start_;
    bool origin_ready_ = false;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    double origin_z_ = 0.0;
    std::string selected_odom_topic_;
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
    
    rclcpp::Publisher<ros2_tools::msg::LidarPose>::SharedPtr lidar_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

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
