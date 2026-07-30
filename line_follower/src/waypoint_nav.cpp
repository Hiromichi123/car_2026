/*
 * waypoint_nav.cpp — 航点导航节点 (C++ 版)
 *
 * 功能:
 *   - 订阅 VIO 定位 (/camera/camera/vio_100hz)
 *   - 以上电后首次定位为原点，航点设在初始朝向正前方 1m
 *   - PID 控制器计算 /cmd_vel 驱动差速底盘
 *
 * 使用:
 *   ros2 launch line_follower navigate.launch.py   # 一键启动，自动开始导航
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cmath>
#include <algorithm>

// ============================================================
// 控制参数
// ============================================================
constexpr double WAYPOINT_DIST    = 1.0;   // 航点距离: 正前方 1m
constexpr double KP_ANGULAR       = 0.8;   // 角度 P 增益
constexpr double KP_LINEAR        = 0.6;   // 速度比例
constexpr double MAX_LINEAR_SPEED = 0.4;   // 最大前进速度 m/s
constexpr double MAX_ANGULAR_SPEED= 0.4;   // 最大角速度 rad/s
constexpr double SLOW_DOWN_DIST   = 0.30;  // 接近航点减速距离 m
constexpr double REACH_DIST       = 0.15;  // 到达航点阈值 m
constexpr double LOST_TIMEOUT     = 1.0;   // 定位丢失超时 s


class WaypointNavigator : public rclcpp::Node {
public:
  WaypointNavigator()
  : Node("waypoint_navigator"),
    state_(State::WAITING_POSE),
    pose_received_(false),
    wp_x_(0.0), wp_y_(0.0),
    cmd_seq_(0)
  {
    // ---- 参数声明 ----
    this->declare_parameter("kp_angular", KP_ANGULAR);
    this->declare_parameter("kp_linear", KP_LINEAR);
    this->declare_parameter("max_linear_speed", MAX_LINEAR_SPEED);
    this->declare_parameter("max_angular_speed", MAX_ANGULAR_SPEED);
    this->declare_parameter("slow_down_dist", SLOW_DOWN_DIST);
    this->declare_parameter("reach_dist", REACH_DIST);
    this->declare_parameter("lost_timeout", LOST_TIMEOUT);

    kp_angular_        = this->get_parameter("kp_angular").as_double();
    kp_linear_         = this->get_parameter("kp_linear").as_double();
    max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    slow_down_dist_    = this->get_parameter("slow_down_dist").as_double();
    reach_dist_        = this->get_parameter("reach_dist").as_double();
    lost_timeout_      = this->get_parameter("lost_timeout").as_double();

    // ---- 发布 / 订阅 ----
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/camera/camera/vio_100hz", 10,
      std::bind(&WaypointNavigator::on_pose, this, std::placeholders::_1));

    // ---- 定时器 ----
    ctrl_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),   // 50Hz
      std::bind(&WaypointNavigator::control_loop, this));

    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&WaypointNavigator::print_status, this));

    watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&WaypointNavigator::watchdog, this));

    RCLCPP_INFO(this->get_logger(),
      "WaypointNavigator C++ 已启动\n"
      "  等待 VIO 定位，航点将设在初始朝向正前方 %.1f m", WAYPOINT_DIST);
  }

private:
  enum class State { WAITING_POSE, FOLLOWING, REACHED };

  // ==================== 回调 ====================

  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    double qx = msg->pose.orientation.x;
    double qy = msg->pose.orientation.y;
    double qz = msg->pose.orientation.z;
    double qw = msg->pose.orientation.w;

    // 相机 VIO 的 Z 轴 = 小车前进方向
    // body-Z 在世界 XY 平面的投影角度作为小车航向
    // R[0][2] = 2*(qx*qz + qw*qy)
    // R[1][2] = 2*(qy*qz - qw*qx)
    double rzx = 2.0 * (qx * qz + qw * qy);
    double rzy = 2.0 * (qy * qz - qw * qx);
    double yaw = std::atan2(rzy, rzx);

    // 首次收到定位 — 记录为原点，航点 = 车头正前方 WAYPOINT_DIST 米
    if (state_ == State::WAITING_POSE) {
      origin_x_ = x;
      origin_y_ = y;
      origin_yaw_ = yaw;
      wp_x_ = origin_x_ + WAYPOINT_DIST * std::cos(origin_yaw_);
      wp_y_ = origin_y_ + WAYPOINT_DIST * std::sin(origin_yaw_);
      state_ = State::FOLLOWING;
      RCLCPP_INFO(this->get_logger(),
        "VIO 就绪! 初始位姿 (%.2f, %.2f)\n"
        "  四元数: qx=%.4f qy=%.4f qz=%.4f qw=%.4f\n"
        "  yaw(heading)=%.0f° (Z轴投影)\n"
        "  航点: (%.2f, %.2f) — 正前方 %.1f m",
        origin_x_, origin_y_,
        qx, qy, qz, qw,
        origin_yaw_ * 180.0 / M_PI,
        wp_x_, wp_y_, WAYPOINT_DIST);
    }

    pose_x_ = x;
    pose_y_ = y;
    pose_yaw_ = yaw;
    pose_received_ = true;
    last_pose_time_ = this->now();
  }

  // ==================== 控制循环 50Hz ====================

  void control_loop() {
    if (state_ == State::WAITING_POSE) return;
    if (state_ == State::REACHED) return;
    if (!pose_received_) return;

    // 计算到航点的距离和方向
    double dx = wp_x_ - pose_x_;
    double dy = wp_y_ - pose_y_;
    double dist = std::hypot(dx, dy);

    double target_yaw = std::atan2(dy, dx);
    double yaw_error = normalize_angle(target_yaw - pose_yaw_);

    // ---- 到达判定 ----
    if (dist < reach_dist_) {
      RCLCPP_INFO(this->get_logger(),
        "=== 到达航点 (%.2f, %.2f)，距离 %.3f m，停车 ===",
        wp_x_, wp_y_, dist);
      publish_cmd(0.0, 0.0);
      state_ = State::REACHED;
      return;
    }

    // ---- PID 控制 ----
    double angular = kp_angular_ * yaw_error;
    angular = std::clamp(angular, -max_angular_speed_, max_angular_speed_);

    double linear = kp_linear_ * dist;
    if (dist < slow_down_dist_) {
      linear *= dist / slow_down_dist_;
    }
    double angle_factor = 1.0 - std::abs(yaw_error) / M_PI;
    linear *= std::max(0.3, angle_factor);
    linear = std::clamp(linear, 0.0, max_linear_speed_);

    publish_cmd(linear, angular);
  }

  void publish_cmd(double linear_x, double angular_z) {
    auto cmd = geometry_msgs::msg::Twist();
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;
    cmd_pub_->publish(cmd);
    cmd_seq_++;
  }

  // ==================== 看门狗 ====================

  void watchdog() {
    if (state_ != State::FOLLOWING) return;
    double elapsed = (this->now() - last_pose_time_).seconds();
    if (elapsed > lost_timeout_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "定位丢失 %.1fs，紧急停车！", elapsed);
      publish_cmd(0.0, 0.0);
    }
  }

  // ==================== 状态打印 ====================

  void print_status() {
    if (state_ != State::FOLLOWING) return;
    double dx = wp_x_ - pose_x_;
    double dy = wp_y_ - pose_y_;
    double dist = std::hypot(dx, dy);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "[FOLLOWING] 目标(%.2f,%.2f) 当前(%.2f,%.2f) 距离%.3fm yaw=%.0f°",
      wp_x_, wp_y_, pose_x_, pose_y_, dist,
      pose_yaw_ * 180.0 / M_PI);
  }

  // ==================== 工具函数 ====================

  static double normalize_angle(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  // ==================== 状态变量 ====================
  State state_;
  bool pose_received_;
  double pose_x_ = 0.0, pose_y_ = 0.0, pose_yaw_ = 0.0;
  double origin_x_ = 0.0, origin_y_ = 0.0, origin_yaw_ = 0.0;
  double wp_x_ = 0.0, wp_y_ = 0.0;
  rclcpp::Time last_pose_time_;
  int cmd_seq_;

  // 参数
  double kp_angular_, kp_linear_;
  double max_linear_speed_, max_angular_speed_;
  double slow_down_dist_, reach_dist_, lost_timeout_;

  // ROS 接口
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_, status_timer_, watchdog_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointNavigator>());
  rclcpp::shutdown();
  return 0;
}
