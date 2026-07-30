#!/usr/bin/env python3
"""
航点导航节点：基于VIO定位，按预设航点闭环巡线一圈。

订阅:
  /camera/camera/vio_100hz (PoseStamped) — VIO定位, 100Hz
发布:
  /cmd_vel (Twist)                      — 差速底盘速度指令

流程:
  IDLE → 收到启动信号 → FOLLOWING(逐个航点) → 回到A → STOPPED

坐标系说明:
  - 小车在A点上电，VIO以当前位姿为原点 → A = (0, 0)
  - X 轴：场地横向 (0~400 cm)，正方向向右
  - Y 轴：场地纵向 (0~500 cm)，正方向向前（小车前进方向）
  - 航点坐标即场地绝对坐标，单位 cm，内部转换为 m 使用

半圆追踪优化 (v2):
  - 曲率前馈: 弧段预加 ω_ff = v / R，消除弯道稳态滞后
  - 横向误差修正: 直道计算到线段的垂直距离，弯道计算到圆弧的径向误差
  - 弧点加密: 12个中间点，更平滑
"""
import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


# ============================================================
# 航点数据结构
# ============================================================

class SegType(Enum):
    STRAIGHT = "straight"   # 直线段
    ARC = "arc"             # 圆弧段


@dataclass
class Waypoint:
    """航点，含曲率和线段几何信息用于前馈+横向误差修正"""
    x: float                          # 坐标 (m)
    y: float                          # 坐标 (m)
    curvature: float = 0.0            # 曲率 1/R (m⁻¹), 正=左转, 负=右转, 0=直线
    seg_type: SegType = SegType.STRAIGHT
    # 直线段用 (线段起点，用于计算横向误差)
    line_start_x: float = 0.0
    line_start_y: float = 0.0
    # 圆弧段用
    arc_center_x: float = 0.0
    arc_center_y: float = 0.0
    arc_radius: float = 0.0


# ============================================================
# 半圆航点生成
# ============================================================

def _semicircle_waypoints(
    start: Tuple[float, float],
    end: Tuple[float, float],
    radius: float,
    direction: str = "right",
    steps: int = 12,
) -> Tuple[List[Waypoint], Tuple[float, float], float]:
    """
    生成半圆弧的中间航点（含曲率前馈和圆弧元信息）。

    start, end: 半圆首尾坐标 (cm)
    radius: 圆弧半径 (cm)
    direction: "right"=顺时针(右转,曲率为负), "left"=逆时针(左转,曲率为正)
    steps: 中间点数(不含首尾)

    Returns:
      (waypoints, arc_center_xy_cm, dir_sign)
    """
    sx, sy = start
    ex, ey = end
    mx, my = (sx + ex) / 2.0, (sy + ey) / 2.0
    dx, dy = ex - sx, ey - sy
    chord = math.hypot(dx, dy)
    if chord > 2 * radius:
        raise ValueError(f"弦长{chord:.1f} > 直径{2*radius:.1f}，无法构成半圆")

    half_chord = chord / 2.0
    dist_to_center = math.sqrt(max(0, radius**2 - half_chord**2))

    # 弦法向量（逆时针旋转90°=左, -90°=右）
    nx, ny = -dy / chord, dx / chord
    if direction == "right":
        nx, ny = -nx, -ny
    cx, cy = mx + nx * dist_to_center, my + ny * dist_to_center

    a0 = math.atan2(sy - cy, sx - cx)
    a1 = math.atan2(ey - cy, ex - cx)

    if direction == "right":
        if a1 > a0:
            a1 -= 2.0 * math.pi
    else:
        if a1 < a0:
            a1 += 2.0 * math.pi

    # 曲率: 1/R (m⁻¹), 右转为负, 左转为正
    dir_sign = 1.0 if direction == "left" else -1.0
    curvature = dir_sign / (radius / 100.0)

    points = []
    for i in range(1, steps + 1):
        t = i / (steps + 1)
        a = a0 + (a1 - a0) * t
        wx = cx + radius * math.cos(a)
        wy = cy + radius * math.sin(a)
        points.append(Waypoint(
            x=wx / 100.0, y=wy / 100.0,
            curvature=curvature,
            seg_type=SegType.ARC,
            arc_center_x=cx / 100.0,
            arc_center_y=cy / 100.0,
            arc_radius=radius / 100.0,
        ))

    return points, (cx, cy), dir_sign


# ============================================================
# D题地图航点: 跑道形闭环
#   A(0,0) → B(150,0) 直行
#   → 右半圆 R=75 → C(150,-150)
#   → D(0,-150) 直行
#   → 左半圆 R=75 → 回到 A(0,0)，停车
# ============================================================

ARC_STEPS = 12  # 半圆中间点数

# 第一个半圆：B(150,0) → C(150,-150)，右转 (顺时针, 曲率负)
_arc1_wps, _arc1_center, _arc1_sign = _semicircle_waypoints(
    start=(150, 0), end=(150, -150), radius=75, direction="right", steps=ARC_STEPS
)
# 第二个半圆：D(0,-150) → A(0,0)，左转 (逆时针, 曲率正)
_arc2_wps, _arc2_center, _arc2_sign = _semicircle_waypoints(
    start=(0, -150), end=(0, 0), radius=75, direction="left", steps=ARC_STEPS
)

# 直线段航点 helper
def _straight_wp(x_cm: float, y_cm: float,
                 line_start_x_cm: float = 0.0,
                 line_start_y_cm: float = 0.0) -> Waypoint:
    return Waypoint(
        x=x_cm / 100.0, y=y_cm / 100.0,
        curvature=0.0,
        seg_type=SegType.STRAIGHT,
        line_start_x=line_start_x_cm / 100.0,
        line_start_y=line_start_y_cm / 100.0,
    )


DEFAULT_WAYPOINTS: List[Waypoint] = [
    # 0  A 起点
    _straight_wp(0, 0, 0, 0),
    # 1  B 直行150cm，线段起点=A
    _straight_wp(150, 0, 0, 0),
    # 2-13  右转半圆 R=75 (12个中间点)
    *_arc1_wps,
    # 14  C 半圆终点 (到C后完成弧段)
    Waypoint(x=1.50, y=-1.50, curvature=_arc1_wps[-1].curvature if _arc1_wps else 0.0,
             seg_type=SegType.ARC,
             arc_center_x=_arc1_center[0] / 100.0,
             arc_center_y=_arc1_center[1] / 100.0,
             arc_radius=0.75),
    # 15  D 直行(右→左)，线段起点=C
    _straight_wp(0, -150, 150, -150),
    # 16-27 左转半圆 R=75 (12个中间点)
    *_arc2_wps,
    # 28  回到A，停车
    Waypoint(x=0.0, y=0.0, curvature=_arc2_wps[-1].curvature if _arc2_wps else 0.0,
             seg_type=SegType.ARC,
             arc_center_x=_arc2_center[0] / 100.0,
             arc_center_y=_arc2_center[1] / 100.0,
             arc_radius=0.75),
]


# ============================================================
# 导航器
# ============================================================

class WaypointNavigator(Node):
    """航点导航器：读定位 → 算控制量(前馈+横向误差修正+PID) → 发 cmd_vel"""

    def __init__(self) -> None:
        super().__init__("waypoint_navigator")

        # ---- 参数 ----
        self.declare_parameter("pose_topic", "/camera/camera/vio_100hz")
        self.declare_parameter("waypoint_reach_dist", 8.0)     # 到达航点距离阈值 cm
        self.declare_parameter("final_reach_dist", 5.0)        # 回到A的距离阈值 cm
        self.declare_parameter("kp_angular", 1.8)              # 航向P增益
        self.declare_parameter("kp_crosstrack", 1.2)           # 横向误差P增益
        self.declare_parameter("kp_linear", 0.6)               # 速度比例
        self.declare_parameter("max_linear_speed", 0.4)        # 最大前进速度 m/s
        self.declare_parameter("max_angular_speed", 0.8)       # 最大角速度 rad/s
        self.declare_parameter("slow_down_dist", 30.0)         # 接近航点时减速距离 cm
        self.declare_parameter("lost_timeout", 1.0)            # 定位丢失超时 s
        self.declare_parameter("feedforward_gain", 0.7)        # 曲率前馈增益 (0~1)

        # 读取参数
        self.pose_topic = self.get_parameter("pose_topic").value
        self.waypoint_reach_dist = self.get_parameter("waypoint_reach_dist").value
        self.final_reach_dist = self.get_parameter("final_reach_dist").value
        self.kp_angular = self.get_parameter("kp_angular").value
        self.kp_crosstrack = self.get_parameter("kp_crosstrack").value
        self.kp_linear = self.get_parameter("kp_linear").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.slow_down_dist = self.get_parameter("slow_down_dist").value
        self.lost_timeout = self.get_parameter("lost_timeout").value
        self.ff_gain = self.get_parameter("feedforward_gain").value

        # 航点
        self.waypoints: List[Waypoint] = DEFAULT_WAYPOINTS

        # ---- 状态 ----
        self._state = "IDLE"
        self._wp_index = 0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._pose_received = False
        self._last_pose_time = self.get_clock().now()
        self._cmd_seq = 0

        # ---- 发布/订阅 ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(Bool, "/nav/started", 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self._on_pose, 10)

        # ---- 定时器 ----
        self.ctrl_timer = self.create_timer(0.02, self._control_loop)
        self.status_timer = self.create_timer(1.0, self._print_status)
        self.watchdog_timer = self.create_timer(0.2, self._watchdog)

        self.get_logger().info(
            f"WaypointNavigator v2 已启动\n"
            f"  定位话题: {self.pose_topic}\n"
            f"  航点数: {len(self.waypoints)}\n"
            f"  弧段中间点: {ARC_STEPS}/半圆\n"
            f"  前馈增益: {self.ff_gain}, 横向误差增益: {self.kp_crosstrack}\n"
            f"  等待启动指令..."
        )
        self.get_logger().info(
            "  发送: ros2 topic pub /nav/start std_msgs/msg/Bool '{data: true}' -1"
        )

        self.start_sub = self.create_subscription(
            Bool, "/nav/start", self._on_start, 10)

    # ==================== 回调 ====================

    def _on_pose(self, msg: PoseStamped) -> None:
        self._pose_x = msg.pose.position.x
        self._pose_y = msg.pose.position.y

        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._pose_yaw = math.atan2(siny, cosy)

        self._pose_received = True
        self._last_pose_time = self.get_clock().now()

    def _on_start(self, msg: Bool) -> None:
        if msg.data and self._state == "IDLE":
            self.get_logger().info("=== 收到启动指令，开始巡线 ===")
            self._state = "FOLLOWING"
            self._wp_index = 0
            self.state_pub.publish(Bool(data=True))

    # ==================== 控制循环 50Hz ====================

    def _control_loop(self) -> None:
        if self._state != "FOLLOWING":
            if self._state == "IDLE":
                self._publish_cmd(0.0, 0.0)
            return

        if not self._pose_received:
            return

        if self._wp_index >= len(self.waypoints):
            self._on_lap_complete()
            return

        wp = self.waypoints[self._wp_index]

        # 距离
        dx = wp.x - self._pose_x
        dy = wp.y - self._pose_y
        dist = math.hypot(dx, dy)

        # ---- 到达航点判断 ----
        is_final = (self._wp_index == len(self.waypoints) - 1)
        reach_thresh = (
            self.final_reach_dist / 100.0 if is_final
            else self.waypoint_reach_dist / 100.0
        )

        if dist < reach_thresh:
            if is_final:
                self._on_lap_complete()
                return
            else:
                self._wp_index += 1
                next_wp = self.waypoints[self._wp_index]
                self.get_logger().info(
                    f"到达航点 {self._wp_index - 1}/{len(self.waypoints) - 1}, "
                    f"下一目标: #{self._wp_index} ({next_wp.x:.2f}, {next_wp.y:.2f})"
                    + (f" 曲率={next_wp.curvature:.2f}" if next_wp.curvature != 0 else "")
                )
                return

        # ---- 计算控制量 ----

        # 1) 航向误差
        target_yaw = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(target_yaw - self._pose_yaw)

        # 2) 横向误差 (cross-track error)
        cte = self._compute_crosstrack_error(wp)

        # 3) 前进速度
        linear = self.kp_linear * dist
        if dist < self.slow_down_dist / 100.0:
            linear *= dist / (self.slow_down_dist / 100.0)
        angle_factor = 1.0 - abs(yaw_error) / math.pi
        linear *= max(0.3, angle_factor)
        linear = max(0.0, min(self.max_linear_speed, linear))

        # 4) 角速度 = 曲率前馈 + 航向修正 + 横向误差修正
        #    前馈: 已知圆弧曲率 → ω = v / R
        angular_ff = self.ff_gain * linear * wp.curvature
        #    航向: P控制
        angular_heading = self.kp_angular * yaw_error
        #    横向: P控制 (cte符号: 正=在路径右侧, 负=在路径左侧)
        angular_cte = self.kp_crosstrack * cte

        angular = angular_ff + angular_heading + angular_cte
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))

        self._publish_cmd(linear, angular)

    # ==================== 横向误差计算 ====================

    def _compute_crosstrack_error(self, wp: Waypoint) -> float:
        """
        计算当前位置到目标路径的横向误差 (有符号距离, m)。

        直线段: 到线段 (line_start → wp) 的垂直有符号距离
                正 = 在线段右侧, 负 = 在线段左侧
        圆弧段: 到圆心的距离 - 半径
                正 = 在圆外, 负 = 在圆内
        """
        if wp.seg_type == SegType.ARC:
            # 圆弧段: 径向误差
            dcx = self._pose_x - wp.arc_center_x
            dcy = self._pose_y - wp.arc_center_y
            dist_to_center = math.hypot(dcx, dcy)
            return dist_to_center - wp.arc_radius
        else:
            # 直线段: 到线段的有符号距离
            # 线段方向
            seg_dx = wp.x - wp.line_start_x
            seg_dy = wp.y - wp.line_start_y
            seg_len = math.hypot(seg_dx, seg_dy)
            if seg_len < 0.001:
                return 0.0
            # 单位方向
            ux, uy = seg_dx / seg_len, seg_dy / seg_len
            # 当前位置相对线段起点的向量
            rx = self._pose_x - wp.line_start_x
            ry = self._pose_y - wp.line_start_y
            # 2D 叉积: u × r = ux*ry - uy*rx
            # 正 = 点在方向向量右侧, 负 = 点在左侧
            return ux * ry - uy * rx

    # ==================== 状态机 ====================

    def _on_lap_complete(self) -> None:
        self.get_logger().info("=== 到达终点A，一圈完成，停车 ===")
        self._publish_cmd(0.0, 0.0)
        self._state = "STOPPED"
        self.state_pub.publish(Bool(data=False))

    def _watchdog(self) -> None:
        if self._state != "FOLLOWING":
            return
        elapsed = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
        if elapsed > self.lost_timeout:
            self.get_logger().error(
                f"定位丢失 {elapsed:.1f}s，紧急停车！", throttle_duration_sec=2.0)
            self._publish_cmd(0.0, 0.0)

    def _print_status(self) -> None:
        if self._state == "FOLLOWING" and self._wp_index < len(self.waypoints):
            wp = self.waypoints[self._wp_index]
            dx = wp.x - self._pose_x
            dy = wp.y - self._pose_y
            dist = math.hypot(dx, dy)
            cte = self._compute_crosstrack_error(wp)
            seg_name = "ARC" if wp.curvature != 0 else "LINE"
            self.get_logger().info(
                f"[{self._state}] #{self._wp_index}/{len(self.waypoints)-1} "
                f"目标({wp.x:.2f},{wp.y:.2f}) {seg_name} "
                f"距离{dist:.2f}m CTE={cte:+.03f}m yaw={math.degrees(self._pose_yaw):.0f}°",
                throttle_duration_sec=2.0,
            )

    # ==================== 工具函数 ====================

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self._cmd_seq += 1

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
