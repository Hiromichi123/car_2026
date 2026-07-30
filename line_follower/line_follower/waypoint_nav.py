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
import json
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from ros2_tools.msg import LidarPose


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
# 比赛场地航点: A(150,200) → B(150,350) → C(300,350)
#   → D(300,200) → A(150,200)，单位 cm。
# ============================================================

ARC_STEPS = 12  # 半圆中间点数

# 上半圆：B(150,350) → C(300,350)，右转 (顺时针, 曲率负)
_arc1_wps, _arc1_center, _arc1_sign = _semicircle_waypoints(
    start=(150, 350), end=(300, 350), radius=75, direction="right", steps=ARC_STEPS
)
# 下半圆：D(300,200) → A(150,200)，右转 (顺时针, 曲率负)
_arc2_wps, _arc2_center, _arc2_sign = _semicircle_waypoints(
    start=(300, 200), end=(150, 200), radius=75, direction="right", steps=ARC_STEPS
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
    _straight_wp(150, 200, 150, 200),
    # 1  B 左侧直线段，线段起点=A
    _straight_wp(150, 350, 150, 200),
    # 2-13  上半圆 R=75 (12个中间点)
    *_arc1_wps,
    # 14  C 半圆终点
    Waypoint(x=3.00, y=3.50, curvature=_arc1_wps[-1].curvature if _arc1_wps else 0.0,
             seg_type=SegType.ARC,
             arc_center_x=_arc1_center[0] / 100.0,
             arc_center_y=_arc1_center[1] / 100.0,
             arc_radius=0.75),
    # 15  D 右侧直线段，线段起点=C
    _straight_wp(300, 200, 300, 350),
    # 16-27 下半圆 R=75 (12个中间点)
    *_arc2_wps,
    # 28  回到A，停车
    Waypoint(x=1.50, y=2.00, curvature=_arc2_wps[-1].curvature if _arc2_wps else 0.0,
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
        self.declare_parameter("mission_command_topic", "/mission/command")
        self.declare_parameter("carrier_pose_topic", "/carrier/lidar_pose")
        self.declare_parameter("car_status_topic", "/car/status")
        self.declare_parameter("field_start_x", 1.50)
        self.declare_parameter("field_start_y", 2.00)
        self.declare_parameter("field_start_yaw", math.pi / 2.0)
        self.declare_parameter("field_position_yaw_offset", math.pi / 2.0)
        self.declare_parameter("field_position_scale_x", 1.0)
        self.declare_parameter("field_position_scale_y", 1.0)
        self.declare_parameter("auto_position_alignment", False)
        self.declare_parameter("reset_origin_on_start", True)
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
        self.declare_parameter("allow_waypoint_overshoot_reach", True)

        # 读取参数
        self.pose_topic = self.get_parameter("pose_topic").value
        self.mission_command_topic = self.get_parameter("mission_command_topic").value
        self.carrier_pose_topic = self.get_parameter("carrier_pose_topic").value
        self.car_status_topic = self.get_parameter("car_status_topic").value
        self.field_start_x = self.get_parameter("field_start_x").value
        self.field_start_y = self.get_parameter("field_start_y").value
        self.field_start_yaw = self.get_parameter("field_start_yaw").value
        self.field_position_yaw_offset = self.get_parameter("field_position_yaw_offset").value
        self.field_position_scale_x = self.get_parameter("field_position_scale_x").value
        self.field_position_scale_y = self.get_parameter("field_position_scale_y").value
        self.auto_position_alignment = self.get_parameter("auto_position_alignment").value
        self.reset_origin_on_start = self.get_parameter("reset_origin_on_start").value
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
        self.allow_waypoint_overshoot_reach = self.get_parameter("allow_waypoint_overshoot_reach").value

        # 航点
        self.waypoints: List[Waypoint] = DEFAULT_WAYPOINTS

        # ---- 状态 ----
        self._state = "IDLE"
        self._wp_index = 0
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._pose_yaw = 0.0
        self._pose_received = False
        self._local_origin_ready = False
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._origin_yaw = 0.0
        self._heading_yaw_delta = 0.0
        self._position_yaw_delta = self.field_position_yaw_offset
        self._raw_x = 0.0
        self._raw_y = 0.0
        self._raw_yaw = 0.0
        self._raw_dx = 0.0
        self._raw_dy = 0.0
        self._last_pose_time = self.get_clock().now()
        self._cmd_seq = 0
        self._task_id = 0

        # ---- 发布/订阅 ----
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(Bool, "/nav/started", 10)
        self.carrier_pose_pub = self.create_publisher(LidarPose, self.carrier_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.car_status_topic, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self._on_pose, 10)

        # ---- 定时器 ----
        self.ctrl_timer = self.create_timer(0.02, self._control_loop)
        self.status_timer = self.create_timer(0.5, self._print_status)
        self.watchdog_timer = self.create_timer(0.2, self._watchdog)
        self.pose_publish_timer = self.create_timer(0.05, self._publish_carrier_pose)

        self.get_logger().info(
            f"WaypointNavigator v2 已启动\n"
            f"  定位话题: {self.pose_topic}\n"
            f"  任务命令: {self.mission_command_topic}\n"
            f"  航母位姿: {self.carrier_pose_topic}\n"
            f"  航点数: {len(self.waypoints)}\n"
            f"  弧段中间点: {ARC_STEPS}/半圆\n"
            f"  前馈增益: {self.ff_gain}, 横向误差增益: {self.kp_crosstrack}\n"
            f"  等待启动指令..."
        )
        self.get_logger().info(
            "  发送: ros2 topic pub /mission/command std_msgs/msg/String '{data: \"{\\\"command\\\":\\\"start\\\",\\\"task\\\":1}\"}' -1"
        )

        self.start_sub = self.create_subscription(
            String, self.mission_command_topic, self._on_mission_command, 10)

    # ==================== 回调 ====================

    def _on_pose(self, msg: PoseStamped) -> None:
        raw_x = msg.pose.position.x
        raw_y = msg.pose.position.y

        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny, cosy)
        self._raw_x = raw_x
        self._raw_y = raw_y
        self._raw_yaw = raw_yaw

        if not self._local_origin_ready:
            self._reset_field_origin(raw_x, raw_y, raw_yaw, "initial pose")

        dx = raw_x - self._origin_x
        dy = raw_y - self._origin_y
        self._raw_dx = dx
        self._raw_dy = dy
        cos_d = math.cos(self._position_yaw_delta)
        sin_d = math.sin(self._position_yaw_delta)
        mapped_x = cos_d * dx - sin_d * dy
        mapped_y = sin_d * dx + cos_d * dy
        self._pose_x = self.field_start_x + self.field_position_scale_x * mapped_x
        self._pose_y = self.field_start_y + self.field_position_scale_y * mapped_y
        self._pose_yaw = self._normalize_angle(raw_yaw + self._heading_yaw_delta)

        self._pose_received = True
        self._last_pose_time = self.get_clock().now()

    def _on_mission_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"忽略非法任务命令: {msg.data}")
            return

        command = payload.get("command")
        if command in ("reset_origin", "set_origin"):
            if self._pose_received:
                self._reset_field_origin(self._raw_x, self._raw_y, self._raw_yaw, command)
                self._publish_status("IDLE", "origin reset at current pose")
            return

        if command == "start" and self._state in ("IDLE", "STOPPED"):
            self._task_id = int(payload.get("task", 1))
            if self.reset_origin_on_start and self._pose_received:
                self._reset_field_origin(self._raw_x, self._raw_y, self._raw_yaw, "mission start")
            self.get_logger().info(f"=== 收到任务{self._task_id}启动指令，开始循线一圈 ===")
            self._state = "FOLLOWING"
            self._wp_index = 0
            self.state_pub.publish(Bool(data=True))
            self._publish_status("RUNNING", "mission command accepted")

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

        reached_by_distance = dist < reach_thresh
        reached_by_overshoot = self._has_passed_straight_waypoint(wp, reach_thresh)
        if reached_by_distance or reached_by_overshoot:
            if is_final:
                self._on_lap_complete()
                return
            else:
                self._wp_index += 1
                next_wp = self.waypoints[self._wp_index]
                reach_reason = "距离阈值" if reached_by_distance else "越过目标投影"
                self.get_logger().info(
                    f"到达航点 {self._wp_index - 1}/{len(self.waypoints) - 1} ({reach_reason}), "
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

    def _has_passed_straight_waypoint(self, wp: Waypoint, reach_thresh: float) -> bool:
        if not self.allow_waypoint_overshoot_reach or wp.seg_type != SegType.STRAIGHT:
            return False

        seg_dx = wp.x - wp.line_start_x
        seg_dy = wp.y - wp.line_start_y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.001:
            return False

        ux, uy = seg_dx / seg_len, seg_dy / seg_len
        rx = self._pose_x - wp.line_start_x
        ry = self._pose_y - wp.line_start_y
        along_track = ux * rx + uy * ry
        cross_track = abs(ux * ry - uy * rx)

        return along_track >= seg_len and cross_track <= max(0.30, 3.0 * reach_thresh)

    # ==================== 状态机 ====================

    def _on_lap_complete(self) -> None:
        self.get_logger().info("=== 到达终点A，一圈完成，停车 ===")
        self._publish_cmd(0.0, 0.0)
        self._state = "STOPPED"
        self.state_pub.publish(Bool(data=False))
        self._publish_status("FINISH", "one lap complete at A")

    def _watchdog(self) -> None:
        if self._state != "FOLLOWING":
            return
        elapsed = (self.get_clock().now() - self._last_pose_time).nanoseconds * 1e-9
        if elapsed > self.lost_timeout:
            self.get_logger().error(
                f"定位丢失 {elapsed:.1f}s，紧急停车！", throttle_duration_sec=2.0)
            self._publish_cmd(0.0, 0.0)

    def _print_status(self) -> None:
        self._publish_status(self._state, "periodic status")
        if self._state == "FOLLOWING" and self._wp_index < len(self.waypoints):
            wp = self.waypoints[self._wp_index]
            dx = wp.x - self._pose_x
            dy = wp.y - self._pose_y
            dist = math.hypot(dx, dy)
            cte = self._compute_crosstrack_error(wp)
            along_track, cross_track = self._straight_progress(wp)
            seg_name = "ARC" if wp.curvature != 0 else "LINE"
            self.get_logger().info(
                f"[{self._state}] #{self._wp_index}/{len(self.waypoints)-1} "
                f"目标({wp.x:.2f},{wp.y:.2f}) {seg_name} "
                f"pos=({self._pose_x:.2f},{self._pose_y:.2f}) raw_d=({self._raw_dx:.2f},{self._raw_dy:.2f}) "
                f"距离{dist:.2f}m CTE={cte:+.03f}m along={along_track:.2f} cross={cross_track:+.02f} "
                f"yaw={math.degrees(self._pose_yaw):.0f}°",
                throttle_duration_sec=2.0,
            )

    # ==================== 工具函数 ====================

    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self._cmd_seq += 1

    def _publish_carrier_pose(self) -> None:
        if not self._pose_received:
            return
        msg = LidarPose()
        msg.x = float(self._pose_x)
        msg.y = float(self._pose_y)
        msg.z = 0.0
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = float(self._pose_yaw)
        self.carrier_pose_pub.publish(msg)

    def _publish_status(self, state: str, detail: str) -> None:
        msg = String()
        wp_x = 0.0
        wp_y = 0.0
        dist = 0.0
        along_track = 0.0
        cross_track = 0.0
        if self._wp_index < len(self.waypoints):
            wp = self.waypoints[self._wp_index]
            wp_x = wp.x
            wp_y = wp.y
            dist = math.hypot(wp.x - self._pose_x, wp.y - self._pose_y)
            along_track, cross_track = self._straight_progress(wp)
        msg.data = json.dumps({
            "role": "car",
            "task": self._task_id,
            "state": state,
            "detail": detail,
            "wp_index": self._wp_index,
            "wp_count": len(self.waypoints),
            "x": round(self._pose_x, 3),
            "y": round(self._pose_y, 3),
            "yaw": round(self._pose_yaw, 3),
            "target_x": round(wp_x, 3),
            "target_y": round(wp_y, 3),
            "target_dist": round(dist, 3),
            "along_track": round(along_track, 3),
            "cross_track": round(cross_track, 3),
            "raw_x": round(self._raw_x, 3),
            "raw_y": round(self._raw_y, 3),
            "raw_dx": round(self._raw_dx, 3),
            "raw_dy": round(self._raw_dy, 3),
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _reset_field_origin(self, raw_x: float, raw_y: float, raw_yaw: float, reason: str) -> None:
        self._origin_x = raw_x
        self._origin_y = raw_y
        self._origin_yaw = raw_yaw
        self._heading_yaw_delta = self.field_start_yaw - self._origin_yaw
        if self.auto_position_alignment:
            self._position_yaw_delta = self._heading_yaw_delta
        else:
            self._position_yaw_delta = self.field_position_yaw_offset
        self._raw_dx = 0.0
        self._raw_dy = 0.0
        self._local_origin_ready = True
        self.get_logger().info(
            f"场地坐标对齐({reason}): VIO origin=({raw_x:.2f},{raw_y:.2f}) "
            f"yaw={math.degrees(raw_yaw):.1f}° -> A=({self.field_start_x:.2f},{self.field_start_y:.2f}), "
            f"pos_offset={math.degrees(self._position_yaw_delta):.1f}°, "
            f"heading_offset={math.degrees(self._heading_yaw_delta):.1f}°, "
            f"scale=({self.field_position_scale_x:.2f},{self.field_position_scale_y:.2f})"
        )

    def _straight_progress(self, wp: Waypoint) -> tuple[float, float]:
        if wp.seg_type != SegType.STRAIGHT:
            return 0.0, 0.0

        seg_dx = wp.x - wp.line_start_x
        seg_dy = wp.y - wp.line_start_y
        seg_len = math.hypot(seg_dx, seg_dy)
        if seg_len < 0.001:
            return 0.0, 0.0

        ux, uy = seg_dx / seg_len, seg_dy / seg_len
        rx = self._pose_x - wp.line_start_x
        ry = self._pose_y - wp.line_start_y
        along_track = ux * rx + uy * ry
        cross_track = ux * ry - uy * rx
        return along_track, cross_track

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
