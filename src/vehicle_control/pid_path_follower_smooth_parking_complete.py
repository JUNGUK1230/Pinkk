#!/usr/bin/env python3

import math
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ControlState(Enum):
    WAIT_ODOM = auto()
    SMOOTH_DRIVE = auto()
    ROTATE = auto()
    DRIVE = auto()
    FINISHED = auto()


class PID:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float,
        integral_limit: float,
        derivative_filter: float = 0.75,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.derivative_filter = clamp(derivative_filter, 0.0, 1.0)

        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = 0.0
        self.filtered_derivative = 0.0
        self.initialized = False

    def update(self, error: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        if not self.initialized:
            derivative = 0.0
            self.previous_error = error
            self.initialized = True
        else:
            derivative = (error - self.previous_error) / dt

        self.filtered_derivative = (
            self.derivative_filter * self.filtered_derivative
            + (1.0 - self.derivative_filter) * derivative
        )

        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * self.filtered_derivative
        )

        self.previous_error = error

        return clamp(
            output,
            -self.output_limit,
            self.output_limit,
        )


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointPIDController(Node):
    """
    동작 순서

    WP1→WP8:
      일반 코스부터 주차 완료점까지 멈추거나 제자리 회전하지 않고
      전진 속도와 회전 속도를 동시에 출력해 연속 곡선주행한다.

    WP7→WP8:
      주차 진입 구간은 속도를 낮추고, 주차 방향을 미리 바라보면서
      곡선으로 진입한다. WP8에 가까워질수록 차체를 주차선 방향에 맞춘다.

    WP8→WP9:
      차체 방향을 유지한 채 직선 후진하여 출차한다.

    WP9→WP10:
      출구 방향으로 회전한 뒤 직진하고 종료한다.
    """

    def __init__(self) -> None:
        super().__init__('waypoint_pid_controller')

        # ============================================================
        # ROS 파라미터
        # ============================================================

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('coordinate_scale', 1.0)

        # 일반 정밀주행
        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('min_linear_speed', 0.0375)
        self.declare_parameter('max_angular_speed', 1.20)
        self.declare_parameter('min_angular_speed', 0.15)

        self.declare_parameter('position_tolerance', 0.018)
        self.declare_parameter('rotation_tolerance_deg', 3.0)
        self.declare_parameter('drive_abort_angle_deg', 25.0)
        self.declare_parameter('slowdown_angle_deg', 12.0)
        self.declare_parameter('linear_slowdown_distance', 0.12)
        self.declare_parameter('waypoint_pause_sec', 0.13)

        self.declare_parameter('linear_kp', 0.90)
        self.declare_parameter('linear_ki', 0.00)
        self.declare_parameter('linear_kd', 0.05)

        self.declare_parameter('angular_kp', 2.40)
        self.declare_parameter('angular_ki', 0.00)
        self.declare_parameter('angular_kd', 0.10)

        self.declare_parameter('heading_kp', 2.00)
        self.declare_parameter('heading_ki', 0.00)
        self.declare_parameter('heading_kd', 0.07)

        # 곡선주행
        self.declare_parameter('smooth_max_linear_speed', 0.15)
        self.declare_parameter('smooth_min_linear_speed', 0.0375)
        self.declare_parameter('smooth_max_angular_speed', 0.63)
        self.declare_parameter('smooth_heading_kp', 1.35)
        self.declare_parameter('smooth_heading_kd', 0.035)
        self.declare_parameter('smooth_switch_distance', 0.055)
        self.declare_parameter('smooth_corner_radius', 0.14)
        self.declare_parameter('smooth_preview_distance', 0.10)
        self.declare_parameter('smooth_end_waypoint_number', 8)

        # 곡선 주차 진입 전용
        self.declare_parameter('parking_curve_max_linear_speed', 0.070)
        self.declare_parameter('parking_curve_min_linear_speed', 0.028)
        self.declare_parameter('parking_curve_max_angular_speed', 0.52)
        self.declare_parameter('parking_curve_heading_kp', 1.55)
        self.declare_parameter('parking_curve_heading_kd', 0.025)
        self.declare_parameter('parking_curve_align_distance', 0.12)

        # 주차 전용
        # 주차 진입 직전 90도 정렬 전용
        # 최대 회전속도를 낮추고 D항을 제거해 오버슈트를 줄인다.
        self.declare_parameter('parking_max_rotate_speed', 0.21)
        self.declare_parameter('parking_min_rotate_speed', 0.042)
        self.declare_parameter('parking_rotate_kp', 0.58)
        self.declare_parameter('parking_rotate_kd', 0.00)

        self.declare_parameter('parking_heading_kp', 0.85)
        self.declare_parameter('parking_max_heading_speed', 0.15)
        self.declare_parameter('parking_forward_speed', 0.045)
        self.declare_parameter('parking_reverse_speed', 0.0525)
        self.declare_parameter('parking_pause_sec', 0.67)

        # ============================================================
        # 파라미터 읽기
        # ============================================================

        odom_topic = str(self.get_parameter('odom_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.control_frequency = float(
            self.get_parameter('control_frequency').value
        )
        self.coordinate_scale = float(
            self.get_parameter('coordinate_scale').value
        )

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.min_linear_speed = float(
            self.get_parameter('min_linear_speed').value
        )
        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )
        self.min_angular_speed = float(
            self.get_parameter('min_angular_speed').value
        )

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )
        self.rotation_tolerance = math.radians(
            float(self.get_parameter('rotation_tolerance_deg').value)
        )
        self.drive_abort_angle = math.radians(
            float(self.get_parameter('drive_abort_angle_deg').value)
        )
        self.slowdown_angle = math.radians(
            float(self.get_parameter('slowdown_angle_deg').value)
        )
        self.linear_slowdown_distance = float(
            self.get_parameter('linear_slowdown_distance').value
        )
        self.waypoint_pause_sec = float(
            self.get_parameter('waypoint_pause_sec').value
        )

        self.smooth_max_linear_speed = float(
            self.get_parameter('smooth_max_linear_speed').value
        )
        self.smooth_min_linear_speed = float(
            self.get_parameter('smooth_min_linear_speed').value
        )
        self.smooth_max_angular_speed = float(
            self.get_parameter('smooth_max_angular_speed').value
        )
        self.smooth_heading_kp = float(
            self.get_parameter('smooth_heading_kp').value
        )
        self.smooth_heading_kd = float(
            self.get_parameter('smooth_heading_kd').value
        )
        self.smooth_switch_distance = float(
            self.get_parameter('smooth_switch_distance').value
        )
        self.smooth_corner_radius = float(
            self.get_parameter('smooth_corner_radius').value
        )
        self.smooth_preview_distance = float(
            self.get_parameter('smooth_preview_distance').value
        )
        self.smooth_end_target_index = (
            int(
                self.get_parameter(
                    'smooth_end_waypoint_number'
                ).value
            )
            - 1
        )

        self.parking_curve_max_linear_speed = float(
            self.get_parameter('parking_curve_max_linear_speed').value
        )
        self.parking_curve_min_linear_speed = float(
            self.get_parameter('parking_curve_min_linear_speed').value
        )
        self.parking_curve_max_angular_speed = float(
            self.get_parameter('parking_curve_max_angular_speed').value
        )
        self.parking_curve_heading_kp = float(
            self.get_parameter('parking_curve_heading_kp').value
        )
        self.parking_curve_heading_kd = float(
            self.get_parameter('parking_curve_heading_kd').value
        )
        self.parking_curve_align_distance = float(
            self.get_parameter('parking_curve_align_distance').value
        )

        self.parking_max_rotate_speed = float(
            self.get_parameter('parking_max_rotate_speed').value
        )
        self.parking_min_rotate_speed = float(
            self.get_parameter('parking_min_rotate_speed').value
        )
        self.parking_rotate_kp = float(
            self.get_parameter('parking_rotate_kp').value
        )
        self.parking_rotate_kd = float(
            self.get_parameter('parking_rotate_kd').value
        )
        self.parking_heading_kp = float(
            self.get_parameter('parking_heading_kp').value
        )
        self.parking_max_heading_speed = float(
            self.get_parameter('parking_max_heading_speed').value
        )
        self.parking_forward_speed = float(
            self.get_parameter('parking_forward_speed').value
        )
        self.parking_reverse_speed = float(
            self.get_parameter('parking_reverse_speed').value
        )
        self.parking_pause_sec = float(
            self.get_parameter('parking_pause_sec').value
        )

        # ============================================================
        # 원본 지도 좌표
        # ============================================================

        self.map_waypoints: List[Tuple[float, float]] = [
            (1.63, 0.17),  # 1: 출발
            (0.61, 0.16),  # 2: 첫 번째 우회전 직전
            (0.61, 0.37),  # 3
            (0.52, 0.37),  # 4
            (0.54, 0.80),  # 5
            (0.80, 0.80),  # 6
            (0.99, 0.80),  # 7: 주차 진입 전
            (0.97, 0.56),  # 8: 전진 주차 완료
            (0.99, 0.83),  # 9: 후진 출차 완료
            (1.63, 0.82),  # 10: 출구
        ]

        # target_index 8 = WP9를 목표로 하는 구간
        # 즉 WP8→WP9에서만 후진
        self.reverse_target_indices = {8}

        self.waypoints: List[Tuple[float, float]] = []
        self.parking_yaw: Optional[float] = None

        # ============================================================
        # PID
        # ============================================================

        self.linear_pid = PID(
            kp=float(self.get_parameter('linear_kp').value),
            ki=float(self.get_parameter('linear_ki').value),
            kd=float(self.get_parameter('linear_kd').value),
            output_limit=self.max_linear_speed,
            integral_limit=0.20,
        )

        self.rotate_pid = PID(
            kp=float(self.get_parameter('angular_kp').value),
            ki=float(self.get_parameter('angular_ki').value),
            kd=float(self.get_parameter('angular_kd').value),
            output_limit=self.max_angular_speed,
            integral_limit=0.30,
        )

        self.heading_pid = PID(
            kp=float(self.get_parameter('heading_kp').value),
            ki=float(self.get_parameter('heading_ki').value),
            kd=float(self.get_parameter('heading_kd').value),
            output_limit=self.max_angular_speed,
            integral_limit=0.20,
        )

        # ============================================================
        # 현재 상태
        # ============================================================

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.odom_received = False
        self.route_initialized = False

        self.target_index = 1
        self.state = ControlState.WAIT_ODOM

        self.pause_until_ns = 0
        self.last_control_time_ns: Optional[int] = None

        self.smooth_previous_yaw_error = 0.0
        self.smooth_error_initialized = False

        self.parking_previous_yaw_error = 0.0
        self.parking_error_initialized = False

        # ============================================================
        # ROS Publisher / Subscriber / Timer
        # ============================================================

        odom_qos = QoSProfile(depth=20)
        odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            odom_qos,
        )

        timer_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            timer_period,
            self.control_loop,
        )

        self.get_logger().info(
            '10개 WP | WP8 주차 완료까지 연속 곡선주행 controller started'
        )
        self.get_logger().info(
            '로봇을 1번 위치에서 1→2 방향으로 놓고 실행하세요.'
        )

    # ================================================================
    # Odom
    # ================================================================

    def odom_callback(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.current_x = float(position.x)
        self.current_y = float(position.y)
        self.current_yaw = quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )

        self.odom_received = True

    # ================================================================
    # 지도 좌표 → Odom 좌표
    # ================================================================

    def initialize_route(self) -> None:
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw

        map_start_x, map_start_y = self.map_waypoints[0]

        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)

        self.waypoints.clear()

        for map_x, map_y in self.map_waypoints:
            local_forward = (
                map_start_x - map_x
            ) * self.coordinate_scale

            local_left = (
                map_start_y - map_y
            ) * self.coordinate_scale

            odom_x = (
                self.start_x
                + cos_yaw * local_forward
                - sin_yaw * local_left
            )
            odom_y = (
                self.start_y
                + sin_yaw * local_forward
                + cos_yaw * local_left
            )

            self.waypoints.append((odom_x, odom_y))

        # 주차 진입 각도를 정확히 90도로 고정한다.
        #
        # WP6→WP7의 진행방향을 기준으로,
        # WP7→WP8 주차점이 왼쪽에 있으면 +90도,
        # 오른쪽에 있으면 -90도를 적용한다.
        wp6_x, wp6_y = self.waypoints[5]
        wp7_x, wp7_y = self.waypoints[6]
        wp8_x, wp8_y = self.waypoints[7]

        incoming_dx = wp7_x - wp6_x
        incoming_dy = wp7_y - wp6_y

        parking_dx = wp8_x - wp7_x
        parking_dy = wp8_y - wp7_y

        incoming_yaw = math.atan2(
            incoming_dy,
            incoming_dx,
        )

        cross_product = (
            incoming_dx * parking_dy
            - incoming_dy * parking_dx
        )

        if cross_product >= 0.0:
            parking_turn_sign = 1.0
        else:
            parking_turn_sign = -1.0

        self.parking_yaw = normalize_angle(
            incoming_yaw
            + parking_turn_sign * math.pi / 2.0
        )

        self.route_initialized = True
        self.target_index = 1
        self.state = ControlState.SMOOTH_DRIVE

        self.reset_all_pid()

        self.get_logger().info(
            '========== 변환된 목표 좌표 =========='
        )
        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(
                f'WP{index + 1}: '
                f'x={waypoint[0]:.3f}, '
                f'y={waypoint[1]:.3f}'
            )
        self.get_logger().info(
            f'주차 고정 방향: '
            f'{math.degrees(self.parking_yaw):.1f} deg '
            f'(6→7 진행방향 기준 '
            f'{parking_turn_sign * 90.0:+.0f}도)'
        )
        self.get_logger().info(
            '======================================'
        )

    # ================================================================
    # 메인 루프
    # ================================================================

    def control_loop(self) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if self.last_control_time_ns is None:
            self.last_control_time_ns = now_ns
            return

        dt = (
            now_ns - self.last_control_time_ns
        ) / 1_000_000_000.0
        self.last_control_time_ns = now_ns
        dt = clamp(dt, 0.001, 0.10)

        if not self.odom_received:
            self.stop_robot()
            return

        if not self.route_initialized:
            self.initialize_route()
            return

        if self.state == ControlState.FINISHED:
            self.stop_robot()
            return

        if now_ns < self.pause_until_ns:
            self.stop_robot()
            return

        if self.target_index >= len(self.waypoints):
            self.finish_route()
            return

        target_x, target_y = self.waypoints[self.target_index]

        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance_error = math.hypot(dx, dy)

        is_reverse = (
            self.target_index in self.reverse_target_indices
        )

        travel_yaw = math.atan2(dy, dx)

        # 주차 진입(WP7→WP8)과 후진 출차(WP8→WP9)는
        # 같은 차체 방향을 유지한다.
        if (
            self.target_index in (7, 8)
            and self.parking_yaw is not None
        ):
            target_yaw = self.parking_yaw
        elif is_reverse:
            target_yaw = normalize_angle(
                travel_yaw + math.pi
            )
        else:
            target_yaw = travel_yaw

        yaw_error = normalize_angle(
            target_yaw - self.current_yaw
        )

        if self.state == ControlState.SMOOTH_DRIVE:
            self.smooth_drive_to_target(
                distance_error=distance_error,
                dt=dt,
            )

        elif self.state == ControlState.ROTATE:
            self.rotate_to_target(
                yaw_error=yaw_error,
                distance_error=distance_error,
                dt=dt,
            )

        elif self.state == ControlState.DRIVE:
            self.drive_to_target(
                distance_error=distance_error,
                yaw_error=yaw_error,
                is_reverse=is_reverse,
                dt=dt,
            )

    # ================================================================
    # WP1→WP7 연속 곡선주행
    # ================================================================

    def smooth_drive_to_target(
        self,
        distance_error: float,
        dt: float,
    ) -> None:
        is_last_smooth_target = (
            self.target_index >= self.smooth_end_target_index
        )

        arrival_tolerance = (
            self.position_tolerance
            if is_last_smooth_target
            else self.smooth_switch_distance
        )

        if distance_error <= arrival_tolerance:
            reached_number = self.target_index + 1

            if is_last_smooth_target:
                # WP8은 곡선으로 주차가 끝나는 지점이다.
                # 여기서만 정지하고, 차체 방향을 유지한 채 WP9까지 후진한다.
                self.stop_robot()

                self.get_logger().info(
                    f'WP{reached_number} 도착: '
                    '곡선 주차 완료, 잠시 정지 후 직선 후진'
                )

                self.target_index += 1
                self.reset_all_pid()

                self.pause_until_ns = (
                    self.get_clock().now().nanoseconds
                    + int(
                        self.parking_pause_sec
                        * 1_000_000_000
                    )
                )

                if self.target_index >= len(self.waypoints):
                    self.finish_route()
                else:
                    self.state = ControlState.DRIVE
                return

            self.get_logger().info(
                f'WP{reached_number} 곡선 통과'
            )
            self.target_index += 1
            self.smooth_previous_yaw_error = 0.0
            self.smooth_error_initialized = False
            return

        aim_x, aim_y = self.get_smooth_aim_point()

        aim_dx = aim_x - self.current_x
        aim_dy = aim_y - self.current_y
        aim_yaw = math.atan2(aim_dy, aim_dx)

        yaw_error = normalize_angle(
            aim_yaw - self.current_yaw
        )

        if self.smooth_error_initialized:
            yaw_derivative = (
                yaw_error - self.smooth_previous_yaw_error
            ) / max(dt, 0.001)
        else:
            yaw_derivative = 0.0
            self.smooth_error_initialized = True

        self.smooth_previous_yaw_error = yaw_error

        is_parking_curve = self.target_index == 7

        if is_parking_curve and self.parking_yaw is not None:
            # WP7→WP8: 목표점만 바라보면 마지막에 차체가 비스듬해질 수 있다.
            # WP8에 가까워질수록 주차선 방향(parking_yaw)을 더 강하게 섞는다.
            align_ratio = 1.0 - clamp(
                distance_error / self.parking_curve_align_distance,
                0.0,
                1.0,
            )
            align_ratio = align_ratio * align_ratio * (3.0 - 2.0 * align_ratio)

            parking_yaw_error = normalize_angle(
                self.parking_yaw - self.current_yaw
            )
            blended_yaw_error = normalize_angle(
                (1.0 - align_ratio) * yaw_error
                + align_ratio * parking_yaw_error
            )

            angular_command = (
                self.parking_curve_heading_kp * blended_yaw_error
                + self.parking_curve_heading_kd * yaw_derivative
            )
            angular_command = clamp(
                angular_command,
                -self.parking_curve_max_angular_speed,
                self.parking_curve_max_angular_speed,
            )

            heading_factor = clamp(
                1.0 - abs(blended_yaw_error) / math.radians(80.0),
                0.35,
                1.0,
            )
            distance_factor = clamp(
                distance_error / 0.14,
                0.50,
                1.0,
            )
            linear_command = (
                self.parking_curve_max_linear_speed
                * heading_factor
                * distance_factor
            )
            linear_command = clamp(
                linear_command,
                self.parking_curve_min_linear_speed,
                self.parking_curve_max_linear_speed,
            )

        else:
            angular_command = (
                self.smooth_heading_kp * yaw_error
                + self.smooth_heading_kd * yaw_derivative
            )
            angular_command = clamp(
                angular_command,
                -self.smooth_max_angular_speed,
                self.smooth_max_angular_speed,
            )

            abs_yaw_error = abs(yaw_error)

            heading_factor = clamp(
                1.0
                - abs_yaw_error / math.radians(75.0),
                0.28,
                1.0,
            )

            corner_factor = clamp(
                distance_error / self.smooth_corner_radius,
                0.48,
                1.0,
            )

            linear_command = (
                self.smooth_max_linear_speed
                * heading_factor
                * corner_factor
            )
            linear_command = clamp(
                linear_command,
                self.smooth_min_linear_speed,
                self.smooth_max_linear_speed,
            )

        self.publish_velocity(
            linear_x=linear_command,
            angular_z=angular_command,
        )

    def get_smooth_aim_point(self) -> Tuple[float, float]:
        target_x, target_y = self.waypoints[self.target_index]

        if (
            self.target_index >= self.smooth_end_target_index
            or self.target_index + 1 >= len(self.waypoints)
        ):
            return target_x, target_y

        next_x, next_y = self.waypoints[
            self.target_index + 1
        ]

        distance_to_target = math.hypot(
            target_x - self.current_x,
            target_y - self.current_y,
        )

        out_dx = next_x - target_x
        out_dy = next_y - target_y
        out_length = math.hypot(out_dx, out_dy)

        if out_length < 1e-6:
            return target_x, target_y

        proximity = 1.0 - clamp(
            distance_to_target / self.smooth_corner_radius,
            0.0,
            1.0,
        )

        # 부드러운 0→1 보간
        blend = proximity * proximity * (
            3.0 - 2.0 * proximity
        )

        preview_distance = min(
            self.smooth_preview_distance,
            out_length * 0.55,
        ) * blend

        aim_x = (
            target_x
            + out_dx / out_length * preview_distance
        )
        aim_y = (
            target_y
            + out_dy / out_length * preview_distance
        )

        return aim_x, aim_y

    # ================================================================
    # 제자리 회전
    # ================================================================

    def rotate_to_target(
        self,
        yaw_error: float,
        distance_error: float,
        dt: float,
    ) -> None:
        if distance_error <= self.position_tolerance:
            self.reach_current_waypoint()
            return

        is_parking_section = self.target_index in (7, 8)

        active_rotation_tolerance = (
            math.radians(2.0)
            if is_parking_section
            else self.rotation_tolerance
        )

        if abs(yaw_error) <= active_rotation_tolerance:
            self.stop_robot()
            self.reset_all_pid()
            self.state = ControlState.DRIVE

            self.get_logger().info(
                f'WP{self.target_index + 1}: '
                '방향 정렬 완료'
            )
            return

        if is_parking_section:
            # 주차 회전은 오버슈트 방지를 위해 P 제어 중심으로 운용한다.
            angular_command = (
                self.parking_rotate_kp * yaw_error
            )

            abs_error = abs(yaw_error)

            # 목표에 가까워질수록 회전속도를 단계적으로 낮춘다.
            if abs_error <= math.radians(5.0):
                parking_speed_limit = 0.060
            elif abs_error <= math.radians(12.0):
                parking_speed_limit = 0.105
            else:
                parking_speed_limit = (
                    self.parking_max_rotate_speed
                )

            angular_command = clamp(
                angular_command,
                -parking_speed_limit,
                parking_speed_limit,
            )

            # 바닥 마찰로 명령이 먹지 않을 정도로 작을 때만 최소값 사용
            if (
                abs_error > self.rotation_tolerance
                and abs(angular_command)
                < self.parking_min_rotate_speed
            ):
                angular_command = math.copysign(
                    self.parking_min_rotate_speed,
                    yaw_error,
                )

        else:
            angular_command = self.rotate_pid.update(
                yaw_error,
                dt,
            )

            if abs(angular_command) < self.min_angular_speed:
                angular_command = math.copysign(
                    self.min_angular_speed,
                    yaw_error,
                )

            angular_command = clamp(
                angular_command,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

        self.publish_velocity(
            linear_x=0.0,
            angular_z=angular_command,
        )

    # ================================================================
    # 정밀 직진 / 후진
    # ================================================================

    def drive_to_target(
        self,
        distance_error: float,
        yaw_error: float,
        is_reverse: bool,
        dt: float,
    ) -> None:
        if distance_error <= self.position_tolerance:
            self.reach_current_waypoint()
            return

        is_parking_line = self.target_index in (7, 8)

        # 주차선 구간은 목표점 방향이 아니라 고정 주차방향을 유지하므로,
        # 큰 오차가 날 때만 다시 정렬한다.
        abort_angle = (
            math.radians(15.0)
            if is_parking_line
            else self.drive_abort_angle
        )

        if abs(yaw_error) >= abort_angle:
            self.stop_robot()
            self.reset_all_pid()
            self.state = ControlState.ROTATE

            self.get_logger().warn(
                f'WP{self.target_index + 1}: '
                f'각도 오차 '
                f'{math.degrees(yaw_error):.1f}도, '
                '재정렬'
            )
            return

        if is_parking_line:
            # 직선 주차/출차에서는 조향을 약하게만 사용한다.
            angular_command = clamp(
                self.parking_heading_kp * yaw_error,
                -self.parking_max_heading_speed,
                self.parking_max_heading_speed,
            )

            if is_reverse:
                linear_command = self.parking_reverse_speed
            else:
                linear_command = self.parking_forward_speed

        else:
            linear_command = self.linear_pid.update(
                distance_error,
                dt,
            )
            angular_command = self.heading_pid.update(
                yaw_error,
                dt,
            )

            distance_ratio = clamp(
                distance_error
                / self.linear_slowdown_distance,
                0.0,
                1.0,
            )

            linear_limit = (
                self.min_linear_speed
                + (
                    self.max_linear_speed
                    - self.min_linear_speed
                ) * distance_ratio
            )

            linear_command = clamp(
                linear_command,
                self.min_linear_speed,
                linear_limit,
            )

            if abs(yaw_error) > self.slowdown_angle:
                available_angle_range = max(
                    self.drive_abort_angle
                    - self.slowdown_angle,
                    math.radians(1.0),
                )

                angle_ratio = 1.0 - clamp(
                    (
                        abs(yaw_error)
                        - self.slowdown_angle
                    ) / available_angle_range,
                    0.0,
                    1.0,
                )
                linear_command *= (
                    0.25 + 0.75 * angle_ratio
                )

            angular_command = clamp(
                angular_command,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

        if is_reverse:
            linear_command = -abs(linear_command)

        self.publish_velocity(
            linear_x=linear_command,
            angular_z=angular_command,
        )

    # ================================================================
    # 웨이포인트 도착 처리
    # ================================================================

    def reach_current_waypoint(self) -> None:
        self.stop_robot()

        reached_index = self.target_index
        reached_number = reached_index + 1

        target_x, target_y = self.waypoints[
            reached_index
        ]
        actual_error = math.hypot(
            target_x - self.current_x,
            target_y - self.current_y,
        )

        self.get_logger().info(
            f'WP{reached_number} 도착 | '
            f'오차={actual_error:.3f}m'
        )

        self.target_index += 1
        self.reset_all_pid()

        if self.target_index >= len(self.waypoints):
            self.finish_route()
            return

        # 방금 WP8에 전진 주차 완료:
        # 잠시 정지 후 재회전 없이 그대로 WP9까지 후진
        if reached_index == 7:
            self.pause_until_ns = (
                self.get_clock().now().nanoseconds
                + int(
                    self.parking_pause_sec
                    * 1_000_000_000
                )
            )
            self.state = ControlState.DRIVE

            self.get_logger().info(
                '주차 완료: 잠시 정지 후 WP9까지 직선 후진'
            )
            return

        # 방금 WP9까지 후진 출차 완료:
        # 그때만 WP10 출구 방향으로 회전
        if reached_index == 8:
            self.pause_until_ns = (
                self.get_clock().now().nanoseconds
                + int(
                    self.waypoint_pause_sec
                    * 1_000_000_000
                )
            )
            self.state = ControlState.ROTATE

            self.get_logger().info(
                '출차 완료: WP10 출구 방향으로 회전'
            )
            return

        self.pause_until_ns = (
            self.get_clock().now().nanoseconds
            + int(
                self.waypoint_pause_sec
                * 1_000_000_000
            )
        )
        self.state = ControlState.ROTATE

    def finish_route(self) -> None:
        self.stop_robot()
        self.state = ControlState.FINISHED
        self.get_logger().info(
            '모든 웨이포인트 주행 완료'
        )

    # ================================================================
    # 공통 함수
    # ================================================================

    def reset_all_pid(self) -> None:
        self.linear_pid.reset()
        self.rotate_pid.reset()
        self.heading_pid.reset()

        self.smooth_previous_yaw_error = 0.0
        self.smooth_error_initialized = False

        self.parking_previous_yaw_error = 0.0
        self.parking_error_initialized = False

    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float,
    ) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_velocity(
            linear_x=0.0,
            angular_z=0.0,
        )

    def destroy_node(self) -> None:
        for _ in range(5):
            self.stop_robot()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointPIDController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            '사용자 종료 요청, 로봇 정지'
        )

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()