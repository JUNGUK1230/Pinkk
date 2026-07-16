#!/usr/bin/env python3

import math
import csv
import os
from enum import Enum, auto
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class ControlState(Enum):
    WAIT_ODOM = auto()
    ROTATE = auto()
    DRIVE = auto()
    FINAL_ROTATE = auto()
    FINISHED = auto()


class PID:
    """
    기본 PID 제어기.

    - integral_limit: 적분 포화 방지
    - output_limit: 출력 제한
    - derivative_filter: 미분항 저역통과 필터
    """

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

        self.derivative_filter = max(0.0, min(1.0, derivative_filter))

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

        output = clamp(
            output,
            -self.output_limit,
            self.output_limit,
        )

        self.previous_error = error
        return output


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_angle(angle: float) -> float:
    """각도를 -pi ~ +pi 범위로 정규화한다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    """
    Quaternion에서 yaw를 직접 계산한다.
    별도 tf_transformations 패키지가 없어도 동작한다.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(siny_cosp, cosy_cosp)


class WaypointPIDController(Node):

    def __init__(self):
        super().__init__('waypoint_pid_controller')
        self.log_file = None
        self.csv_writer = None
        # ============================================================
        # ROS 파라미터
        # ============================================================

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('control_frequency', 30.0)

        # 사진 좌표 단위가 실제 미터와 같은 경우 1.0
        self.declare_parameter('coordinate_scale', 1.0)

        # 주행 속도
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.025)

        # 회전 속도
        self.declare_parameter('max_angular_speed', 0.80)
        self.declare_parameter('min_angular_speed', 0.10)

        # 목표 판정 허용 오차
        self.declare_parameter('position_tolerance', 0.018)
        self.declare_parameter(
            'rotation_tolerance_deg',
            3.0,
        )

        # 직진 중 각도 오차가 너무 커지면 재회전
        self.declare_parameter(
            'drive_abort_angle_deg',
            25.0,
        )

        # 방향이 안 맞을 때 선속도를 줄이기 시작하는 각도
        self.declare_parameter(
            'slowdown_angle_deg',
            12.0,
        )

        # 목표점 근처 감속 거리
        self.declare_parameter(
            'linear_slowdown_distance',
            0.12,
        )

        # 웨이포인트 도착 후 잠깐 정지하는 시간
        self.declare_parameter(
            'waypoint_pause_sec',
            0.20,
        )

        # 선형 거리 PID
        self.declare_parameter('linear_kp', 0.90)
        self.declare_parameter('linear_ki', 0.00)
        self.declare_parameter('linear_kd', 0.05)

        # 회전 PID
        self.declare_parameter('angular_kp', 2.40)
        self.declare_parameter('angular_ki', 0.00)
        self.declare_parameter('angular_kd', 0.10)

        # 직진 중 방향 보정 PID
        self.declare_parameter('heading_kp', 2.00)
        self.declare_parameter('heading_ki', 0.00)
        self.declare_parameter('heading_kd', 0.07)

        odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )

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
            float(
                self.get_parameter(
                    'rotation_tolerance_deg'
                ).value
            )
        )

        self.drive_abort_angle = math.radians(
            float(
                self.get_parameter(
                    'drive_abort_angle_deg'
                ).value
            )
        )

        self.slowdown_angle = math.radians(
            float(
                self.get_parameter(
                    'slowdown_angle_deg'
                ).value
            )
        )

        self.linear_slowdown_distance = float(
            self.get_parameter(
                'linear_slowdown_distance'
            ).value
        )

        self.waypoint_pause_sec = float(
            self.get_parameter(
                'waypoint_pause_sec'
            ).value
        )

        # ============================================================
        # 사진에서 추출한 원본 좌표
        # ============================================================

        self.map_waypoints: List[Tuple[float, float]] = [
            (1.63, 0.17),  # 1: 출발
            (0.61, 0.16),  # 2
            (0.61, 0.37),  # 3
            (0.52, 0.37),  # 4
            (0.54, 0.80),  # 5
            (0.85, 0.80),  # 6
            (1.04, 0.80),  # 7: 주차구역 앞
            (1.04, 0.56),  # 8: 전진 진입
            (1.04, 0.83),  # 9: 후진 출차
            (1.63, 0.82),  # 10: 최종 도착
        ]

# target_index가 8일 때 목표는 9번 웨이포인트다.
# 즉 8번 → 9번 구간만 후진한다.
        self.reverse_target_indices = {8}

        # 변환된 실제 odom 목표점
        self.waypoints: List[Tuple[float, float]] = []

        # ============================================================
        # PID 생성
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
        # 현재 로봇 상태
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

        # ============================================================
        # Publisher / Subscriber
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
            'Waypoint PID controller started'
        )
        self.get_logger().info(
            f'odom topic: {odom_topic}'
        )
        self.get_logger().info(
            f'cmd_vel topic: {cmd_vel_topic}'
        )
        self.get_logger().info(
            '로봇을 1번 위치에 놓고 1→2 방향을 바라보게 한 뒤 실행하세요.'
        )

    # ================================================================
    # Odometry
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
    # 경로 좌표 변환
    # ================================================================

    def initialize_route(self) -> None:
        """
        사진 좌표를 실행 순간의 odom 좌표로 변환한다.

        사진 좌표:
          x 감소 = 화면 오른쪽 = 로봇 전진
          y 증가 = 화면 아래 = 로봇 오른쪽

        ROS 로봇 로컬 좌표:
          +x = 전진
          +y = 왼쪽

        따라서:
          local_x = map_start_x - map_x
          local_y = map_start_y - map_y
        """

        self.start_x = self.current_x
        self.start_y = self.current_y
        self.start_yaw = self.current_yaw

        map_start_x, map_start_y = self.map_waypoints[0]

        cos_yaw = math.cos(self.start_yaw)
        sin_yaw = math.sin(self.start_yaw)

        self.waypoints.clear()

        for map_x, map_y in self.map_waypoints:
            # 사진 좌표 → 시작 로봇 기준 좌표
            local_forward = (
                map_start_x - map_x
            ) * self.coordinate_scale

            local_left = (
                map_start_y - map_y
            ) * self.coordinate_scale

            # 로봇 시작 자세 기준 좌표 → odom 좌표
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

        self.route_initialized = True
        self.target_index = 1
        self.state = ControlState.ROTATE

        self.reset_all_pid()

        self.get_logger().info(
            '========== 변환된 목표 좌표 =========='
        )

        for index, waypoint in enumerate(self.waypoints):
            self.get_logger().info(
                f'{index + 1}: '
                f'x={waypoint[0]:.3f}, '
                f'y={waypoint[1]:.3f}'
            )

        self.get_logger().info(
            '======================================'
        )
        # ===================== PID LOG =====================
        self.log_file = open(
            os.path.expanduser("~/pid_log.csv"),
            "w",
            newline=""
        )

        
# ===================================================

    # ================================================================
    # 메인 제어 루프
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

        # 타이머 지연으로 dt가 지나치게 커졌을 때 PID 폭주 방지
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

        target_x, target_y = self.waypoints[
            self.target_index
        ]

        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance_error = math.hypot(dx, dy)

        # 현재 구간이 후진 구간인지 확인
        is_reverse = (
            self.target_index in self.reverse_target_indices
        )


        travel_yaw = math.atan2(dy, dx)

        if is_reverse:
            # 목표점 반대 방향을 바라본 상태로 후진
            target_yaw = normalize_angle(
                travel_yaw + math.pi
            )
        else:
            # 목표점을 바라보고 전진
            target_yaw = travel_yaw

        yaw_error = normalize_angle(
            target_yaw - self.current_yaw
        )

        if self.csv_writer is not None:
            self.csv_writer.writerow([
                self.get_clock().now().nanoseconds / 1e9,
                self.target_index + 1,
                round(self.current_x, 4),
                round(self.current_y, 4),
                round(math.degrees(self.current_yaw), 2),
                round(target_x, 4),
                round(target_y, 4),
                round(distance_error, 4),
                round(math.degrees(yaw_error), 2),
            ])

        if self.state == ControlState.ROTATE:
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
    # 제자리 회전
    # ================================================================

    def rotate_to_target(
        self,
        yaw_error: float,
        distance_error: float,
        dt: float,
    ) -> None:

        # 이미 위치 허용 오차 안이라면 다음 점으로
        if distance_error <= self.position_tolerance:
            self.reach_current_waypoint()
            return

        if abs(yaw_error) <= self.rotation_tolerance:
            self.stop_robot()

            self.rotate_pid.reset()
            self.linear_pid.reset()
            self.heading_pid.reset()

            self.state = ControlState.DRIVE

            self.get_logger().info(
                f'WP {self.target_index + 1}: '
                '방향 정렬 완료, 직진 시작'
            )
            return

        angular_command = self.rotate_pid.update(
            yaw_error,
            dt,
        )

        # 마찰 때문에 너무 작은 회전 명령이 안 먹는 문제 방지
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
    # 직진 + 방향 보정
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

        # 주행 중 방향이 크게 틀어지면 직진을 멈추고 다시 회전
        if abs(yaw_error) >= self.drive_abort_angle:
            self.stop_robot()

            self.reset_all_pid()
            self.state = ControlState.ROTATE

            self.get_logger().warn(
                f'WP {self.target_index + 1}: '
                f'각도 오차 {math.degrees(yaw_error):.1f}도, '
                '재정렬'
            )
            return

        linear_command = self.linear_pid.update(
            distance_error,
            dt,
        )

        angular_command = self.heading_pid.update(
            yaw_error,
            dt,
        )

        # 목표점 근처에서 감속
        distance_ratio = clamp(
            distance_error / self.linear_slowdown_distance,
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

        # 방향 오차가 커지면 선속도를 자동으로 감소
        abs_yaw_error = abs(yaw_error)

        if abs_yaw_error > self.slowdown_angle:
            available_angle_range = max(
                self.drive_abort_angle - self.slowdown_angle,
                math.radians(1.0),
            )

            angle_ratio = 1.0 - clamp(
                (
                    abs_yaw_error - self.slowdown_angle
                ) / available_angle_range,
                0.0,
                1.0,
            )

            speed_factor = 0.25 + 0.75 * angle_ratio
            linear_command *= speed_factor

        angular_command = clamp(
            angular_command,
            -self.max_angular_speed,
            self.max_angular_speed,
        )

        # 8번 → 9번 구간에서만 음수 선속도로 후진
        if is_reverse:
            linear_command = -linear_command

        self.publish_velocity(
            linear_x=linear_command,
            angular_z=angular_command,
        )

    # ================================================================
    # 웨이포인트 처리
    # ================================================================

    def reach_current_waypoint(self) -> None:
        self.stop_robot()

        reached_number = self.target_index + 1

        target_x, target_y = self.waypoints[
            self.target_index
        ]

        actual_error = math.hypot(
            target_x - self.current_x,
            target_y - self.current_y,
        )

        if self.log_file is not None and not self.log_file.closed:
            self.log_file.flush()

        self.get_logger().info(
            f'WP {reached_number} 도착 | '
            f'오차={actual_error:.3f} m | '
            f'현재=({self.current_x:.3f}, '
            f'{self.current_y:.3f})'
        )

        self.target_index += 1
        self.reset_all_pid()

        self.pause_until_ns = (
            self.get_clock().now().nanoseconds
            + int(self.waypoint_pause_sec * 1_000_000_000)
        )

        if self.target_index >= len(self.waypoints):
            self.finish_route()
        else:
            self.state = ControlState.ROTATE

    def finish_route(self) -> None:
        self.stop_robot()
        self.state = ControlState.FINISHED

        if self.log_file is not None and not self.log_file.closed:
            self.log_file.flush()

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

    def publish_velocity(
        self,
        linear_x: float,
        angular_z: float,
    ) -> None:
        msg = Twist()

        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.cmd_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_velocity(
            linear_x=0.0,
            angular_z=0.0,
        )

    def destroy_node(self):
        if self.log_file is not None and not self.log_file.closed:
            self.log_file.flush()
            self.log_file.close()

        for _ in range(5):
            self.stop_robot()

        super().destroy_node()

def main(args=None):
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