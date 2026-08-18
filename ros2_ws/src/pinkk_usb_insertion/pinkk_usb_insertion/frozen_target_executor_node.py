"""초기 PBVS 목표를 고정하고 flange TF 오차만 반복 보정하는 시험 노드."""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from copy import deepcopy
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
import numpy as np
from pinkk_usb_insertion_interfaces.action import CartesianMove
from pinkk_usb_insertion_interfaces.msg import UsbPortObservation
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .control.frozen_target import (
    circular_mean_degrees,
    circular_median_degrees,
    final_insertion_target_z_m,
    limited_xy_target,
    maximum_angular_deviation_degrees,
    port_based_flange_target_z,
    proportional_xy_target,
    proportional_z_descent_m,
    xy_residual_m,
)
from .control.auto_start import AutoStartSample, evaluate_auto_start
from .control.serial_chain import (
    SerialChainModel,
    damped_joint_step,
)
from .control.yaw_alignment import (
    apply_observation_roll_pitch_with_current_yaw,
    apply_proportional_observation_roll_pitch_with_current_yaw,
    calibrated_keypoint_joint_step_rad,
    joint6_yaw_target_rad,
)
from .geometry.transforms import (
    make_transform,
    rotation_to_rpy_degrees,
    rpy_degrees_to_rotation,
)
from .ros_utils import pose_to_transform, transform_to_pose


JOINT_NAMES = (
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
)
CARTESIAN_ACTION = '/robot_arm/cartesian_move'
JOINT_ACTION = '/arm_group_controller/follow_joint_trajectory'
COMMAND_TOPIC = '/robot_arm/frozen_target/command'
STATUS_TOPIC = '/robot_arm/frozen_target/status'


class FrozenTargetExecutorNode(Node):
    """한 번 관측한 목표를 로봇 좌표 폐루프로 추종한 뒤 Yaw를 실행한다."""

    def __init__(
        self,
        *,
        node_name: str = 'pinkk_frozen_target_executor_node',
        command_topic: str = COMMAND_TOPIC,
        status_topic: str = STATUS_TOPIC,
        proportional_control: bool = False,
    ) -> None:
        super().__init__(node_name)
        self._proportional_control = bool(proportional_control)
        self.declare_parameter('enable_execution', False)
        self.declare_parameter('base_frame', 'g_base')
        self.declare_parameter('flange_frame', 'joint6_flange')
        self.declare_parameter('maximum_input_age_seconds', 1.0)
        # 영상 중앙에서 일정 시간 정지한 포트를 확인한 뒤 통합 제어를
        # 프로세스당 한 번 자동 시작한다.
        self.declare_parameter('auto_start_enabled', False)
        self.declare_parameter('auto_start_stable_duration_seconds', 5.0)
        self.declare_parameter('auto_start_minimum_samples', 30)
        self.declare_parameter('auto_start_image_center_u_px', 320.0)
        self.declare_parameter('auto_start_image_center_v_px', 240.0)
        self.declare_parameter('auto_start_center_tolerance_px', 80.0)
        self.declare_parameter('auto_start_maximum_center_spread_px', 5.0)
        self.declare_parameter('auto_start_maximum_depth_spread_m', 0.005)
        self.declare_parameter('auto_start_maximum_yaw_spread_deg', 2.0)
        self.declare_parameter('auto_start_maximum_sample_gap_seconds', 0.5)
        self.declare_parameter(
            'hardware_cartesian_pose_maximum_age_seconds', 2.0
        )
        self.declare_parameter('initial_observation_sample_count', 5)
        self.declare_parameter(
            'initial_observation_aggregation_method', 'median'
        )
        self.declare_parameter(
            'initial_observation_sample_window_seconds', 1.0
        )
        self.declare_parameter(
            'initial_observation_maximum_xy_spread_m', 0.008
        )
        self.declare_parameter(
            'initial_observation_maximum_z_spread_m', 0.015
        )
        self.declare_parameter(
            'initial_observation_maximum_yaw_spread_deg', 8.0
        )
        self.declare_parameter('minimum_xy_step_m', 0.003)
        self.declare_parameter('maximum_coarse_xy_step_m', 0.100)
        self.declare_parameter(
            'maximum_partial_coarse_xy_residual_m', 0.030
        )
        self.declare_parameter('robot_xy_tracking_tolerance_m', 0.005)
        self.declare_parameter('robot_xy_refine_max_cycles', 3)
        self.declare_parameter('robot_xy_refine_maximum_step_m', 0.020)
        self.declare_parameter('robot_xy_minimum_improvement_m', 0.001)
        self.declare_parameter('stop_on_insufficient_xy_improvement', True)
        self.declare_parameter('robot_xy_kp', 0.7)
        self.declare_parameter('cartesian_speed', 10)
        self.declare_parameter('cartesian_mode', 1)
        self.declare_parameter('cartesian_timeout_seconds', 100.0)
        self.declare_parameter('warning_delay_seconds', 3.0)
        self.declare_parameter('settle_seconds', 0.8)
        self.declare_parameter('use_pbvs_target_z', True)
        self.declare_parameter('minimum_target_z_m', 0.100)
        self.declare_parameter('maximum_target_z_m', 0.350)
        self.declare_parameter('lock_z', True)
        self.declare_parameter('lock_roll_pitch', True)
        self.declare_parameter('use_fixed_roll_pitch_target', False)
        self.declare_parameter('fixed_roll_target_deg', -180.0)
        self.declare_parameter('fixed_pitch_target_deg', 0.0)
        self.declare_parameter('pitch_correction_gain', 1.0)
        self.declare_parameter('roll_pitch_tolerance_deg', 5.0)
        self.declare_parameter('enable_roll_pitch_recovery', True)
        self.declare_parameter('roll_pitch_recovery_max_cycles', 2)
        self.declare_parameter('roll_pitch_kp', 0.5)
        self.declare_parameter('final_coupled_validation_max_cycles', 2)
        self.declare_parameter('enable_final_z_descent', True)
        self.declare_parameter('final_z_descent_step_m', 0.005)
        self.declare_parameter('final_z_descent_speed', 5)
        self.declare_parameter('final_z_descent_cartesian_mode', 0)
        self.declare_parameter('vertical_z_cartesian_speed', 5)
        self.declare_parameter('vertical_z_cartesian_mode', 1)
        # joint=기존 Jacobian/send_angles, cartesian=send_coords Z-only.
        self.declare_parameter('vertical_z_control_backend', 'cartesian')
        self.declare_parameter(
            'final_z_descent_minimum_progress_m', 0.002
        )
        self.declare_parameter('final_z_descent_minimum_z_m', 0.160)
        self.declare_parameter('enable_final_z_p_descent', True)
        self.declare_parameter('final_z_p_z_kp', 0.4)
        self.declare_parameter('final_z_p_maximum_overshoot_m', 0.003)
        self.declare_parameter('final_z_p_maximum_coupled_z_drift_m', 0.003)
        self.declare_parameter('final_z_p_roll_pitch_kp', 0.4)
        self.declare_parameter('final_z_p_max_roll_pitch_step_deg', 1.5)
        self.declare_parameter('final_z_p_roll_pitch_deadband_deg', 0.7)
        self.declare_parameter('final_z_p_roll_pitch_abort_deg', 12.0)
        self.declare_parameter('final_z_p_roll_command_sign', 1.0)
        self.declare_parameter('final_z_p_pitch_command_sign', 1.0)
        self.declare_parameter(
            'final_z_p_minimum_tilt_improvement_deg', 0.1
        )
        self.declare_parameter('final_z_p_xy_kp', 0.3)
        self.declare_parameter('final_z_p_max_xy_step_m', 0.005)
        self.declare_parameter('final_z_p_xy_deadband_m', 0.004)
        self.declare_parameter('final_z_p_use_port_target', True)
        self.declare_parameter('final_tcp_offset_z_m', 0.120)
        self.declare_parameter('final_port_insertion_depth_m', 0.010)
        # 힘 센서가 없는 마지막 구간은 자동 반복하지 않고 명시적인 단발
        # 승인으로만 이동한다.
        self.declare_parameter('enable_final_insertion', False)
        self.declare_parameter('final_insertion_step_m', 0.0005)
        self.declare_parameter('final_insertion_speed', 1)
        self.declare_parameter('final_insertion_maximum_actual_step_m', 0.002)
        self.declare_parameter('final_insertion_target_tolerance_m', 0.0005)
        self.declare_parameter('final_z_p_target_z_m', 0.190)
        self.declare_parameter('final_z_p_max_cycles', 12)
        self.declare_parameter('enable_initial_yaw', True)
        self.declare_parameter('keypoint_desired_axis_deg', 0.0)
        self.declare_parameter('minimum_yaw_step_deg', 3.0)
        self.declare_parameter('keypoint_maximum_step_deg', 5.0)
        self.declare_parameter('keypoint_tolerance_deg', 3.0)
        self.declare_parameter('keypoint_command_sign', -1.0)
        self.declare_parameter('keypoint_joint_gain', 1.32)
        self.declare_parameter('joint6_direction', -1.0)
        self.declare_parameter('joint6_limit_deg', 175.0)
        self.declare_parameter('joint6_move_seconds', 15.0)
        # send_coords를 사용하지 않는 관절 Jacobian Z 하강 단발 시험.
        self.declare_parameter('enable_joint_vertical_descent', True)
        self.declare_parameter(
            'joint_vertical_robot_description_package',
            'mycobot_description',
        )
        self.declare_parameter(
            'joint_vertical_robot_description_relative_path',
            'urdf/mycobot_280_m5/mycobot_280_m5.urdf',
        )
        self.declare_parameter('joint_vertical_z_kp', 0.4)
        self.declare_parameter('joint_vertical_z_step_m', 0.006)
        self.declare_parameter('joint_vertical_xy_kp', 0.5)
        self.declare_parameter('joint_vertical_orientation_kp', 0.3)
        self.declare_parameter('joint_vertical_damping', 0.01)
        self.declare_parameter(
            'joint_vertical_orientation_scale_m_per_rad', 0.05
        )
        self.declare_parameter('joint_vertical_max_joint_step_deg', 2.0)
        self.declare_parameter('joint_vertical_joint_limit_margin_deg', 3.0)
        self.declare_parameter('joint_vertical_motion_seconds', 10.0)
        self.declare_parameter('joint_vertical_minimum_progress_m', 0.0005)
        self.declare_parameter('joint_vertical_maximum_overshoot_m', 0.003)
        self.declare_parameter(
            'allow_mixed_correction_below_final_z', False
        )
        self.declare_parameter(
            'joint_vertical_hard_maximum_descent_m', 0.015
        )
        self.declare_parameter(
            'joint_vertical_xy_correction_deadband_m', 0.003
        )
        self.declare_parameter(
            'joint_vertical_roll_pitch_correction_deadband_deg', 5.0
        )
        self.declare_parameter(
            'joint_vertical_roll_pitch_hard_limit_deg', 12.0
        )
        self.declare_parameter(
            'joint_vertical_minimum_tilt_improvement_deg', 0.2
        )
        self.declare_parameter(
            'joint_vertical_maximum_correction_z_drift_m', 0.005
        )
        self.declare_parameter('joint_vertical_final_z_guard_m', 0.015)
        self.declare_parameter(
            'joint_vertical_hard_maximum_cycle_descent_m', 0.030
        )
        self.declare_parameter('joint_vertical_max_cycles', 8)
        self.declare_parameter('joint_vertical_xy_tolerance_m', 0.005)
        self.declare_parameter(
            'joint_vertical_roll_pitch_tolerance_deg', 5.0
        )
        # guard 도달 뒤 별도 승인 또는 전용 통합 명령으로 수행하는 최종 Z.
        self.declare_parameter('enable_final_insertion_z', True)
        self.declare_parameter('final_insertion_relative_distance_m', 0.010)
        self.declare_parameter('final_insertion_tolerance_m', 0.002)
        self.declare_parameter(
            'final_insertion_hard_maximum_total_descent_m', 0.015
        )
        # 최종 Z 이동 뒤 초기 관측 R/P 전체값을 한 번 적용한다. 이 시험
        # 단계에서는 결합 Z 하강을 제한하거나 실패 조건으로 사용하지 않는다.
        self.declare_parameter(
            'enable_post_insertion_roll_pitch_recovery', True
        )
        self.declare_parameter('post_insertion_roll_pitch_speed', 10)
        # 최종 Z 삽입과 R/P 복구가 끝난 뒤 상대 Z를 한 번 더 내린다.
        self.declare_parameter('enable_post_recovery_final_z', True)
        self.declare_parameter('post_recovery_final_z_distance_m', 0.005)

        self._enabled = bool(self.get_parameter('enable_execution').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._flange_frame = str(self.get_parameter('flange_frame').value)
        self._maximum_age = float(
            self.get_parameter('maximum_input_age_seconds').value
        )
        self._auto_start_enabled = bool(
            self.get_parameter('auto_start_enabled').value
        )
        self._auto_start_stable_duration = float(
            self.get_parameter('auto_start_stable_duration_seconds').value
        )
        self._auto_start_minimum_samples = int(
            self.get_parameter('auto_start_minimum_samples').value
        )
        self._auto_start_image_center_u = float(
            self.get_parameter('auto_start_image_center_u_px').value
        )
        self._auto_start_image_center_v = float(
            self.get_parameter('auto_start_image_center_v_px').value
        )
        self._auto_start_center_tolerance = float(
            self.get_parameter('auto_start_center_tolerance_px').value
        )
        self._auto_start_maximum_center_spread = float(
            self.get_parameter(
                'auto_start_maximum_center_spread_px'
            ).value
        )
        self._auto_start_maximum_depth_spread = float(
            self.get_parameter(
                'auto_start_maximum_depth_spread_m'
            ).value
        )
        self._auto_start_maximum_yaw_spread = float(
            self.get_parameter(
                'auto_start_maximum_yaw_spread_deg'
            ).value
        )
        self._auto_start_maximum_sample_gap = float(
            self.get_parameter(
                'auto_start_maximum_sample_gap_seconds'
            ).value
        )
        self._hardware_pose_maximum_age = float(
            self.get_parameter(
                'hardware_cartesian_pose_maximum_age_seconds'
            ).value
        )
        self._initial_sample_count = int(
            self.get_parameter('initial_observation_sample_count').value
        )
        self._initial_aggregation_method = str(
            self.get_parameter(
                'initial_observation_aggregation_method'
            ).value
        ).strip().lower()
        self._initial_sample_window = float(
            self.get_parameter(
                'initial_observation_sample_window_seconds'
            ).value
        )
        self._initial_maximum_xy_spread = float(
            self.get_parameter(
                'initial_observation_maximum_xy_spread_m'
            ).value
        )
        self._initial_maximum_z_spread = float(
            self.get_parameter(
                'initial_observation_maximum_z_spread_m'
            ).value
        )
        self._initial_maximum_yaw_spread = float(
            self.get_parameter(
                'initial_observation_maximum_yaw_spread_deg'
            ).value
        )
        self._minimum_xy_step = float(
            self.get_parameter('minimum_xy_step_m').value
        )
        self._maximum_coarse_step = float(
            self.get_parameter('maximum_coarse_xy_step_m').value
        )
        self._maximum_partial_coarse_residual = float(
            self.get_parameter(
                'maximum_partial_coarse_xy_residual_m'
            ).value
        )
        self._tracking_tolerance = float(
            self.get_parameter('robot_xy_tracking_tolerance_m').value
        )
        self._refine_max_cycles = int(
            self.get_parameter('robot_xy_refine_max_cycles').value
        )
        self._refine_maximum_step = float(
            self.get_parameter('robot_xy_refine_maximum_step_m').value
        )
        self._minimum_improvement = float(
            self.get_parameter('robot_xy_minimum_improvement_m').value
        )
        self._stop_on_insufficient_improvement = bool(
            self.get_parameter(
                'stop_on_insufficient_xy_improvement'
            ).value
        )
        self._robot_xy_kp = float(
            self.get_parameter('robot_xy_kp').value
        )
        self._cartesian_speed = int(
            self.get_parameter('cartesian_speed').value
        )
        self._cartesian_mode = int(
            self.get_parameter('cartesian_mode').value
        )
        self._cartesian_timeout = float(
            self.get_parameter('cartesian_timeout_seconds').value
        )
        self._warning_delay = float(
            self.get_parameter('warning_delay_seconds').value
        )
        self._settle = float(self.get_parameter('settle_seconds').value)
        self._use_pbvs_target_z = bool(
            self.get_parameter('use_pbvs_target_z').value
        )
        self._minimum_target_z = float(
            self.get_parameter('minimum_target_z_m').value
        )
        self._maximum_target_z = float(
            self.get_parameter('maximum_target_z_m').value
        )
        self._lock_z = bool(self.get_parameter('lock_z').value)
        self._lock_roll_pitch = bool(
            self.get_parameter('lock_roll_pitch').value
        )
        self._use_fixed_roll_pitch_target = bool(
            self.get_parameter('use_fixed_roll_pitch_target').value
        )
        self._fixed_roll_target = float(
            self.get_parameter('fixed_roll_target_deg').value
        )
        self._fixed_pitch_target = float(
            self.get_parameter('fixed_pitch_target_deg').value
        )
        self._pitch_correction_gain = float(
            self.get_parameter('pitch_correction_gain').value
        )
        self._roll_pitch_tolerance = float(
            self.get_parameter('roll_pitch_tolerance_deg').value
        )
        self._enable_roll_pitch_recovery = bool(
            self.get_parameter('enable_roll_pitch_recovery').value
        )
        self._roll_pitch_recovery_max_cycles = int(
            self.get_parameter('roll_pitch_recovery_max_cycles').value
        )
        self._roll_pitch_kp = float(
            self.get_parameter('roll_pitch_kp').value
        )
        self._final_coupled_validation_max_cycles = int(
            self.get_parameter(
                'final_coupled_validation_max_cycles'
            ).value
        )
        self._enable_final_z_descent = bool(
            self.get_parameter('enable_final_z_descent').value
        )
        self._final_z_descent_step = float(
            self.get_parameter('final_z_descent_step_m').value
        )
        self._final_z_descent_speed = int(
            self.get_parameter('final_z_descent_speed').value
        )
        self._final_z_descent_cartesian_mode = int(
            self.get_parameter('final_z_descent_cartesian_mode').value
        )
        self._vertical_z_cartesian_speed = int(
            self.get_parameter('vertical_z_cartesian_speed').value
        )
        self._vertical_z_cartesian_mode = int(
            self.get_parameter('vertical_z_cartesian_mode').value
        )
        self._vertical_z_control_backend = str(
            self.get_parameter('vertical_z_control_backend').value
        ).strip().lower()
        self._final_z_descent_minimum_progress = float(
            self.get_parameter(
                'final_z_descent_minimum_progress_m'
            ).value
        )
        self._final_z_descent_minimum_z = float(
            self.get_parameter('final_z_descent_minimum_z_m').value
        )
        self._enable_final_z_p_descent = bool(
            self.get_parameter('enable_final_z_p_descent').value
        )
        self._final_z_p_z_kp = float(
            self.get_parameter('final_z_p_z_kp').value
        )
        self._final_z_p_maximum_overshoot = float(
            self.get_parameter('final_z_p_maximum_overshoot_m').value
        )
        self._final_z_p_maximum_coupled_z_drift = float(
            self.get_parameter(
                'final_z_p_maximum_coupled_z_drift_m'
            ).value
        )
        self._final_z_p_roll_pitch_kp = float(
            self.get_parameter('final_z_p_roll_pitch_kp').value
        )
        self._final_z_p_max_roll_pitch_step = float(
            self.get_parameter(
                'final_z_p_max_roll_pitch_step_deg'
            ).value
        )
        self._final_z_p_roll_pitch_deadband = float(
            self.get_parameter(
                'final_z_p_roll_pitch_deadband_deg'
            ).value
        )
        self._final_z_p_roll_pitch_abort = float(
            self.get_parameter('final_z_p_roll_pitch_abort_deg').value
        )
        self._final_z_p_roll_command_sign = float(
            self.get_parameter('final_z_p_roll_command_sign').value
        )
        self._final_z_p_pitch_command_sign = float(
            self.get_parameter('final_z_p_pitch_command_sign').value
        )
        self._final_z_p_minimum_tilt_improvement = float(
            self.get_parameter(
                'final_z_p_minimum_tilt_improvement_deg'
            ).value
        )
        self._final_z_p_xy_kp = float(
            self.get_parameter('final_z_p_xy_kp').value
        )
        self._final_z_p_max_xy_step = float(
            self.get_parameter('final_z_p_max_xy_step_m').value
        )
        self._final_z_p_xy_deadband = float(
            self.get_parameter('final_z_p_xy_deadband_m').value
        )
        self._final_z_p_use_port_target = bool(
            self.get_parameter('final_z_p_use_port_target').value
        )
        self._final_tcp_offset_z = float(
            self.get_parameter('final_tcp_offset_z_m').value
        )
        self._final_port_insertion_depth = float(
            self.get_parameter('final_port_insertion_depth_m').value
        )
        self._enable_final_insertion = bool(
            self.get_parameter('enable_final_insertion').value
        )
        self._final_insertion_step = float(
            self.get_parameter('final_insertion_step_m').value
        )
        self._final_insertion_speed = int(
            self.get_parameter('final_insertion_speed').value
        )
        self._final_insertion_maximum_actual_step = float(
            self.get_parameter(
                'final_insertion_maximum_actual_step_m'
            ).value
        )
        self._final_insertion_target_tolerance = float(
            self.get_parameter(
                'final_insertion_target_tolerance_m'
            ).value
        )
        self._final_z_p_target_z = float(
            self.get_parameter('final_z_p_target_z_m').value
        )
        self._final_z_p_max_cycles = int(
            self.get_parameter('final_z_p_max_cycles').value
        )
        self._enable_initial_yaw = bool(
            self.get_parameter('enable_initial_yaw').value
        )
        self._desired_axis = float(
            self.get_parameter('keypoint_desired_axis_deg').value
        )
        self._minimum_yaw_step = float(
            self.get_parameter('minimum_yaw_step_deg').value
        )
        self._maximum_yaw_step = float(
            self.get_parameter('keypoint_maximum_step_deg').value
        )
        self._yaw_tolerance = float(
            self.get_parameter('keypoint_tolerance_deg').value
        )
        self._keypoint_command_sign = float(
            self.get_parameter('keypoint_command_sign').value
        )
        self._keypoint_joint_gain = float(
            self.get_parameter('keypoint_joint_gain').value
        )
        self._joint6_direction = float(
            self.get_parameter('joint6_direction').value
        )
        self._joint6_limit = float(
            self.get_parameter('joint6_limit_deg').value
        )
        self._joint6_move_seconds = float(
            self.get_parameter('joint6_move_seconds').value
        )
        self._enable_joint_vertical_descent = bool(
            self.get_parameter('enable_joint_vertical_descent').value
        )
        self._joint_vertical_description_package = str(
            self.get_parameter(
                'joint_vertical_robot_description_package'
            ).value
        )
        self._joint_vertical_description_relative_path = str(
            self.get_parameter(
                'joint_vertical_robot_description_relative_path'
            ).value
        )
        self._joint_vertical_z_kp = float(
            self.get_parameter('joint_vertical_z_kp').value
        )
        self._joint_vertical_max_z_step = float(
            self.get_parameter('joint_vertical_z_step_m').value
        )
        self._joint_vertical_xy_kp = float(
            self.get_parameter('joint_vertical_xy_kp').value
        )
        self._joint_vertical_orientation_kp = float(
            self.get_parameter('joint_vertical_orientation_kp').value
        )
        self._joint_vertical_damping = float(
            self.get_parameter('joint_vertical_damping').value
        )
        self._joint_vertical_orientation_scale = float(
            self.get_parameter(
                'joint_vertical_orientation_scale_m_per_rad'
            ).value
        )
        self._joint_vertical_max_joint_step = math.radians(
            float(
                self.get_parameter(
                    'joint_vertical_max_joint_step_deg'
                ).value
            )
        )
        self._joint_vertical_joint_limit_margin = math.radians(
            float(
                self.get_parameter(
                    'joint_vertical_joint_limit_margin_deg'
                ).value
            )
        )
        self._joint_vertical_motion_seconds = float(
            self.get_parameter('joint_vertical_motion_seconds').value
        )
        self._joint_vertical_minimum_progress = float(
            self.get_parameter('joint_vertical_minimum_progress_m').value
        )
        self._joint_vertical_maximum_overshoot = float(
            self.get_parameter('joint_vertical_maximum_overshoot_m').value
        )
        self._allow_mixed_correction_below_final_z = bool(
            self.get_parameter(
                'allow_mixed_correction_below_final_z'
            ).value
        )
        self._joint_vertical_hard_maximum_descent = float(
            self.get_parameter(
                'joint_vertical_hard_maximum_descent_m'
            ).value
        )
        self._joint_vertical_xy_correction_deadband = float(
            self.get_parameter(
                'joint_vertical_xy_correction_deadband_m'
            ).value
        )
        self._joint_vertical_roll_pitch_correction_deadband = float(
            self.get_parameter(
                'joint_vertical_roll_pitch_correction_deadband_deg'
            ).value
        )
        self._joint_vertical_roll_pitch_hard_limit = float(
            self.get_parameter(
                'joint_vertical_roll_pitch_hard_limit_deg'
            ).value
        )
        self._joint_vertical_minimum_tilt_improvement = float(
            self.get_parameter(
                'joint_vertical_minimum_tilt_improvement_deg'
            ).value
        )
        self._joint_vertical_maximum_correction_z_drift = float(
            self.get_parameter(
                'joint_vertical_maximum_correction_z_drift_m'
            ).value
        )
        self._joint_vertical_final_z_guard = float(
            self.get_parameter('joint_vertical_final_z_guard_m').value
        )
        self._joint_vertical_hard_maximum_cycle_descent = float(
            self.get_parameter(
                'joint_vertical_hard_maximum_cycle_descent_m'
            ).value
        )
        self._joint_vertical_max_cycles = int(
            self.get_parameter('joint_vertical_max_cycles').value
        )
        self._joint_vertical_xy_tolerance = float(
            self.get_parameter('joint_vertical_xy_tolerance_m').value
        )
        self._joint_vertical_roll_pitch_tolerance = float(
            self.get_parameter(
                'joint_vertical_roll_pitch_tolerance_deg'
            ).value
        )
        self._enable_final_insertion_z = bool(
            self.get_parameter('enable_final_insertion_z').value
        )
        self._final_insertion_relative_distance = float(
            self.get_parameter('final_insertion_relative_distance_m').value
        )
        self._final_insertion_tolerance = float(
            self.get_parameter('final_insertion_tolerance_m').value
        )
        self._final_insertion_hard_maximum_total_descent = float(
            self.get_parameter(
                'final_insertion_hard_maximum_total_descent_m'
            ).value
        )
        self._enable_post_insertion_roll_pitch_recovery = bool(
            self.get_parameter(
                'enable_post_insertion_roll_pitch_recovery'
            ).value
        )
        self._post_insertion_roll_pitch_speed = int(
            self.get_parameter('post_insertion_roll_pitch_speed').value
        )
        self._enable_post_recovery_final_z = bool(
            self.get_parameter('enable_post_recovery_final_z').value
        )
        self._post_recovery_final_z_distance = float(
            self.get_parameter('post_recovery_final_z_distance_m').value
        )
        self._validate_parameters()

        self._joint_vertical_chain = None
        if self._enable_joint_vertical_descent:
            description_share = get_package_share_directory(
                self._joint_vertical_description_package
            )
            description_path = (
                Path(description_share)
                / self._joint_vertical_description_relative_path
            )
            self._joint_vertical_chain = SerialChainModel.from_urdf_file(
                description_path,
                self._base_frame,
                self._flange_frame,
            )
            if self._joint_vertical_chain.active_joints != JOINT_NAMES:
                raise ValueError(
                    '관절 Jacobian URDF 순서가 bridge 순서와 다릅니다: '
                    f'urdf={self._joint_vertical_chain.active_joints}, '
                    f'bridge={JOINT_NAMES}'
                )

        self._latest_target: PoseStamped | None = None
        self._target_received_at: float | None = None
        self._latest_hardware_pose: PoseStamped | None = None
        self._hardware_pose_received_at: float | None = None
        self._latest_port_pose: PoseStamped | None = None
        self._port_received_at: float | None = None
        self._observation_reference: PoseStamped | None = None
        self._latest_axis: float | None = None
        self._axis_received_at: float | None = None
        self._latest_joints: list[float] | None = None
        self._joints_received_at: float | None = None
        sample_buffer_size = max(30, self._initial_sample_count * 4)
        self._target_samples = deque(maxlen=sample_buffer_size)
        self._port_z_samples = deque(maxlen=sample_buffer_size)
        self._axis_samples = deque(maxlen=sample_buffer_size)
        auto_buffer_size = max(
            300,
            self._auto_start_minimum_samples * 4,
        )
        self._auto_start_samples = deque(maxlen=auto_buffer_size)
        self._auto_start_triggered = False
        self._auto_start_last_reason = ''
        self._aligned_frozen_xy = None
        self._aligned_observation_transform = None
        self._aligned_port_z: float | None = None
        self._aligned_final_target_z: float | None = None
        self._aligned_hardware_xy = None
        self._aligned_hardware_observation_transform = None
        self._aligned_hardware_final_target_z: float | None = None
        self._aligned_hardware_yaw_deg: float | None = None
        self._alignment_ready_for_descent = False
        self._insertion_ready = False
        self._final_insertion_ready = False
        self._executing = False
        self._lock = threading.Lock()

        callbacks = ReentrantCallbackGroup()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cartesian_action = ActionClient(
            self,
            CartesianMove,
            CARTESIAN_ACTION,
            callback_group=callbacks,
        )
        self._joint_action = ActionClient(
            self,
            FollowJointTrajectory,
            JOINT_ACTION,
            callback_group=callbacks,
        )
        self._status = self.create_publisher(String, status_topic, 10)
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/target_flange_pose',
            self._target_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            PoseStamped,
            '/robot_arm/cartesian_pose_actual',
            self._hardware_pose_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/port_pose_base',
            self._port_pose_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            PoseStamped,
            '/robot_arm/pbvs/observation_reference_pose',
            self._observation_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            Float64,
            '/robot_arm/perception/usb_port/keypoint_axis_angle_deg',
            self._axis_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            UsbPortObservation,
            '/robot_arm/perception/usb_port/observation',
            self._auto_start_observation_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10,
            callback_group=callbacks,
        )
        self.create_subscription(
            String,
            command_topic,
            self._command_callback,
            10,
            callback_group=callbacks,
        )
        if self._auto_start_enabled:
            self.create_timer(
                0.5,
                self._auto_start_timer_callback,
                callback_group=callbacks,
            )
        mode = '허용' if self._enabled else '차단'
        self.get_logger().warning(
            '초기 관측 고정목표 실행기: '
            '실행='
            f'{mode}, commands=execute_once/yaw_only_once/'
            'descend_joint_z_once/descend_joint_z_to_guard/'
            'insert_final_z_once/execute_full_sequence/'
            'execute_full_sequence_with_final_z/insert_step_once, '
            f'p_control={self._proportional_control}, '
            'hardware_pose_max_age='
            f'{self._hardware_pose_maximum_age:.1f}s, '
            f'initial_samples={self._initial_sample_count}, '
            f'initial_aggregation={self._initial_aggregation_method}, '
            f'sample_window={self._initial_sample_window:.1f}s, '
            f'xy_kp={self._robot_xy_kp:.2f}, '
            f'roll_pitch_kp={self._roll_pitch_kp:.2f}, '
            f'mode={self._cartesian_mode}, '
            f'xy_tolerance={self._tracking_tolerance * 1000.0:.1f}mm, '
            'partial_coarse_limit='
            f'{self._maximum_partial_coarse_residual * 1000.0:.1f}mm, '
            f'refine_cycles={self._refine_max_cycles}, '
            f'refine_max_step={self._refine_maximum_step * 1000.0:.1f}mm, '
            'stop_on_low_improvement='
            f'{self._stop_on_insufficient_improvement}, '
            f'pbvs_target_z={self._use_pbvs_target_z}, '
            f'lock_z={self._lock_z}, '
            f'lock_roll_pitch={self._lock_roll_pitch}, '
            f'fixed_rp={self._use_fixed_roll_pitch_target}, '
            'fixed_rp_target='
            f'[{self._fixed_roll_target:+.1f}, '
            f'{self._fixed_pitch_target:+.1f}]deg, '
            f'pitch_correction_gain={self._pitch_correction_gain:.2f}, '
            f'roll_pitch_recovery={self._enable_roll_pitch_recovery}, '
            'roll_pitch_recovery_cycles='
            f'{self._roll_pitch_recovery_max_cycles}, '
            'coupled_validation_cycles='
            f'{self._final_coupled_validation_max_cycles}, '
            f'final_z_descent={self._enable_final_z_descent}, '
            f'final_insertion={self._enable_final_insertion}, '
            'insertion_depth='
            f'{self._final_port_insertion_depth * 1000.0:.1f}mm, '
            f'insertion_step={self._final_insertion_step * 1000.0:.1f}mm, '
            f'z_step={self._final_z_descent_step * 1000.0:.1f}mm, '
            f'z_speed={self._final_z_descent_speed}, '
            f'z_mode={self._final_z_descent_cartesian_mode}, '
            'z_minimum_progress='
            f'{self._final_z_descent_minimum_progress * 1000.0:.1f}mm, '
            f'z_p_descent={self._enable_final_z_p_descent}, '
            f'z_p_z_kp={self._final_z_p_z_kp:.2f}, '
            'z_p_max_overshoot='
            f'{self._final_z_p_maximum_overshoot * 1000.0:.1f}mm, '
            f'z_p_rp_kp={self._final_z_p_roll_pitch_kp:.2f}, '
            'z_p_rp_max_step='
            f'{self._final_z_p_max_roll_pitch_step:.1f}deg, '
            'z_p_rp_signs='
            f'[{self._final_z_p_roll_command_sign:+.0f}, '
            f'{self._final_z_p_pitch_command_sign:+.0f}], '
            f'z_p_xy_kp={self._final_z_p_xy_kp:.2f}, '
            f'z_p_xy_max_step={self._final_z_p_max_xy_step * 1000.0:.1f}mm, '
            f'z_p_port_target={self._final_z_p_use_port_target}, '
            f'tcp_z={self._final_tcp_offset_z * 1000.0:.1f}mm, '
            'insertion_depth='
            f'{self._final_port_insertion_depth * 1000.0:.1f}mm, '
            'manual_z_p_target='
            f'{self._final_z_p_target_z * 1000.0:.1f}mm, '
            f'z_p_max_cycles={self._final_z_p_max_cycles}, '
            'joint_vertical='
            f'{self._enable_joint_vertical_descent}, '
            f'joint_vertical_z_kp={self._joint_vertical_z_kp:.2f}, '
            'joint_vertical_z_step='
            f'{self._joint_vertical_max_z_step * 1000.0:.1f}mm, '
            'joint_vertical_max_joint_step='
            f'{math.degrees(self._joint_vertical_max_joint_step):.1f}deg, '
            'allow_mixed_below_final_z='
            f'{self._allow_mixed_correction_below_final_z}, '
            f'z_backend={self._vertical_z_control_backend}, '
            f'z_cartesian_mode={self._vertical_z_cartesian_mode}, '
            f'z_cartesian_speed={self._vertical_z_cartesian_speed}, '
            f'final_insert={self._enable_final_insertion_z}, '
            'final_insert_distance='
            f'{self._final_insertion_relative_distance * 1000.0:.1f}mm, '
            f'post_recovery_z={self._enable_post_recovery_final_z}, '
            'post_recovery_z_distance='
            f'{self._post_recovery_final_z_distance * 1000.0:.1f}mm, '
            f'initial_yaw={self._enable_initial_yaw}'
        )

    def _validate_parameters(self) -> None:
        if not 1.0 <= self._auto_start_stable_duration <= 60.0:
            raise ValueError(
                'auto_start_stable_duration_seconds는 1~60초여야 합니다'
            )
        if not 5 <= self._auto_start_minimum_samples <= 1000:
            raise ValueError('auto_start_minimum_samples는 5~1000이어야 합니다')
        if not 1.0 <= self._auto_start_center_tolerance <= 500.0:
            raise ValueError('auto_start_center_tolerance_px 범위가 잘못됐습니다')
        if not 0.5 <= self._auto_start_maximum_center_spread <= 100.0:
            raise ValueError(
                'auto_start_maximum_center_spread_px 범위가 잘못됐습니다'
            )
        if not 0.0001 <= self._auto_start_maximum_depth_spread <= 0.100:
            raise ValueError(
                'auto_start_maximum_depth_spread_m 범위가 잘못됐습니다'
            )
        if not 0.1 <= self._auto_start_maximum_yaw_spread <= 45.0:
            raise ValueError(
                'auto_start_maximum_yaw_spread_deg 범위가 잘못됐습니다'
            )
        if not 0.05 <= self._auto_start_maximum_sample_gap <= 5.0:
            raise ValueError(
                'auto_start_maximum_sample_gap_seconds 범위가 잘못됐습니다'
            )
        if not 0.0 < self._maximum_age <= 5.0:
            raise ValueError('maximum_input_age_seconds는 0~5초여야 합니다')
        if not 0.5 <= self._hardware_pose_maximum_age <= 5.0:
            raise ValueError(
                'hardware_cartesian_pose_maximum_age_seconds는 '
                '0.5~5초여야 합니다'
            )
        if not 1 <= self._initial_sample_count <= 30:
            raise ValueError('initial_observation_sample_count는 1~30이어야 합니다')
        if self._initial_aggregation_method not in ('mean', 'median'):
            raise ValueError(
                'initial_observation_aggregation_method는 mean 또는 median이어야 합니다'
            )
        if not 0.1 <= self._initial_sample_window <= 5.0:
            raise ValueError(
                'initial_observation_sample_window_seconds는 0.1~5초여야 합니다'
            )
        if not 0.0005 <= self._initial_maximum_xy_spread <= 0.050:
            raise ValueError(
                'initial_observation_maximum_xy_spread_m 범위가 잘못됐습니다'
            )
        if not 0.0005 <= self._initial_maximum_z_spread <= 0.050:
            raise ValueError(
                'initial_observation_maximum_z_spread_m 범위가 잘못됐습니다'
            )
        if not 0.1 <= self._initial_maximum_yaw_spread <= 45.0:
            raise ValueError(
                'initial_observation_maximum_yaw_spread_deg 범위가 잘못됐습니다'
            )
        if not 0.0 < self._minimum_xy_step <= 0.020:
            raise ValueError('minimum_xy_step_m은 0~0.02m여야 합니다')
        if not self._minimum_xy_step <= self._tracking_tolerance <= 0.050:
            raise ValueError('robot_xy_tracking_tolerance_m 범위가 잘못됐습니다')
        if not self._tracking_tolerance <= self._maximum_coarse_step <= 0.200:
            raise ValueError('maximum_coarse_xy_step_m 범위가 잘못됐습니다')
        if not (
            self._tracking_tolerance
            <= self._maximum_partial_coarse_residual
            <= 0.050
        ):
            raise ValueError(
                'maximum_partial_coarse_xy_residual_m 범위가 잘못됐습니다'
            )
        if not self._minimum_xy_step <= self._refine_maximum_step <= 0.050:
            raise ValueError('robot_xy_refine_maximum_step_m 범위가 잘못됐습니다')
        if not 0 <= self._refine_max_cycles <= 5:
            raise ValueError('robot_xy_refine_max_cycles는 0~5여야 합니다')
        if not 0.0 <= self._minimum_improvement <= 0.020:
            raise ValueError('robot_xy_minimum_improvement_m 범위가 잘못됐습니다')
        if not 0.0 < self._robot_xy_kp <= 1.0:
            raise ValueError('robot_xy_kp는 0보다 크고 1 이하여야 합니다')
        if self._cartesian_mode not in (0, 1):
            raise ValueError('cartesian_mode는 0 또는 1이어야 합니다')
        if not 1 <= self._cartesian_speed <= 100:
            raise ValueError('cartesian_speed는 1~100이어야 합니다')
        if not 1.0 <= self._cartesian_timeout <= 120.0:
            raise ValueError('cartesian_timeout_seconds는 1~120초여야 합니다')
        if not 0.0 <= self._warning_delay <= 10.0:
            raise ValueError('warning_delay_seconds는 0~10초여야 합니다')
        if not 0.0 <= self._settle <= 5.0:
            raise ValueError('settle_seconds는 0~5초여야 합니다')
        if not (
            0.050
            <= self._minimum_target_z
            < self._maximum_target_z
            <= 0.500
        ):
            raise ValueError('minimum/maximum_target_z_m 범위가 잘못됐습니다')
        if not 0.5 <= self._roll_pitch_tolerance <= 15.0:
            raise ValueError('roll_pitch_tolerance_deg는 0.5~15도여야 합니다')
        if not -180.0 <= self._fixed_roll_target <= 180.0:
            raise ValueError('fixed_roll_target_deg는 -180~180도여야 합니다')
        if not -89.0 <= self._fixed_pitch_target <= 89.0:
            raise ValueError('fixed_pitch_target_deg는 -89~89도여야 합니다')
        if not 0.1 <= self._pitch_correction_gain <= 2.0:
            raise ValueError('pitch_correction_gain은 0.1~2.0이어야 합니다')
        if not 0 <= self._roll_pitch_recovery_max_cycles <= 3:
            raise ValueError('roll_pitch_recovery_max_cycles는 0~3이어야 합니다')
        if not 0.0 < self._roll_pitch_kp <= 1.0:
            raise ValueError('roll_pitch_kp는 0보다 크고 1 이하여야 합니다')
        if not 0 <= self._final_coupled_validation_max_cycles <= 5:
            raise ValueError(
                'final_coupled_validation_max_cycles는 0~5여야 합니다'
            )
        if not 0.001 <= self._final_z_descent_step <= 0.020:
            raise ValueError('final_z_descent_step_m은 0.001~0.020m여야 합니다')
        if not 1 <= self._final_z_descent_speed <= 100:
            raise ValueError('final_z_descent_speed는 1~100이어야 합니다')
        if self._final_z_descent_cartesian_mode not in (0, 1):
            raise ValueError('final_z_descent_cartesian_mode는 0 또는 1이어야 합니다')
        if not 1 <= self._vertical_z_cartesian_speed <= 100:
            raise ValueError('vertical_z_cartesian_speed는 1~100이어야 합니다')
        if self._vertical_z_cartesian_mode not in (0, 1):
            raise ValueError('vertical_z_cartesian_mode는 0 또는 1이어야 합니다')
        if self._vertical_z_control_backend not in ('joint', 'cartesian'):
            raise ValueError(
                'vertical_z_control_backend는 joint 또는 cartesian이어야 합니다'
            )
        if not (
            0.00025
            <= self._final_z_descent_minimum_progress
            <= self._final_z_descent_step
        ):
            raise ValueError(
                'final_z_descent_minimum_progress_m 범위가 잘못됐습니다'
            )
        if not 0.050 <= self._final_z_descent_minimum_z <= 0.400:
            raise ValueError(
                'final_z_descent_minimum_z_m은 0.050~0.400m여야 합니다'
            )
        if not 0.0 < self._final_z_p_z_kp <= 1.0:
            raise ValueError('final_z_p_z_kp는 0보다 크고 1 이하여야 합니다')
        if not 0.0005 <= self._final_z_p_maximum_overshoot <= 0.020:
            raise ValueError(
                'final_z_p_maximum_overshoot_m은 0.0005~0.020m여야 합니다'
            )
        if not 0.0005 <= self._final_z_p_maximum_coupled_z_drift <= 0.020:
            raise ValueError(
                'final_z_p_maximum_coupled_z_drift_m은 '
                '0.0005~0.020m여야 합니다'
            )
        if not 0.0 < self._final_z_p_roll_pitch_kp <= 1.0:
            raise ValueError(
                'final_z_p_roll_pitch_kp는 0보다 크고 1 이하여야 합니다'
            )
        if not 0.1 <= self._final_z_p_max_roll_pitch_step <= 5.0:
            raise ValueError(
                'final_z_p_max_roll_pitch_step_deg는 0.1~5도여야 합니다'
            )
        if not (
            0.0
            <= self._final_z_p_roll_pitch_deadband
            < self._final_z_p_roll_pitch_abort
            <= 15.0
        ):
            raise ValueError(
                'final Z P제어 Roll/Pitch deadband/abort 범위가 '
                '잘못됐습니다'
            )
        if self._final_z_p_roll_command_sign not in (-1.0, 1.0):
            raise ValueError('final_z_p_roll_command_sign은 -1 또는 +1이어야 합니다')
        if self._final_z_p_pitch_command_sign not in (-1.0, 1.0):
            raise ValueError('final_z_p_pitch_command_sign은 -1 또는 +1이어야 합니다')
        if not 0.0 <= self._final_z_p_minimum_tilt_improvement <= 2.0:
            raise ValueError(
                'final_z_p_minimum_tilt_improvement_deg는 0~2도여야 합니다'
            )
        if not 0.0 < self._final_z_p_xy_kp <= 1.0:
            raise ValueError('final_z_p_xy_kp는 0보다 크고 1 이하여야 합니다')
        if not 0.0001 <= self._final_z_p_max_xy_step <= 0.005:
            raise ValueError('final_z_p_max_xy_step_m은 0.1~5mm여야 합니다')
        if not 0.0 <= self._final_z_p_xy_deadband <= 0.005:
            raise ValueError('final_z_p_xy_deadband_m은 0~5mm여야 합니다')
        if not 0.0 <= self._final_tcp_offset_z <= 0.300:
            raise ValueError('final_tcp_offset_z_m은 0~0.300m여야 합니다')
        if not 0.0 <= self._final_port_insertion_depth <= 0.050:
            raise ValueError(
                'final_port_insertion_depth_m은 0~0.050m여야 합니다'
            )
        if self._final_port_insertion_depth > self._final_tcp_offset_z:
            raise ValueError(
                '삽입 깊이는 flange-to-tip TCP Z보다 클 수 없습니다'
            )
        if not 0.0001 <= self._final_insertion_step <= 0.001:
            raise ValueError('final_insertion_step_m은 0.1~1.0mm여야 합니다')
        if not 1 <= self._final_insertion_speed <= 10:
            raise ValueError('final_insertion_speed는 1~10이어야 합니다')
        if not (
            self._final_insertion_step
            <= self._final_insertion_maximum_actual_step
            <= 0.003
        ):
            raise ValueError(
                'final_insertion_maximum_actual_step_m은 명령 step 이상, '
                '3mm 이하여야 합니다'
            )
        if not 0.0001 <= self._final_insertion_target_tolerance <= 0.001:
            raise ValueError(
                'final_insertion_target_tolerance_m은 0.1~1.0mm여야 합니다'
            )
        if not (
            self._final_z_descent_minimum_z
            <= self._final_z_p_target_z
            <= self._maximum_target_z
        ):
            raise ValueError(
                'final_z_p_target_z_m은 final Z 하한과 maximum_target_z_m '
                '사이여야 합니다'
            )
        if not 1 <= self._final_z_p_max_cycles <= 20:
            raise ValueError('final_z_p_max_cycles는 1~20이어야 합니다')
        if not 0.0 < self._minimum_yaw_step <= self._maximum_yaw_step <= 30.0:
            raise ValueError('Yaw 최소/최대 step 범위가 잘못됐습니다')
        if not 0.0 <= self._yaw_tolerance <= 5.0:
            raise ValueError('keypoint_tolerance_deg는 0~5도여야 합니다')
        if self._keypoint_command_sign not in (-1.0, 1.0):
            raise ValueError('keypoint_command_sign은 -1 또는 +1이어야 합니다')
        if not 0.1 <= self._keypoint_joint_gain <= 5.0:
            raise ValueError('keypoint_joint_gain은 0.1~5.0이어야 합니다')
        if self._joint6_direction not in (-1.0, 1.0):
            raise ValueError('joint6_direction은 -1 또는 +1이어야 합니다')
        if not 0.0 < self._joint6_limit < 180.0:
            raise ValueError('joint6_limit_deg는 0~180도여야 합니다')
        if not 1.0 <= self._joint6_move_seconds <= 30.0:
            raise ValueError('joint6_move_seconds는 1~30초여야 합니다')
        if not 0.0 < self._joint_vertical_z_kp <= 1.0:
            raise ValueError('joint_vertical_z_kp는 0보다 크고 1 이하여야 합니다')
        if not 0.0005 <= self._joint_vertical_max_z_step <= 0.010:
            raise ValueError(
                'joint_vertical_z_step_m은 0.0005~0.010m여야 합니다'
            )
        if not 0.0 < self._joint_vertical_xy_kp <= 1.0:
            raise ValueError(
                'joint_vertical_xy_kp는 0보다 크고 1 이하여야 합니다'
            )
        if not 0.0 < self._joint_vertical_orientation_kp <= 1.0:
            raise ValueError(
                'joint_vertical_orientation_kp는 0보다 크고 1 이하여야 합니다'
            )
        if not 1e-4 <= self._joint_vertical_damping <= 1.0:
            raise ValueError('joint_vertical_damping 범위가 잘못됐습니다')
        if not 0.001 <= self._joint_vertical_orientation_scale <= 1.0:
            raise ValueError(
                'joint_vertical_orientation_scale_m_per_rad 범위가 잘못됐습니다'
            )
        if not (
            math.radians(0.1)
            <= self._joint_vertical_max_joint_step
            <= math.radians(5.0)
        ):
            raise ValueError(
                'joint_vertical_max_joint_step_deg는 0.1~5도여야 합니다'
            )
        if not (
            0.0
            <= self._joint_vertical_joint_limit_margin
            <= math.radians(15.0)
        ):
            raise ValueError(
                'joint_vertical_joint_limit_margin_deg는 0~15도여야 합니다'
            )
        if not 1.0 <= self._joint_vertical_motion_seconds <= 30.0:
            raise ValueError('joint_vertical_motion_seconds는 1~30초여야 합니다')
        if not 0.0001 <= self._joint_vertical_minimum_progress <= 0.003:
            raise ValueError(
                'joint_vertical_minimum_progress_m 범위가 잘못됐습니다'
            )
        if not 0.0005 <= self._joint_vertical_maximum_overshoot <= 0.010:
            raise ValueError(
                'joint_vertical_maximum_overshoot_m 범위가 잘못됐습니다'
            )
        if not (
            self._joint_vertical_maximum_overshoot
            < self._joint_vertical_hard_maximum_descent
            <= 0.030
        ):
            raise ValueError(
                'joint_vertical_hard_maximum_descent_m은 soft overshoot보다 '
                '크고 0.030m 이하여야 합니다'
            )
        if not (
            0.0
            <= self._joint_vertical_xy_correction_deadband
            <= self._joint_vertical_xy_tolerance
        ):
            raise ValueError(
                'joint_vertical_xy_correction_deadband_m은 0 이상이고 '
                'XY 허용오차 이하여야 합니다'
            )
        if not (
            0.0
            <= self._joint_vertical_roll_pitch_correction_deadband
            <= self._joint_vertical_roll_pitch_tolerance
            < self._joint_vertical_roll_pitch_hard_limit
            <= 20.0
        ):
            raise ValueError(
                'joint_vertical Roll/Pitch deadband/tolerance/hard limit '
                '순서가 잘못됐습니다'
            )
        if not 0.0 <= self._joint_vertical_minimum_tilt_improvement <= 5.0:
            raise ValueError(
                'joint_vertical_minimum_tilt_improvement_deg는 0~5도여야 합니다'
            )
        if not (
            0.0005
            <= self._joint_vertical_maximum_correction_z_drift
            <= 0.020
        ):
            raise ValueError(
                'joint_vertical_maximum_correction_z_drift_m 범위가 '
                '잘못됐습니다'
            )
        if not 0.005 <= self._joint_vertical_final_z_guard <= 0.050:
            raise ValueError(
                'joint_vertical_final_z_guard_m은 0.005~0.050m여야 합니다'
            )
        if not (
            self._joint_vertical_hard_maximum_descent
            < self._joint_vertical_hard_maximum_cycle_descent
            <= 0.050
        ):
            raise ValueError(
                'joint_vertical_hard_maximum_cycle_descent_m은 Z 단계 '
                '하드 한계보다 크고 0.050m 이하여야 합니다'
            )
        if not 1 <= self._joint_vertical_max_cycles <= 20:
            raise ValueError('joint_vertical_max_cycles는 1~20이어야 합니다')
        if not 0.001 <= self._joint_vertical_xy_tolerance <= 0.020:
            raise ValueError('joint_vertical_xy_tolerance_m 범위가 잘못됐습니다')
        if not 1.0 <= self._joint_vertical_roll_pitch_tolerance <= 15.0:
            raise ValueError(
                'joint_vertical_roll_pitch_tolerance_deg는 1~15도여야 합니다'
            )
        if not 0.001 <= self._final_insertion_relative_distance <= 0.030:
            raise ValueError(
                'final_insertion_relative_distance_m은 1~30mm여야 합니다'
            )
        if not (
            self._joint_vertical_minimum_progress
            <= self._final_insertion_tolerance
            <= 0.005
        ):
            raise ValueError(
                'final_insertion_tolerance_m은 관절 최소 진행량 이상 '
                '5mm 이하여야 합니다'
            )
        if not (
            self._final_insertion_relative_distance
            <= self._final_insertion_hard_maximum_total_descent
            <= 0.030
        ):
            raise ValueError(
                'final_insertion_hard_maximum_total_descent_m은 최종 삽입 '
                '거리 이상 30mm 이하여야 합니다'
            )
        if not 1 <= self._post_insertion_roll_pitch_speed <= 100:
            raise ValueError('post_insertion_roll_pitch_speed는 1~100이어야 합니다')
        if not 0.001 <= self._post_recovery_final_z_distance <= 0.010:
            raise ValueError(
                'post_recovery_final_z_distance_m은 1~10mm여야 합니다'
            )

    def _publish(self, text: str) -> None:
        self._status.publish(String(data=text))
        self.get_logger().info(text)

    def _target_callback(self, message: PoseStamped) -> None:
        received_at = time.monotonic()
        with self._lock:
            self._latest_target = message
            self._target_received_at = received_at
            self._target_samples.append(
                (
                    received_at,
                    message.header.frame_id,
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                    float(message.pose.position.z),
                )
            )

    def _hardware_pose_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id == self._base_frame:
            with self._lock:
                self._latest_hardware_pose = message
                self._hardware_pose_received_at = time.monotonic()

    def _port_pose_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id == self._base_frame:
            received_at = time.monotonic()
            with self._lock:
                self._latest_port_pose = message
                self._port_received_at = received_at
                self._port_z_samples.append(
                    (received_at, float(message.pose.position.z))
                )

    def _observation_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id == self._base_frame:
            with self._lock:
                self._observation_reference = message

    def _axis_callback(self, message: Float64) -> None:
        value = float(message.data)
        if math.isfinite(value):
            received_at = time.monotonic()
            with self._lock:
                self._latest_axis = value
                self._axis_received_at = received_at
                self._axis_samples.append((received_at, value))

    def _auto_start_observation_callback(
        self,
        message: UsbPortObservation,
    ) -> None:
        if not self._auto_start_enabled or self._auto_start_triggered:
            return
        if not message.valid or not all(point.visible for point in message.keypoints):
            with self._lock:
                self._auto_start_samples.clear()
            return
        points = np.asarray(
            [(point.x, point.y) for point in message.keypoints],
            dtype=np.float64,
        )
        depth = float(message.depth_m)
        if not np.all(np.isfinite(points)) or not math.isfinite(depth):
            with self._lock:
                self._auto_start_samples.clear()
            return
        long_axis = points[1] - points[0]
        axis_deg = math.degrees(
            math.atan2(float(long_axis[1]), float(long_axis[0]))
        )
        center = np.mean(points, axis=0)
        sample = AutoStartSample(
            timestamp=time.monotonic(),
            center_u=float(center[0]),
            center_v=float(center[1]),
            depth_m=depth,
            axis_deg=axis_deg,
        )
        with self._lock:
            self._auto_start_samples.append(sample)

    def _auto_start_timer_callback(self) -> None:
        if not self._auto_start_enabled:
            return
        with self._lock:
            if self._auto_start_triggered or self._executing:
                return
            samples = list(self._auto_start_samples)
            observation_ready = self._observation_reference is not None
        if not observation_ready:
            reason = '초기 관측 자세 확인 대기'
            if reason != self._auto_start_last_reason:
                self._auto_start_last_reason = reason
                self._publish('AUTO_START_WAIT: ' + reason)
            return
        result = evaluate_auto_start(
            samples,
            now=time.monotonic(),
            stable_duration_seconds=self._auto_start_stable_duration,
            minimum_samples=self._auto_start_minimum_samples,
            image_center_u_px=self._auto_start_image_center_u,
            image_center_v_px=self._auto_start_image_center_v,
            center_tolerance_px=self._auto_start_center_tolerance,
            maximum_center_spread_px=(
                self._auto_start_maximum_center_spread
            ),
            maximum_depth_spread_m=self._auto_start_maximum_depth_spread,
            maximum_yaw_spread_deg=self._auto_start_maximum_yaw_spread,
            maximum_sample_gap_seconds=self._auto_start_maximum_sample_gap,
        )
        if not result.ready:
            if result.reason != self._auto_start_last_reason:
                self._auto_start_last_reason = result.reason
                self._publish(
                    'AUTO_START_WAIT: '
                    f'{result.reason}, samples={result.sample_count}, '
                    f'duration={result.duration_seconds:.1f}s, '
                    f'center_error={result.center_error_px:.1f}px, '
                    f'center_spread={result.center_spread_px:.1f}px, '
                    f'depth_spread={result.depth_spread_m * 1000.0:.1f}mm, '
                    f'yaw_spread={result.yaw_spread_deg:.1f}deg'
                )
            return
        with self._lock:
            if self._auto_start_triggered or self._executing:
                return
            self._auto_start_triggered = True
            self._executing = True
        self._publish(
            'AUTO_START_TRIGGERED: 포트 중앙·정지 관측 완료, '
            f'samples={result.sample_count}, '
            f'duration={result.duration_seconds:.1f}s, '
            f'center_error={result.center_error_px:.1f}px, '
            f'center_spread={result.center_spread_px:.1f}px, '
            f'depth_spread={result.depth_spread_m * 1000.0:.1f}mm, '
            f'yaw_spread={result.yaw_spread_deg:.1f}deg. '
            '최종 Z 포함 통합 제어를 한 번 자동 실행합니다'
        )
        threading.Thread(
            target=self._auto_start_worker,
            name='pinkk-auto-start',
            daemon=True,
        ).start()

    def _auto_start_worker(self) -> None:
        try:
            if not self._enabled:
                raise ValueError('enable_execution=false')
            self._execute_full_sequence(include_final_insertion=True)
        except Exception as error:
            self._publish(f'REJECTED: 자동 통합 실행 실패: {error}')
        finally:
            with self._lock:
                self._executing = False

    def _joint_callback(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINT_NAMES):
            with self._lock:
                self._latest_joints = [
                    float(values[name]) for name in JOINT_NAMES
                ]
                self._joints_received_at = time.monotonic()

    def _fresh(self, received_at: float | None, label: str) -> None:
        if received_at is None:
            raise ValueError(f'{label} 입력이 없습니다')
        age = time.monotonic() - received_at
        if not 0.0 <= age <= self._maximum_age:
            raise ValueError(f'{label} 입력이 오래됐습니다: age={age:.3f}s')

    def _averaged_initial_perception(self):
        """최근 PBVS/포트/Yaw 표본을 검증하고 평균 관측을 반환한다."""
        now = time.monotonic()
        cutoff = now - self._initial_sample_window
        with self._lock:
            latest_target = deepcopy(self._latest_target)
            latest_port = deepcopy(self._latest_port_pose)
            target_samples = [
                sample for sample in self._target_samples if sample[0] >= cutoff
            ]
            port_samples = [
                sample for sample in self._port_z_samples if sample[0] >= cutoff
            ]
            axis_samples = [
                sample for sample in self._axis_samples if sample[0] >= cutoff
            ]

        required = self._initial_sample_count
        required_inputs = (
            ('PBVS target', target_samples),
            ('keypoint Yaw', axis_samples),
        )
        for label, samples in required_inputs:
            if len(samples) < required:
                raise ValueError(
                    f'초기 {label} 평균 표본이 부족합니다: '
                    f'{len(samples)}/{required}, '
                    f'window={self._initial_sample_window:.1f}s'
                )
        if self._final_z_p_use_port_target and len(port_samples) < required:
            raise ValueError(
                '초기 port_pose 평균 표본이 부족합니다: '
                f'{len(port_samples)}/{required}, '
                f'window={self._initial_sample_window:.1f}s'
            )
        if latest_target is None:
            raise ValueError('초기 PBVS target 입력이 없습니다')
        if self._final_z_p_use_port_target and latest_port is None:
            raise ValueError('초기 port_pose_base 입력이 없습니다')

        target_samples = target_samples[-required:]
        axis_samples = axis_samples[-required:]
        target_frames = {sample[1] for sample in target_samples}
        if target_frames != {self._base_frame}:
            raise ValueError(
                f'PBVS target 표본 frame이 {self._base_frame}와 다릅니다: '
                f'{sorted(target_frames)}'
            )
        target_xyz = np.asarray(
            [sample[2:5] for sample in target_samples], dtype=np.float64
        )
        if not np.all(np.isfinite(target_xyz)):
            raise ValueError('초기 PBVS target 표본에 유한하지 않은 값이 있습니다')
        if self._initial_aggregation_method == 'median':
            center_target_xyz = np.median(target_xyz, axis=0)
        else:
            center_target_xyz = np.mean(target_xyz, axis=0)
        xy_spread = float(
            np.max(
                np.linalg.norm(
                    target_xyz[:, :2] - center_target_xyz[:2], axis=1
                )
            )
        )
        target_z_spread = float(
            np.max(np.abs(target_xyz[:, 2] - center_target_xyz[2]))
        )
        if xy_spread > self._initial_maximum_xy_spread:
            raise ValueError(
                '초기 PBVS XY 관측 편차가 큽니다: '
                f'spread={xy_spread * 1000.0:.1f}mm, '
                f'limit={self._initial_maximum_xy_spread * 1000.0:.1f}mm'
            )
        if target_z_spread > self._initial_maximum_z_spread:
            raise ValueError(
                '초기 PBVS target Z 관측 편차가 큽니다: '
                f'spread={target_z_spread * 1000.0:.1f}mm, '
                f'limit={self._initial_maximum_z_spread * 1000.0:.1f}mm'
            )

        axis_values = [sample[1] for sample in axis_samples]
        if self._initial_aggregation_method == 'median':
            center_axis = circular_median_degrees(axis_values)
        else:
            center_axis = circular_mean_degrees(axis_values)
        yaw_spread = maximum_angular_deviation_degrees(
            axis_values, center_axis
        )
        if yaw_spread > self._initial_maximum_yaw_spread:
            raise ValueError(
                '초기 keypoint Yaw 관측 편차가 큽니다: '
                f'spread={yaw_spread:.2f}deg, '
                f'limit={self._initial_maximum_yaw_spread:.2f}deg'
            )

        latest_target.pose.position.x = float(center_target_xyz[0])
        latest_target.pose.position.y = float(center_target_xyz[1])
        latest_target.pose.position.z = float(center_target_xyz[2])
        port_z_spread = 0.0
        if self._final_z_p_use_port_target:
            port_samples = port_samples[-required:]
            port_z_values = np.asarray(
                [sample[1] for sample in port_samples], dtype=np.float64
            )
            if not np.all(np.isfinite(port_z_values)):
                raise ValueError('초기 port Z 표본에 유한하지 않은 값이 있습니다')
            if self._initial_aggregation_method == 'median':
                center_port_z = float(np.median(port_z_values))
            else:
                center_port_z = float(np.mean(port_z_values))
            port_z_spread = float(
                np.max(np.abs(port_z_values - center_port_z))
            )
            if port_z_spread > self._initial_maximum_z_spread:
                raise ValueError(
                    '초기 port Z 관측 편차가 큽니다: '
                    f'spread={port_z_spread * 1000.0:.1f}mm, '
                    f'limit={self._initial_maximum_z_spread * 1000.0:.1f}mm'
                )
            latest_port.pose.position.z = center_port_z

        summary = (
            f'samples={required}, aggregation={self._initial_aggregation_method}, '
            f'xy_spread={xy_spread * 1000.0:.1f}mm, '
            f'target_z_spread={target_z_spread * 1000.0:.1f}mm, '
            f'port_z_spread={port_z_spread * 1000.0:.1f}mm, '
            f'yaw_spread={yaw_spread:.2f}deg'
        )
        return latest_target, latest_port, float(center_axis), summary

    def _command_callback(self, message: String) -> None:
        command = message.data.strip().lower()
        if command not in (
            'execute_once',
            'yaw_only_once',
            'descend_joint_z_once',
            'descend_joint_z_to_guard',
            'insert_step_once',
            'insert_final_z_once',
            'execute_full_sequence',
            'execute_full_sequence_with_final_z',
        ):
            self._publish(
                'REJECTED: 허용된 명령은 execute_once, yaw_only_once, '
                'descend_joint_z_once, descend_joint_z_to_guard, '
                'insert_final_z_once, execute_full_sequence, '
                'execute_full_sequence_with_final_z, insert_step_once입니다'
            )
            return
        with self._lock:
            if self._executing:
                self._publish('REJECTED: 이전 이동이 실행 중입니다')
                return
            self._executing = True
            # 수동 명령과 자동 시작이 같은 세션에서 연속 실행되지 않도록
            # 어떤 수동 이동 명령이든 받으면 자동 one-shot을 소모한다.
            if self._auto_start_enabled:
                self._auto_start_triggered = True
                self._auto_start_samples.clear()
        try:
            if not self._enabled:
                raise ValueError('enable_execution=false')
            if command == 'yaw_only_once':
                self._execute_yaw_only_once()
            elif command == 'descend_joint_z_once':
                self._execute_descend_joint_z_once()
            elif command == 'descend_joint_z_to_guard':
                self._execute_descend_joint_z_to_guard()
            elif command == 'insert_step_once':
                self._execute_insert_step_once()
            elif command == 'insert_final_z_once':
                self._execute_final_insertion_z()
            elif command == 'execute_full_sequence':
                self._execute_full_sequence()
            elif command == 'execute_full_sequence_with_final_z':
                self._execute_full_sequence(include_final_insertion=True)
            else:
                self._execute_once()
        except Exception as error:
            self._publish(f'REJECTED: {error}')
        finally:
            with self._lock:
                self._executing = False

    def _execute_yaw_only_once(self) -> None:
        with self._lock:
            initial_axis = self._latest_axis
            axis_received_at = self._axis_received_at
        self._fresh(axis_received_at, 'keypoint angle')
        if initial_axis is None:
            raise ValueError('Yaw 시험용 keypoint angle이 없습니다')
        self._publish(
            'Yaw 단독 시험 시작: '
            f'keypoint_axis={initial_axis:+.3f}deg, '
            f'maximum_joint_step={self._maximum_yaw_step:.1f}deg'
        )
        self._execute_initial_yaw(float(initial_axis))
        self._publish(
            'EXECUTED: yaw_only_once 완료. 새 영상각을 직접 확인하세요'
        )

    def _roll_pitch_reference(self, transform):
        """선택에 따라 관측 R/P 또는 고정 R/P를 갖는 기준 pose를 반환한다."""
        reference = np.asarray(transform, dtype=np.float64).copy()
        if not self._use_fixed_roll_pitch_target:
            return reference
        _, _, yaw = rotation_to_rpy_degrees(reference[:3, :3])
        reference[:3, :3] = rpy_degrees_to_rotation(
            self._fixed_roll_target,
            self._fixed_pitch_target,
            yaw,
        )
        return reference

    def _execute_once(self) -> None:
        self._alignment_ready_for_descent = False
        self._insertion_ready = False
        self._aligned_frozen_xy = None
        self._aligned_observation_transform = None
        self._aligned_port_z = None
        self._aligned_final_target_z = None
        self._aligned_hardware_xy = None
        self._aligned_hardware_observation_transform = None
        self._aligned_hardware_final_target_z = None
        self._aligned_hardware_yaw_deg = None
        self._final_insertion_ready = False
        with self._lock:
            observation = deepcopy(self._observation_reference)
        (
            initial_target,
            port_pose,
            initial_axis,
            sample_summary,
        ) = self._averaged_initial_perception()
        if (
            initial_target is None
            or observation is None
            or initial_axis is None
            or (self._final_z_p_use_port_target and port_pose is None)
        ):
            raise ValueError(
                '초기 PBVS/포트/관측 기준/Yaw 입력이 모두 필요합니다'
            )
        if initial_target.header.frame_id != self._base_frame:
            raise ValueError(f'PBVS target frame이 {self._base_frame}가 아닙니다')

        port_z = None
        if self._final_z_p_use_port_target:
            if port_pose.header.frame_id != self._base_frame:
                raise ValueError(
                    f'port_pose_base frame이 {self._base_frame}가 아닙니다'
                )
            port_z = float(port_pose.pose.position.z)
            final_target_z = port_based_flange_target_z(
                port_z,
                self._final_tcp_offset_z,
                self._final_port_insertion_depth,
            )
            final_z_source = 'frozen_port_z+tcp_z-insertion_depth'
        else:
            final_target_z = self._final_z_p_target_z
            final_z_source = 'manual_yaml'
        if not math.isfinite(final_target_z):
            raise ValueError('계산된 최종 Z가 유한값이 아닙니다')
        if not (
            self._final_z_descent_minimum_z
            <= final_target_z
            <= self._maximum_target_z
        ):
            raise ValueError(
                f'계산된 최종 Z {final_target_z * 1000.0:.1f}mm가 '
                f'허용범위 {self._final_z_descent_minimum_z * 1000.0:.1f}~'
                f'{self._maximum_target_z * 1000.0:.1f}mm 밖입니다'
            )
        hardware_observation_transform = self._roll_pitch_reference(
            self._current_hardware_transform()
        )

        initial_target_transform = pose_to_transform(initial_target.pose)
        observation_transform = self._roll_pitch_reference(
            pose_to_transform(observation.pose)
        )
        current = self._current_flange()
        frozen_target = current.copy()
        frozen_target[:2, 3] = initial_target_transform[:2, 3]
        if self._use_pbvs_target_z:
            frozen_target[2, 3] = initial_target_transform[2, 3]
            z_source = 'pbvs_pre_approach'
        else:
            frozen_target[2, 3] = observation_transform[2, 3]
            z_source = 'initial_observation'
        if not (
            self._minimum_target_z
            <= frozen_target[2, 3]
            <= self._maximum_target_z
        ):
            raise ValueError(
                f'목표 Z {frozen_target[2, 3] * 1000.0:.1f}mm가 허용범위 '
                f'{self._minimum_target_z * 1000.0:.1f}~'
                f'{self._maximum_target_z * 1000.0:.1f}mm 밖입니다'
            )
        frozen_target[:3, :3] = (
            apply_observation_roll_pitch_with_current_yaw(
                current,
                observation_transform,
            )[:3, :3]
        )
        frozen_xy = frozen_target[:2, 3].copy()
        initial_residual = xy_residual_m(current[:2, 3], frozen_xy)
        if initial_residual > self._maximum_coarse_step:
            raise ValueError(
                f'초기 XY 이동 {initial_residual * 1000.0:.1f}mm가 제한 '
                f'{self._maximum_coarse_step * 1000.0:.1f}mm를 초과합니다'
            )

        self._publish(
            '초기 관측 목표 고정: '
            f'xy=[{frozen_xy[0]:+.6f}, {frozen_xy[1]:+.6f}]m, '
            f'z={frozen_target[2, 3]:.6f}m({z_source}), '
            f'keypoint_axis={initial_axis:+.3f}deg, '
            f'{sample_summary}. '
            '이후 카메라 관측은 제어에 사용하지 않습니다'
        )
        reference_rpy = rotation_to_rpy_degrees(
            observation_transform[:3, :3]
        )
        self._publish(
            'Roll/Pitch 기준 고정: '
            f'source={"fixed_yaml" if self._use_fixed_roll_pitch_target else "initial_observation"}, '
            f'target_rp=[{reference_rpy[0]:+.2f}, '
            f'{reference_rpy[1]:+.2f}]deg'
        )
        self._publish(
            '최종 삽입 Z 고정: '
            f'port_z={port_z * 1000.0:.1f}mm, '
            f'tcp_z={self._final_tcp_offset_z * 1000.0:.1f}mm, '
            'insertion_depth='
            f'{self._final_port_insertion_depth * 1000.0:.1f}mm, '
            f'target_flange_z={final_target_z * 1000.0:.1f}mm, '
            f'source={final_z_source}'
            if port_z is not None
            else (
                '최종 삽입 Z 고정: '
                f'target_flange_z={final_target_z * 1000.0:.1f}mm, '
                f'source={final_z_source}'
            )
        )

        previous_residual = initial_residual
        if initial_residual >= self._minimum_xy_step:
            self._send_cartesian_transform(
                frozen_target,
                'coarse',
                accept_timeout_xy_residual_m=(
                    self._maximum_partial_coarse_residual
                ),
            )
            time.sleep(self._settle)
        self._recover_roll_pitch(observation_transform, 'coarse 후')
        residual = xy_residual_m(self._current_flange()[:2, 3], frozen_xy)
        improvement = previous_residual - residual
        self._publish(
            f'coarse 후 로봇좌표 XY 잔여오차={residual * 1000.0:.1f}mm, '
            f'improvement={improvement * 1000.0:+.1f}mm'
        )

        residual = self._refine_xy_to_frozen_target(
            frozen_xy,
            observation_transform,
            phase='robot_xy_refine',
            initial_improvement=improvement,
        )

        self._publish(
            f'로봇좌표 XY 수렴: residual={residual * 1000.0:.1f}mm. '
            '저장한 초기 관측 Yaw를 적용합니다'
        )
        if self._enable_initial_yaw:
            self._execute_initial_yaw(float(initial_axis))
        residual = self._finalize_coupled_alignment(
            frozen_xy,
            observation_transform,
            context='Yaw 후 최종 정렬',
        )
        self._aligned_frozen_xy = frozen_xy.copy()
        self._aligned_observation_transform = observation_transform.copy()
        self._aligned_port_z = port_z
        self._aligned_final_target_z = final_target_z
        # Joint6/Cartesian 정렬 직후 브리지의 2Hz get_coords pose가 한 번
        # 갱신될 시간을 보장한 뒤 삽입 기준을 저장한다.
        time.sleep(max(self._settle, 0.6))
        hardware_aligned = self._current_hardware_transform()
        tf_aligned = self._current_flange()
        remaining_descent = float(tf_aligned[2, 3] - final_target_z)
        if remaining_descent < 0.0:
            raise RuntimeError(
                '정렬 완료 flange Z가 포트 기반 최종 Z보다 이미 낮습니다: '
                f'current_tf_z={tf_aligned[2, 3] * 1000.0:.1f}mm, '
                f'final_tf_z={final_target_z * 1000.0:.1f}mm'
            )
        self._aligned_hardware_xy = hardware_aligned[:2, 3].copy()
        self._aligned_hardware_observation_transform = (
            hardware_observation_transform.copy()
        )
        self._aligned_hardware_final_target_z = (
            float(hardware_aligned[2, 3]) - remaining_descent
        )
        self._aligned_hardware_yaw_deg = float(
            rotation_to_rpy_degrees(hardware_aligned[:3, :3])[2]
        )
        self._alignment_ready_for_descent = True
        hardware_observation_rpy = rotation_to_rpy_degrees(
            hardware_observation_transform[:3, :3]
        )
        self._publish(
            '삽입 제어 제조사 좌표 기준 저장: '
            f'hardware_xy=[{hardware_aligned[0, 3] * 1000.0:+.1f}, '
            f'{hardware_aligned[1, 3] * 1000.0:+.1f}]mm, '
            f'hardware_start_z={hardware_aligned[2, 3] * 1000.0:.1f}mm, '
            'initial_observation_rp='
            f'[{hardware_observation_rpy[0]:+.2f}, '
            f'{hardware_observation_rpy[1]:+.2f}]deg, '
            f'remaining_descent={remaining_descent * 1000.0:.1f}mm, '
            'hardware_final_z='
            f'{self._aligned_hardware_final_target_z * 1000.0:.1f}mm'
        )
        self._publish(
            'EXECUTED: 초기 관측 고정목표 XY/Yaw 완료. '
            '이동 후 카메라 재관측은 사용하지 않았습니다. '
            'descend_joint_z_once 또는 descend_joint_z_to_guard 승인 대기'
        )

    def _execute_descend_z_once(self) -> None:
        if not self._enable_final_z_descent:
            raise ValueError('enable_final_z_descent=false')
        if self._cartesian_mode != 1:
            raise ValueError('Z 선형 하강에는 cartesian_mode=1이 필요합니다')
        if (
            not self._alignment_ready_for_descent
            or self._aligned_frozen_xy is None
            or self._aligned_observation_transform is None
            or self._aligned_final_target_z is None
        ):
            raise ValueError('성공한 execute_once 정렬 결과가 먼저 필요합니다')
        self._alignment_ready_for_descent = False
        current = self._current_flange()
        current_z = float(current[2, 3])
        target_z = max(
            current_z - self._final_z_descent_step,
            self._aligned_final_target_z,
            self._final_z_descent_minimum_z,
        )
        dz = current_z - target_z
        if dz < 0.0005:
            raise ValueError(
                'Z 하강 하한에 도달했습니다: '
                f'current_z={current_z * 1000.0:.1f}mm, '
                f'minimum_z={self._final_z_descent_minimum_z * 1000.0:.1f}mm'
            )
        descent_target = current.copy()
        descent_target[2, 3] = target_z
        self._publish(
            '최종 정렬 자세에서 Z 선형 하강 승인: '
            f'dz=-{dz * 1000.0:.1f}mm, '
            f'target_z={target_z * 1000.0:.1f}mm'
        )
        self._send_cartesian_transform(
            descent_target,
            'final_z_descent',
            lock_z=False,
            lock_roll_pitch=True,
            speed=self._final_z_descent_speed,
        )
        time.sleep(self._settle)
        actual = self._current_flange()
        actual_descent = current_z - float(actual[2, 3])
        residual = xy_residual_m(
            actual[:2, 3],
            self._aligned_frozen_xy,
        )
        roll_error, pitch_error = self._roll_pitch_errors(
            actual,
            self._aligned_observation_transform,
        )
        self._publish(
            'Z 하강 후 검사만 수행: '
            f'actual_dz=-{actual_descent * 1000.0:.1f}mm, '
            f'xy={residual * 1000.0:.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg. '
            '삽입 중 횡방향/자세 재보정은 보내지 않습니다'
        )
        if actual_descent < self._final_z_descent_minimum_progress:
            raise RuntimeError(
                'Z 하강 실제 진행량이 부족합니다: '
                f'actual={actual_descent * 1000.0:.1f}mm, '
                f'minimum='
                f'{self._final_z_descent_minimum_progress * 1000.0:.1f}mm'
            )
        if residual > self._tracking_tolerance:
            raise RuntimeError(
                'Z 하강 중 XY가 허용오차를 벗어났습니다. '
                '횡방향 자동 보정 없이 다음 하강을 차단합니다: '
                f'xy={residual * 1000.0:.1f}mm, '
                f'limit={self._tracking_tolerance * 1000.0:.1f}mm'
            )
        if max(roll_error, pitch_error) > self._roll_pitch_tolerance:
            raise RuntimeError(
                'Z 하강 중 Roll/Pitch가 허용오차를 벗어났습니다. '
                '자세 자동 보정 없이 다음 하강을 차단합니다: '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg, '
                f'limit={self._roll_pitch_tolerance:.2f}deg'
            )
        self._alignment_ready_for_descent = True
        self._publish(
            'EXECUTED: Z 선형 하강 1단계 완료: '
            f'target_z={target_z * 1000.0:.1f}mm, '
            f'xy_residual={residual * 1000.0:.1f}mm. '
            '다음 descend_z_once 승인 대기'
        )

    def _execute_descend_z_p_once(self) -> None:
        """Z 하강, XY P보정, 초기 Roll/Pitch P보정을 순서대로 수행한다."""
        if not self._enable_final_z_descent:
            raise ValueError('enable_final_z_descent=false')
        if not self._enable_final_z_p_descent:
            raise ValueError('enable_final_z_p_descent=false')
        if (
            not self._alignment_ready_for_descent
            or self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
        ):
            raise ValueError('성공한 execute_once 정렬 결과가 먼저 필요합니다')

        self._alignment_ready_for_descent = False
        current = self._current_hardware_transform()
        current_z = float(current[2, 3])
        remaining_z = current_z - self._aligned_hardware_final_target_z
        if remaining_z <= self._joint_vertical_final_z_guard:
            self._alignment_ready_for_descent = False
            self._publish(
                'EXECUTED: 포트 기반 최종 Z 안전 여유 구간에 도달해 관절 '
                'Z 사이클을 시작하지 않습니다: '
                f'remaining={remaining_z * 1000.0:.1f}mm, '
                f'guard={self._joint_vertical_final_z_guard * 1000.0:.1f}mm. '
                '마지막 삽입은 별도 저속/접촉 감시 단계가 필요합니다'
            )
            return
        dz = proportional_z_descent_m(
            remaining_z,
            self._final_z_p_z_kp,
            self._final_z_descent_step,
            self._final_z_descent_minimum_progress,
        )
        if dz < self._final_z_descent_minimum_progress:
            raise ValueError(
                '포트 기반 최종 Z에 도달했거나 유효 하강량이 남지 않았습니다: '
                f'remaining_dz={dz * 1000.0:.1f}mm, '
                'hardware_target_z='
                f'{self._aligned_hardware_final_target_z * 1000.0:.1f}mm'
            )
        target_z = current_z - dz

        self._publish(
            'Z 순차 P제어 1/3: 먼저 현재 XY·자세를 유지한 채 '
            f'remaining={remaining_z * 1000.0:.1f}mm, '
            f'kp={self._final_z_p_z_kp:.2f}, '
            f'dz_command=-{dz * 1000.0:.1f}mm, '
            f'target_z={target_z * 1000.0:.1f}mm'
        )
        z_target = current.copy()
        z_target[2, 3] = target_z
        z_result = self._send_cartesian_transform(
            z_target,
            'staged_z_descent',
            lock_z=False,
            lock_roll_pitch=True,
            speed=self._final_z_descent_speed,
            mode=self._final_z_descent_cartesian_mode,
        )
        time.sleep(self._settle)

        after_z = pose_to_transform(z_result.pose)
        actual_descent = current_z - float(after_z[2, 3])
        if actual_descent < self._final_z_descent_minimum_progress:
            raise RuntimeError(
                'Z 순차 P제어 하강 실제 진행량이 부족합니다: '
                f'actual={actual_descent * 1000.0:.1f}mm, '
                f'minimum='
                f'{self._final_z_descent_minimum_progress * 1000.0:.1f}mm'
            )
        descent_overshoot = actual_descent - dz
        if descent_overshoot > self._final_z_p_maximum_overshoot:
            raise RuntimeError(
                'Z P제어 실제 하강량이 명령을 과도하게 초과하여 '
                'XY/Roll/Pitch 보정을 시작하지 않습니다: '
                f'command={dz * 1000.0:.1f}mm, '
                f'actual={actual_descent * 1000.0:.1f}mm, '
                f'overshoot={descent_overshoot * 1000.0:.1f}mm, '
                f'limit={self._final_z_p_maximum_overshoot * 1000.0:.1f}mm'
            )
        if (
            float(after_z[2, 3])
            < self._aligned_hardware_final_target_z
            - self._final_z_p_maximum_overshoot
        ):
            raise RuntimeError(
                'Z P제어가 포트 기반 최종 목표 아래로 초과 하강했습니다: '
                f'actual_z={after_z[2, 3] * 1000.0:.1f}mm, '
                'target_z='
                f'{self._aligned_hardware_final_target_z * 1000.0:.1f}mm'
            )
        after_z_roll, after_z_pitch = self._roll_pitch_errors(
            after_z,
            self._aligned_hardware_observation_transform,
        )
        if max(after_z_roll, after_z_pitch) > self._final_z_p_roll_pitch_abort:
            raise RuntimeError(
                'Z 하강 직후 Roll/Pitch 하드 안전한계 초과로 보정을 '
                '시작하지 않습니다: '
                f'roll={after_z_roll:.2f}deg, '
                f'pitch={after_z_pitch:.2f}deg, '
                f'limit={self._final_z_p_roll_pitch_abort:.2f}deg'
            )

        xy_error = xy_residual_m(
            after_z[:2, 3], self._aligned_hardware_xy
        )
        self._publish(
            'Z 순차 P제어 1/3 완료: '
            f'actual_dz=-{actual_descent * 1000.0:.1f}mm, '
            f'xy_error={xy_error * 1000.0:.1f}mm, '
            f'roll={after_z_roll:.2f}deg, pitch={after_z_pitch:.2f}deg'
        )

        after_xy = after_z
        if xy_error > self._final_z_p_xy_deadband:
            desired_hardware_xy, _ = proportional_xy_target(
                after_z[:2, 3],
                self._aligned_hardware_xy,
                self._final_z_p_xy_kp,
                self._final_z_p_max_xy_step,
                self._minimum_xy_step,
            )
            xy_target = after_z.copy()
            xy_target[:2, 3] = desired_hardware_xy
            xy_command = xy_residual_m(
                after_z[:2, 3], xy_target[:2, 3]
            )
            self._publish(
                'Z 순차 P제어 2/3: '
                f'xy_error={xy_error * 1000.0:.1f}mm, '
                f'xy_command={xy_command * 1000.0:.1f}mm'
            )
            xy_result = self._send_cartesian_transform(
                xy_target,
                'staged_xy_p_correction',
                lock_z=True,
                lock_roll_pitch=True,
                speed=self._final_z_descent_speed,
            )
            time.sleep(self._settle)
            after_xy = pose_to_transform(xy_result.pose)
        xy_after_correction = xy_residual_m(
            after_xy[:2, 3], self._aligned_hardware_xy
        )
        self._publish(
            'Z 순차 P제어 2/3 완료: '
            f'xy_error={xy_after_correction * 1000.0:.1f}mm'
        )

        current_rpy = rotation_to_rpy_degrees(after_xy[:3, :3])
        reference_rpy = rotation_to_rpy_degrees(
            self._aligned_hardware_observation_transform[:3, :3]
        )
        signed_roll_error = self._wrapped_error_deg(
            reference_rpy[0], current_rpy[0]
        )
        signed_pitch_error = self._wrapped_error_deg(
            reference_rpy[1], current_rpy[1]
        )
        maximum_tilt_error = max(
            abs(signed_roll_error), abs(signed_pitch_error)
        )
        if maximum_tilt_error > self._final_z_p_roll_pitch_abort:
            raise RuntimeError(
                'XY 보정 후 Roll/Pitch 안전범위 초과: '
                f'roll_error={signed_roll_error:+.2f}deg, '
                f'pitch_error={signed_pitch_error:+.2f}deg, '
                f'limit={self._final_z_p_roll_pitch_abort:.2f}deg'
            )

        effective_rp_gain = 0.0
        after_rp = after_xy
        if maximum_tilt_error > self._final_z_p_roll_pitch_deadband:
            effective_rp_gain = min(
                self._final_z_p_roll_pitch_kp,
                self._final_z_p_max_roll_pitch_step
                / maximum_tilt_error,
            )
            roll_command = (
                self._final_z_p_roll_command_sign
                * effective_rp_gain
                * signed_roll_error
            )
            pitch_command = (
                self._final_z_p_pitch_command_sign
                * effective_rp_gain
                * signed_pitch_error
            )
            rp_target = after_xy.copy()
            rp_target[:3, :3] = rpy_degrees_to_rotation(
                current_rpy[0] + roll_command,
                current_rpy[1] + pitch_command,
                current_rpy[2],
            )
            self._publish(
                'Z 순차 P제어 3/3: '
                'initial_target_rp='
                f'[{reference_rpy[0]:+.2f}, {reference_rpy[1]:+.2f}]deg, '
                'current_rp='
                f'[{current_rpy[0]:+.2f}, {current_rpy[1]:+.2f}]deg, '
                f'roll_error={signed_roll_error:+.2f}deg, '
                f'pitch_error={signed_pitch_error:+.2f}deg, '
                f'roll_command={roll_command:+.2f}deg, '
                f'pitch_command={pitch_command:+.2f}deg, '
                f'effective_gain={effective_rp_gain:.3f}'
            )
            rp_result = self._send_cartesian_transform(
                rp_target,
                'staged_roll_pitch_p_correction',
                lock_z=True,
                # P 보정 자세가 시작 자세로 덮어써지지 않게 한다.
                lock_roll_pitch=False,
                speed=self._final_z_descent_speed,
            )
            time.sleep(self._settle)
            after_rp = pose_to_transform(rp_result.pose)

        coupled_z_drift = abs(float(after_rp[2, 3] - after_xy[2, 3]))
        if coupled_z_drift > self._final_z_p_maximum_coupled_z_drift:
            raise RuntimeError(
                'Roll/Pitch 보정 중 Z 결합 이동이 허용값을 초과했습니다: '
                f'z_drift={coupled_z_drift * 1000.0:.1f}mm, '
                'limit='
                f'{self._final_z_p_maximum_coupled_z_drift * 1000.0:.1f}mm'
            )

        residual = xy_residual_m(
            after_rp[:2, 3], self._aligned_hardware_xy
        )
        final_roll_error, final_pitch_error = self._roll_pitch_errors(
            after_rp,
            self._aligned_hardware_observation_transform,
        )
        final_maximum_tilt_error = max(
            final_roll_error, final_pitch_error
        )
        tilt_improvement = maximum_tilt_error - final_maximum_tilt_error
        self._publish(
            'Z 순차 P제어 3/3 완료 및 사이클 검사: '
            f'xy={residual * 1000.0:.1f}mm, '
            f'roll={final_roll_error:.2f}deg, '
            f'pitch={final_pitch_error:.2f}deg, '
            f'rp_coupled_z_drift={coupled_z_drift * 1000.0:.1f}mm, '
            f'tilt_improvement={tilt_improvement:+.2f}deg'
        )
        if residual > self._tracking_tolerance:
            raise RuntimeError(
                'Roll/Pitch 보정 후 XY가 허용오차를 벗어나 다음 Z 하강을 '
                '차단합니다: '
                f'xy={residual * 1000.0:.1f}mm, '
                f'limit={self._tracking_tolerance * 1000.0:.1f}mm'
            )
        if (
            effective_rp_gain > 0.0
            and tilt_improvement
            < self._final_z_p_minimum_tilt_improvement
        ):
            raise RuntimeError(
                'Roll/Pitch P보정 후 오차가 충분히 감소하지 않아 다음 Z '
                '하강을 차단합니다: '
                f'improvement={tilt_improvement:+.2f}deg, '
                f'minimum='
                f'{self._final_z_p_minimum_tilt_improvement:.2f}deg'
            )
        if (
            final_maximum_tilt_error
            > self._final_z_p_roll_pitch_abort
        ):
            raise RuntimeError(
                'Z P제어 하강 후 Roll/Pitch 안전범위 초과로 '
                '다음 하강을 차단합니다: '
                f'roll={final_roll_error:.2f}deg, '
                f'pitch={final_pitch_error:.2f}deg, '
                f'limit={self._final_z_p_roll_pitch_abort:.2f}deg'
            )
        if final_maximum_tilt_error > self._roll_pitch_tolerance:
            raise RuntimeError(
                'Roll/Pitch P보정 후 자세가 다음 Z 하강 허용범위에 '
                '들어오지 못했습니다: '
                f'roll={final_roll_error:.2f}deg, '
                f'pitch={final_pitch_error:.2f}deg, '
                f'continue_limit={self._roll_pitch_tolerance:.2f}deg, '
                f'hard_abort={self._final_z_p_roll_pitch_abort:.2f}deg'
            )
        self._alignment_ready_for_descent = True
        self._publish(
            'EXECUTED: Z→XY P보정→Roll/Pitch P보정 1사이클 완료. '
            '실제 결과를 확인한 뒤 다음 descend_z_p_once 승인 대기'
        )

    def _execute_joint_task_step(
        self,
        task_error: np.ndarray,
        label: str,
    ):
        """현재 관절에서 6D P 오차를 한 번의 관절 증분으로 실행한다."""
        if self._joint_vertical_chain is None:
            raise ValueError('관절 Jacobian URDF 체인이 준비되지 않았습니다')
        with self._lock:
            joints = (
                None
                if self._latest_joints is None
                else list(self._latest_joints)
            )
            joints_received_at = self._joints_received_at
            hardware_received_at = self._hardware_pose_received_at
        self._fresh(joints_received_at, f'{label} /joint_states')
        self._fresh(
            hardware_received_at,
            f'{label} manufacturer Cartesian pose',
        )
        if joints is None:
            raise ValueError(f'{label}용 /joint_states가 없습니다')

        before = self._current_hardware_transform()
        positions = dict(zip(JOINT_NAMES, joints))
        model_transform, jacobian = (
            self._joint_vertical_chain.forward_and_jacobian(positions)
        )
        tf_transform = self._current_flange()
        model_position_error = float(
            np.linalg.norm(model_transform[:3, 3] - tf_transform[:3, 3])
        )
        if model_position_error > 0.010:
            raise RuntimeError(
                f'{label}: 관절 Jacobian URDF와 robot_state_publisher TF가 '
                '다릅니다: '
                f'position_error={model_position_error * 1000.0:.1f}mm'
            )

        joint_delta = damped_joint_step(
            jacobian,
            np.asarray(task_error, dtype=np.float64),
            self._joint_vertical_damping,
            self._joint_vertical_orientation_scale,
            self._joint_vertical_max_joint_step,
        )
        target_joints = np.asarray(joints, dtype=np.float64) + joint_delta
        limited_target = self._joint_vertical_chain.clamp_to_limits(
            JOINT_NAMES,
            target_joints,
            self._joint_vertical_joint_limit_margin,
        )
        if not np.allclose(target_joints, limited_target, atol=1e-9):
            raise RuntimeError(
                f'{label}: 계산된 목표가 URDF 관절 한계 여유를 침범합니다'
            )

        predicted_task = jacobian @ joint_delta
        joint_delta_degrees = [
            round(math.degrees(float(value)), 3)
            for value in joint_delta
        ]
        smallest_singular = float(
            np.min(np.linalg.svd(jacobian, compute_uv=False))
        )
        self._publish(
            f'{label} 관절 명령: joint_delta_deg={joint_delta_degrees}, '
            'predicted_xyz='
            f'[{predicted_task[0] * 1000.0:+.1f}, '
            f'{predicted_task[1] * 1000.0:+.1f}, '
            f'{predicted_task[2] * 1000.0:+.1f}]mm, '
            f'jacobian_sigma_min={smallest_singular:.6f}'
        )
        self._send_joint_target(
            limited_target.tolist(),
            motion_seconds=self._joint_vertical_motion_seconds,
        )
        action_finished_at = time.monotonic()
        after = self._wait_for_new_hardware_transform(
            action_finished_at,
            timeout_seconds=5.0,
        )
        time.sleep(self._settle)
        # 첫 새 샘플은 action 종료 직후의 과도 상태일 수 있다. settle 동안
        # bridge가 갱신한 가장 최신 정지 자세를 해당 단계의 측정값으로 쓴다.
        after = self._current_hardware_transform()
        return before, after, predicted_task

    def _execute_vertical_z_step(self, dz: float, label: str):
        """선택한 backend로 상대 Z 하강 한 단계를 실행하고 실제 pose를 읽는다."""
        distance = float(dz)
        if not math.isfinite(distance) or distance <= 0.0:
            raise ValueError(f'{label}: Z 하강량은 양의 유한값이어야 합니다')
        if self._vertical_z_control_backend == 'joint':
            task_error = np.zeros(6, dtype=np.float64)
            task_error[2] = -distance
            return self._execute_joint_task_step(task_error, label)

        before = self._current_hardware_transform()
        target = before.copy()
        target[2, 3] = float(before[2, 3]) - distance
        self._send_cartesian_transform(
            target,
            f'{label} send_coords Z-only',
            lock_z=False,
            lock_roll_pitch=False,
            speed=self._vertical_z_cartesian_speed,
            mode=self._vertical_z_cartesian_mode,
        )
        time.sleep(self._settle)
        after = self._current_hardware_transform()
        predicted_task = np.zeros(6, dtype=np.float64)
        predicted_task[2] = -distance
        return before, after, predicted_task

    def _execute_descend_joint_z_once(self) -> None:
        """Z→측정→XY P보정→측정→초기 R/P P보정→측정 1회."""
        if (
            self._vertical_z_control_backend == 'joint'
            and not self._enable_joint_vertical_descent
        ):
            raise ValueError('enable_joint_vertical_descent=false')
        if (
            self._vertical_z_control_backend == 'joint'
            and self._joint_vertical_chain is None
        ):
            raise ValueError('관절 Jacobian URDF 체인이 준비되지 않았습니다')
        if (
            not self._alignment_ready_for_descent
            or self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
        ):
            raise ValueError('성공한 execute_once 정렬 결과가 먼저 필요합니다')
        current = self._current_hardware_transform()
        current_z = float(current[2, 3])
        remaining_z = current_z - self._aligned_hardware_final_target_z
        dz = proportional_z_descent_m(
            remaining_z,
            self._joint_vertical_z_kp,
            self._joint_vertical_max_z_step,
            self._joint_vertical_minimum_progress,
        )
        if dz < self._joint_vertical_minimum_progress:
            raise ValueError(
                '관절 Z 하강 목표에 도달했거나 유효 하강량이 없습니다: '
                f'remaining={remaining_z * 1000.0:.1f}mm'
            )

        self._alignment_ready_for_descent = False
        self._publish(
            '순차 P제어 1/3 Z 하강 준비: '
            f'remaining={remaining_z * 1000.0:.1f}mm, '
            f'dz_command=-{dz * 1000.0:.1f}mm, '
            f'backend={self._vertical_z_control_backend}'
        )
        if self._vertical_z_control_backend == 'joint':
            time.sleep(self._warning_delay)
        _, after_z, _ = self._execute_vertical_z_step(dz, 'Z 1/3')
        actual_descent = current_z - float(after_z[2, 3])
        overshoot = actual_descent - dz
        xy_error = xy_residual_m(
            after_z[:2, 3], self._aligned_hardware_xy
        )
        roll_error, pitch_error = self._roll_pitch_errors(
            after_z,
            self._aligned_hardware_observation_transform,
        )
        self._publish(
            '순차 P제어 1/3 Z 측정: '
            f'backend={self._vertical_z_control_backend}, '
            f'command={dz * 1000.0:.1f}mm, '
            f'actual={actual_descent * 1000.0:.1f}mm, '
            f'overshoot={overshoot * 1000.0:+.1f}mm, '
            f'xy={xy_error * 1000.0:.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg'
        )
        if actual_descent < self._joint_vertical_minimum_progress:
            raise RuntimeError('관절 Jacobian Z 하강 실제 진행량이 부족합니다')
        if actual_descent > self._joint_vertical_hard_maximum_descent:
            raise RuntimeError(
                '관절 Z 실제 하강량이 하드 한계를 초과했습니다: '
                f'actual={actual_descent * 1000.0:.1f}mm, '
                'hard_limit='
                f'{self._joint_vertical_hard_maximum_descent * 1000.0:.1f}mm'
            )
        if (
            float(after_z[2, 3])
            < self._aligned_hardware_final_target_z
            - self._joint_vertical_minimum_progress
        ):
            raise RuntimeError(
                '관절 Z 하강이 최종 목표 Z 아래로 지나갔습니다: '
                f'actual_z={after_z[2, 3] * 1000.0:.1f}mm, '
                'target_z='
                f'{self._aligned_hardware_final_target_z * 1000.0:.1f}mm'
            )
        if overshoot > self._joint_vertical_maximum_overshoot:
            self._publish(
                'WARN: Z 하강 soft overshoot를 초과했지만 하드 한계 안이라 '
                '이번 XY/Roll/Pitch 측정·보정은 계속합니다: '
                f'overshoot={overshoot * 1000.0:.1f}mm'
            )
        if (
            max(roll_error, pitch_error)
            > self._joint_vertical_roll_pitch_hard_limit
        ):
            raise RuntimeError(
                'Z 하강 직후 초기 Roll/Pitch 하드 한계 초과: '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg, '
                f'limit={self._joint_vertical_roll_pitch_hard_limit:.2f}deg'
            )

        # 2/3: Z에만 관절 Jacobian을 사용한다. XY는 앞선 coarse/refine에서
        # 상대적으로 잘 동작한 제조사 Cartesian 좌표 명령으로 보정한다.
        after_xy = after_z
        if xy_error > self._joint_vertical_xy_correction_deadband:
            xy_target = after_z.copy()
            xy_target[:2, 3] = after_z[:2, 3] + (
                self._aligned_hardware_xy - after_z[:2, 3]
            ) * self._joint_vertical_xy_kp
            xy_command = xy_residual_m(
                xy_target[:2, 3], after_z[:2, 3]
            )
            if xy_command > self._refine_maximum_step:
                raise RuntimeError(
                    'Z 하강 후 Cartesian XY 보정량이 상한을 초과했습니다: '
                    f'command={xy_command * 1000.0:.1f}mm, '
                    f'limit={self._refine_maximum_step * 1000.0:.1f}mm'
                )
            self._publish(
                '혼합 순차 P제어 2/3 Cartesian XY 보정 시작: '
                f'error={xy_error * 1000.0:.1f}mm, '
                f'command={xy_command * 1000.0:.1f}mm, '
                f'kp={self._joint_vertical_xy_kp:.2f}, '
                'Z/RPY target=current, path_lock=false'
            )
            self._send_cartesian_transform(
                xy_target,
                'Cartesian XY 2/3',
                lock_z=False,
                lock_roll_pitch=False,
                accept_timeout_xy_residual_m=(
                    self._joint_vertical_xy_tolerance
                ),
            )
            time.sleep(self._settle)
            after_xy = self._current_hardware_transform()
        xy_after = xy_residual_m(
            after_xy[:2, 3], self._aligned_hardware_xy
        )
        xy_z_drift = abs(float(after_xy[2, 3] - after_z[2, 3]))
        self._publish(
            '혼합 순차 P제어 2/3 Cartesian XY 측정: '
            f'xy={xy_after * 1000.0:.1f}mm, '
            f'z_drift={xy_z_drift * 1000.0:.1f}mm'
        )
        if xy_z_drift > self._joint_vertical_maximum_correction_z_drift:
            self._publish(
                'WARN: Cartesian XY 보정의 Z 결합 이동이 경고값을 '
                '초과했습니다. 절대 포트 목표 Z로 계속 판정합니다: '
                f'z_drift={xy_z_drift * 1000.0:.1f}mm'
            )
        if (
            float(after_xy[2, 3])
            < self._aligned_hardware_final_target_z
            - self._joint_vertical_minimum_progress
        ):
            message = (
                'Cartesian XY 보정 후 포트 기반 목표 Z 아래로 내려갔습니다: '
                f'actual_z={after_xy[2, 3] * 1000.0:.1f}mm, '
                'target_z='
                f'{self._aligned_hardware_final_target_z * 1000.0:.1f}mm'
            )
            if not self._allow_mixed_correction_below_final_z:
                raise RuntimeError(message)
            self._publish(
                'WARN: ' + message + '. 설정에 따라 차단하지 않고 '
                '혼합 보정을 계속합니다'
            )

        # 3/3: 기존 coarse/Yaw 뒤 자세 복구와 같은 Cartesian 경로를 쓴다.
        # 초기 관측 R/P를 목표로 하고 현재 XYZ/Yaw는 그대로 둔다.
        rp_before = self._roll_pitch_errors(
            after_xy,
            self._aligned_hardware_observation_transform,
        )
        tilt_before = max(rp_before)
        after_rp = after_xy
        rp_executed = False
        if tilt_before > self._joint_vertical_roll_pitch_correction_deadband:
            rp_target = (
                apply_proportional_observation_roll_pitch_with_current_yaw(
                    after_xy,
                    self._aligned_hardware_observation_transform,
                    self._joint_vertical_orientation_kp,
                    self._pitch_correction_gain,
                )
            )
            target_rpy = rotation_to_rpy_degrees(rp_target[:3, :3])
            initial_rpy = rotation_to_rpy_degrees(
                self._aligned_hardware_observation_transform[:3, :3]
            )
            self._publish(
                '혼합 순차 P제어 3/3 Cartesian 초기 Roll/Pitch 보정 시작: '
                f'roll={rp_before[0]:.2f}deg, '
                f'pitch={rp_before[1]:.2f}deg, '
                f'kp={self._joint_vertical_orientation_kp:.2f}, '
                f'pitch_gain={self._pitch_correction_gain:.2f}, '
                'initial_target_rp='
                f'[{initial_rpy[0]:+.2f}, {initial_rpy[1]:+.2f}]deg, '
                'command_target_rp='
                f'[{target_rpy[0]:+.2f}, {target_rpy[1]:+.2f}]deg'
            )
            self._send_cartesian_transform(
                rp_target,
                'Cartesian Roll/Pitch 3/3',
                lock_z=False,
                lock_roll_pitch=False,
                # bridge orientation tolerance 경계에서 timeout이어도 실제
                # XY가 유지됐다면 결과 pose를 받아 아래 R/P·Z 검사로 판정한다.
                accept_timeout_xy_residual_m=(
                    self._joint_vertical_xy_tolerance
                ),
            )
            time.sleep(self._settle)
            after_rp = self._current_hardware_transform()
            rp_executed = True

        final_xy = xy_residual_m(
            after_rp[:2, 3], self._aligned_hardware_xy
        )
        final_roll, final_pitch = self._roll_pitch_errors(
            after_rp,
            self._aligned_hardware_observation_transform,
        )
        final_tilt = max(final_roll, final_pitch)
        tilt_improvement = tilt_before - final_tilt
        rp_z_drift = abs(float(after_rp[2, 3] - after_xy[2, 3]))
        total_descent = current_z - float(after_rp[2, 3])
        final_remaining_z = float(
            after_rp[2, 3] - self._aligned_hardware_final_target_z
        )
        self._publish(
            '혼합 순차 P제어 3/3 최종 측정: '
            f'total_dz={total_descent * 1000.0:+.1f}mm, '
            f'remaining_z={final_remaining_z * 1000.0:+.1f}mm, '
            f'xy={final_xy * 1000.0:.1f}mm, '
            f'roll={final_roll:.2f}deg, pitch={final_pitch:.2f}deg, '
            f'rp_z_drift={rp_z_drift * 1000.0:.1f}mm, '
            f'tilt_improvement={tilt_improvement:+.2f}deg'
        )
        if rp_z_drift > self._joint_vertical_maximum_correction_z_drift:
            self._publish(
                'WARN: Cartesian Roll/Pitch 보정의 Z 결합 이동이 경고값을 '
                '초과했습니다. 절대 포트 목표 Z로 결과를 판정합니다: '
                f'z_drift={rp_z_drift * 1000.0:.1f}mm'
            )
        if final_remaining_z < -self._joint_vertical_minimum_progress:
            message = (
                '혼합 보정 후 포트 기반 최종 Z 아래로 내려갔습니다: '
                f'overshoot={-final_remaining_z * 1000.0:.1f}mm'
            )
            if not self._allow_mixed_correction_below_final_z:
                raise RuntimeError(message)
            self._publish(
                'WARN: ' + message + '. 설정에 따라 과삽입 차단 없이 '
                '다음 단계로 진행합니다'
            )
        if total_descent > self._joint_vertical_hard_maximum_cycle_descent:
            raise RuntimeError(
                '전체 혼합 보정 중 누적 Z 하강이 사이클 하드 한계를 '
                '초과했습니다: '
                f'total={total_descent * 1000.0:.1f}mm, '
                'limit='
                '%.1fmm'
                % (self._joint_vertical_hard_maximum_cycle_descent * 1000.0)
            )
        if final_xy > self._joint_vertical_xy_tolerance:
            raise RuntimeError(
                '순차 보정 후 XY 허용오차 초과: '
                f'xy={final_xy * 1000.0:.1f}mm'
            )
        if rp_executed and (
            tilt_improvement
            < self._joint_vertical_minimum_tilt_improvement
        ):
            raise RuntimeError(
                '초기 Roll/Pitch P보정 후 오차 감소 부족: '
                f'improvement={tilt_improvement:+.2f}deg'
            )
        if final_tilt > self._joint_vertical_roll_pitch_tolerance:
            raise RuntimeError(
                '순차 보정 후 초기 Roll/Pitch 허용오차 초과: '
                f'roll={final_roll:.2f}deg, pitch={final_pitch:.2f}deg'
            )
        if final_remaining_z <= self._joint_vertical_final_z_guard:
            self._alignment_ready_for_descent = False
            self._insertion_ready = True
            self._final_insertion_ready = True
            self._publish(
                'EXECUTED: 혼합 P제어로 포트 기반 최종 Z 안전 여유 '
                '구간까지 도달했습니다: '
                f'remaining={final_remaining_z * 1000.0:.1f}mm. '
                '다음 Z 사이클은 차단하며 마지막 삽입은 별도 '
                '저속/접촉 감시 단계가 필요합니다'
            )
        else:
            self._alignment_ready_for_descent = True
            self._publish(
                'EXECUTED: 관절 Z→측정→Cartesian XY P보정→측정→'
                'Cartesian 초기 Roll/Pitch P보정→측정 1사이클 완료. '
                '절대 포트 목표 Z의 남은 거리로 다음 '
                'descend_joint_z_once 승인 대기'
            )

    def _execute_descend_joint_z_to_guard(self) -> None:
        """혼합 P제어 사이클을 포트 목표 Z 안전 여유까지 반복한다."""
        if (
            not self._alignment_ready_for_descent
            or self._aligned_hardware_final_target_z is None
        ):
            raise ValueError('성공한 execute_once 정렬 결과가 먼저 필요합니다')
        start_z = float(self._current_hardware_transform()[2, 3])
        start_remaining = (
            start_z - self._aligned_hardware_final_target_z
        )
        self._publish(
            'Z+Cartesian XY/RP 자동 반복 시작: '
            f'z_backend={self._vertical_z_control_backend}, '
            f'remaining={start_remaining * 1000.0:.1f}mm, '
            f'guard={self._joint_vertical_final_z_guard * 1000.0:.1f}mm, '
            f'max_cycles={self._joint_vertical_max_cycles}. '
            '각 사이클은 완전 정지·실제 좌표 측정 후 진행합니다'
        )
        completed = 0
        for cycle in range(1, self._joint_vertical_max_cycles + 1):
            if not self._alignment_ready_for_descent:
                break
            current_z = float(self._current_hardware_transform()[2, 3])
            remaining = current_z - self._aligned_hardware_final_target_z
            if remaining <= self._joint_vertical_final_z_guard:
                self._alignment_ready_for_descent = False
                break
            self._publish(
                f'자동 혼합 P제어 사이클 {cycle}/'
                f'{self._joint_vertical_max_cycles}: '
                f'remaining={remaining * 1000.0:.1f}mm'
            )
            self._execute_descend_joint_z_once()
            completed = cycle

        final_z = float(self._current_hardware_transform()[2, 3])
        final_remaining = (
            final_z - self._aligned_hardware_final_target_z
        )
        if final_remaining > self._joint_vertical_final_z_guard:
            self._alignment_ready_for_descent = False
            raise RuntimeError(
                '자동 혼합 P제어 최대 사이클 후에도 안전 여유에 '
                '도달하지 못했습니다: '
                f'cycles={completed}, '
                f'remaining={final_remaining * 1000.0:.1f}mm'
            )
        self._alignment_ready_for_descent = False
        self._insertion_ready = True
        self._final_insertion_ready = True
        self._publish(
            'EXECUTED: 자동 혼합 P제어가 포트 기반 최종 Z 안전 여유에 '
            '도달했습니다: '
            f'cycles={completed}, start_remaining='
            f'{start_remaining * 1000.0:.1f}mm, '
            f'final_remaining={final_remaining * 1000.0:.1f}mm. '
            'insert_step_once 또는 insert_final_z_once 승인 대기'
        )

    def _execute_final_insertion_z(self) -> None:
        """guard 자세에서 선택한 Z backend로 상대 하강한다."""
        if not self._enable_final_insertion_z:
            raise ValueError('enable_final_insertion_z=false')
        if (
            self._vertical_z_control_backend == 'joint'
            and self._joint_vertical_chain is None
        ):
            raise ValueError('관절 Jacobian URDF 체인이 준비되지 않았습니다')
        if (
            not self._final_insertion_ready
            or self._aligned_hardware_final_target_z is None
            or self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
        ):
            raise ValueError(
                '성공한 descend_joint_z_to_guard 결과가 먼저 필요합니다'
            )

        self._final_insertion_ready = False
        start = self._current_hardware_transform()
        start_z = float(start[2, 3])
        # 기본 모드는 상대 10mm 목표가 configured 포트 삽입 목표보다
        # 낮으면 포트 기반 최종 Z에서 자른다. 과삽입 허용 시험 모드에서
        # 이미 configured Z 아래라면 10mm 단계는 생략하고 R/P 복구와
        # 그 뒤의 별도 5mm 하강 단계로 이어간다.
        if (
            self._allow_mixed_correction_below_final_z
            and start_z < self._aligned_hardware_final_target_z
        ):
            self._publish(
                'WARN: guard 시작 자세가 configured 최종 Z보다 이미 낮아 '
                '최종 상대 10mm 하강을 생략합니다. 과삽입을 거절하지 않고 '
                'R/P 복구와 후속 Z 하강으로 진행합니다: below_target='
                f'{(self._aligned_hardware_final_target_z - start_z) * 1000.0:.1f}mm'
            )
            self._execute_post_insertion_roll_pitch_recovery()
            self._execute_post_recovery_final_z()
            return
        else:
            target_z = final_insertion_target_z_m(
                start_z,
                self._final_insertion_relative_distance,
                self._aligned_hardware_final_target_z,
            )
        commanded_total = start_z - target_z
        if commanded_total <= self._final_insertion_tolerance:
            self._publish(
                'EXECUTED: guard 도달 자세가 최종 삽입 Z 허용범위 안이라 '
                '추가 하강을 생략합니다: '
                f'remaining={commanded_total * 1000.0:.1f}mm'
            )
            self._execute_post_insertion_roll_pitch_recovery()
            self._execute_post_recovery_final_z()
            return

        self._publish(
            f'{self._warning_delay:.1f}초 후 최종 Z-only 단발 삽입: '
            f'relative_request={self._final_insertion_relative_distance * 1000.0:.1f}mm, '
            f'actual_command={commanded_total * 1000.0:.1f}mm, '
            f'start_z={start_z * 1000.0:.1f}mm, '
            f'target_z={target_z * 1000.0:.1f}mm. '
            'XY/Roll/Pitch 보정 없이 Z 명령을 한 번만 전송합니다. '
            f'backend={self._vertical_z_control_backend}'
        )
        if self._vertical_z_control_backend == 'joint':
            time.sleep(self._warning_delay)
        _, final, predicted_task = self._execute_vertical_z_step(
            commanded_total,
            '최종 Z-only 단발 삽입',
        )
        final_descent = start_z - float(final[2, 3])
        final_remaining = float(final[2, 3] - target_z)
        target_overshoot = max(0.0, -final_remaining)
        xy_error = xy_residual_m(final[:2, 3], self._aligned_hardware_xy)
        roll_error, pitch_error = self._roll_pitch_errors(
            final,
            self._aligned_hardware_observation_transform,
        )
        if final_descent < self._joint_vertical_minimum_progress:
            raise RuntimeError('최종 Z-only 단발 삽입 실제 진행량이 부족합니다')
        if final_descent > self._final_insertion_hard_maximum_total_descent:
            raise RuntimeError(
                '최종 Z-only 단발 삽입 실제 하강량이 하드 한계를 '
                '초과했습니다: '
                f'actual={final_descent * 1000.0:.1f}mm, '
                'limit='
                f'{self._final_insertion_hard_maximum_total_descent * 1000.0:.1f}mm'
            )
        if target_overshoot > self._joint_vertical_maximum_overshoot:
            raise RuntimeError(
                '최종 Z-only 단발 삽입 목표를 과도하게 지나갔습니다: '
                f'overshoot={target_overshoot * 1000.0:.1f}mm'
            )
        self._publish(
            'EXECUTED: guard 이후 최종 Z-only 단발 삽입 완료: '
            f'predicted_dz={predicted_task[2] * 1000.0:+.1f}mm, '
            f'actual_dz={-final_descent * 1000.0:+.1f}mm, '
            f'target_error={final_remaining * 1000.0:+.1f}mm, '
            f'within_tolerance={abs(final_remaining) <= self._final_insertion_tolerance}, '
            f'xy={xy_error * 1000.0:.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg'
        )
        self._execute_post_insertion_roll_pitch_recovery()
        self._execute_post_recovery_final_z()

    def _execute_post_insertion_roll_pitch_recovery(self) -> None:
        """최종 삽입 뒤 actual XYZ/Yaw에서 초기 관측 R/P를 한 번 적용한다."""
        if not self._enable_post_insertion_roll_pitch_recovery:
            return
        if self._aligned_hardware_observation_transform is None:
            raise RuntimeError('삽입 후 Roll/Pitch 복구용 초기 관측 기준이 없습니다')
        before = self._current_hardware_transform()
        before_z = float(before[2, 3])
        before_roll, before_pitch = self._roll_pitch_errors(
            before,
            self._aligned_hardware_observation_transform,
        )
        target = apply_proportional_observation_roll_pitch_with_current_yaw(
            before,
            self._aligned_hardware_observation_transform,
            1.0,
            self._pitch_correction_gain,
        )
        target_rpy = rotation_to_rpy_degrees(target[:3, :3])
        self._publish(
            '삽입 후 초기 Roll/Pitch 무제한 복구 시작: '
            f'before_roll={before_roll:.2f}deg, '
            f'before_pitch={before_pitch:.2f}deg, '
            f'pitch_gain={self._pitch_correction_gain:.2f}, '
            'target_rp='
            f'[{target_rpy[0]:+.2f}, {target_rpy[1]:+.2f}]deg, '
            f'speed={self._post_insertion_roll_pitch_speed}. '
            'actual X/Y/Z와 Yaw를 목표에 유지하지만 결합 Z 하강은 '
            '제한하거나 실패 처리하지 않습니다'
        )
        self._send_cartesian_transform(
            target,
            '삽입 후 초기 Roll/Pitch 복구',
            lock_z=False,
            lock_roll_pitch=False,
            speed=self._post_insertion_roll_pitch_speed,
        )
        time.sleep(self._settle)
        after = self._current_hardware_transform()
        coupled_descent = before_z - float(after[2, 3])
        xy_error = xy_residual_m(after[:2, 3], self._aligned_hardware_xy)
        roll_error, pitch_error = self._roll_pitch_errors(
            after,
            self._aligned_hardware_observation_transform,
        )
        final_z_error = float(
            after[2, 3] - self._aligned_hardware_final_target_z
        )
        self._publish(
            'EXECUTED: 삽입 후 초기 Roll/Pitch 복구 완료: '
            f'coupled_dz={-coupled_descent * 1000.0:+.1f}mm, '
            f'final_z_error={final_z_error * 1000.0:+.1f}mm, '
            f'xy={xy_error * 1000.0:.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg. '
            '추가 Z 하강 제한은 적용하지 않았습니다'
        )

    def _execute_post_recovery_final_z(self) -> None:
        """삽입 후 R/P 복구가 끝난 자세에서 상대 Z를 한 번 더 내린다."""
        if not self._enable_post_recovery_final_z:
            return
        if (
            self._vertical_z_control_backend == 'joint'
            and self._joint_vertical_chain is None
        ):
            raise ValueError('관절 Jacobian URDF 체인이 준비되지 않았습니다')
        if (
            self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
        ):
            raise RuntimeError('삽입 후 추가 Z 하강용 정렬 기준이 없습니다')

        start = self._current_hardware_transform()
        start_z = float(start[2, 3])
        target_z = start_z - self._post_recovery_final_z_distance
        if target_z < self._final_z_descent_minimum_z:
            raise RuntimeError(
                '삽입 후 추가 Z 목표가 절대 하한보다 낮습니다: '
                f'target_z={target_z * 1000.0:.1f}mm, '
                f'minimum_z={self._final_z_descent_minimum_z * 1000.0:.1f}mm'
            )

        self._publish(
            f'{self._warning_delay:.1f}초 후 R/P 복구 뒤 최종 Z-only 하강: '
            f'relative_request={self._post_recovery_final_z_distance * 1000.0:.1f}mm, '
            f'start_z={start_z * 1000.0:.1f}mm, '
            f'target_z={target_z * 1000.0:.1f}mm. '
            'XY/Roll/Pitch 추가 보정 없이 Z 명령을 한 번만 전송합니다. '
            f'backend={self._vertical_z_control_backend}'
        )
        if self._vertical_z_control_backend == 'joint':
            time.sleep(self._warning_delay)
        _, final, predicted_task = self._execute_vertical_z_step(
            self._post_recovery_final_z_distance,
            'R/P 복구 뒤 최종 Z-only 하강',
        )
        actual_descent = start_z - float(final[2, 3])
        xy_error = xy_residual_m(final[:2, 3], self._aligned_hardware_xy)
        roll_error, pitch_error = self._roll_pitch_errors(
            final,
            self._aligned_hardware_observation_transform,
        )
        if actual_descent < self._joint_vertical_minimum_progress:
            message = (
                'R/P 복구 뒤 최종 Z-only 실제 진행량이 부족합니다: '
                f'command={self._post_recovery_final_z_distance * 1000.0:.1f}mm, '
                f'actual={actual_descent * 1000.0:+.1f}mm, '
                f'minimum={self._joint_vertical_minimum_progress * 1000.0:.1f}mm'
            )
            if not self._allow_mixed_correction_below_final_z:
                raise RuntimeError(message)
            self._publish(
                'WARN: ' + message + '. 과삽입 허용 시험 설정에 따라 '
                '거절하지 않고 현재 실제 자세로 완료 처리합니다'
            )
        if actual_descent > self._final_insertion_hard_maximum_total_descent:
            raise RuntimeError(
                'R/P 복구 뒤 최종 Z-only 실제 하강량이 하드 한계를 '
                '초과했습니다: '
                f'actual={actual_descent * 1000.0:.1f}mm, '
                'limit='
                f'{self._final_insertion_hard_maximum_total_descent * 1000.0:.1f}mm'
            )
        self._publish(
            'EXECUTED: R/P 복구 뒤 최종 Z-only 하강 완료: '
            f'command={self._post_recovery_final_z_distance * 1000.0:.1f}mm, '
            f'predicted_dz={predicted_task[2] * 1000.0:+.1f}mm, '
            f'actual_dz={-actual_descent * 1000.0:+.1f}mm, '
            f'xy={xy_error * 1000.0:.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg'
        )

    def _execute_full_sequence(
        self,
        *,
        include_final_insertion: bool = False,
    ) -> None:
        """초기 정렬부터 Z guard와 선택적 최종 Z까지 한 번에 실행한다."""
        self._publish(
            '통합 실행 시작: 초기 PBVS 고정목표 정렬 → XY/Roll/Pitch/Yaw '
            '결합 보정 → Z+Cartesian XY/RP 자동 반복 → 최종 오차 측정. '
            f'z_backend={self._vertical_z_control_backend}'
        )
        try:
            self._execute_once()
        except Exception as error:
            raise RuntimeError(
                f'통합 실행 초기 정렬 단계 실패: {error}'
            ) from error
        self._publish(
            '통합 실행 초기 정렬 완료: Z 안전 여유까지 자동 하강을 시작합니다'
        )
        try:
            self._execute_descend_joint_z_to_guard()
        except Exception as error:
            raise RuntimeError(
                f'통합 실행 Z 자동 하강 단계 실패: {error}'
            ) from error
        if include_final_insertion:
            self._publish(
                '통합 실행 guard 도달 완료: 최종 상대 Z 삽입을 시작합니다'
            )
            try:
                self._execute_final_insertion_z()
            except Exception as error:
                raise RuntimeError(
                    f'통합 실행 최종 Z 삽입 단계 실패: {error}'
                ) from error
        self._publish_final_error_report()
        self._publish(
            'EXECUTED: '
            f'{"execute_full_sequence_with_final_z" if include_final_insertion else "execute_full_sequence"} '
            '완료. 초기 정렬부터 Z 이동과 최종 오차 측정을 마쳤습니다'
        )

    def _publish_final_error_report(self) -> None:
        """통합 실행 종료 시 제조사 실제 자세와 저장 목표의 오차를 발행한다."""
        if (
            self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
            or self._aligned_hardware_yaw_deg is None
        ):
            raise RuntimeError('최종 오차 계산용 정렬 기준이 없습니다')
        actual = self._current_hardware_transform()
        error_x = float(actual[0, 3] - self._aligned_hardware_xy[0])
        error_y = float(actual[1, 3] - self._aligned_hardware_xy[1])
        error_xy = math.hypot(error_x, error_y)
        remaining_z = float(
            actual[2, 3] - self._aligned_hardware_final_target_z
        )
        guard_error = float(
            remaining_z - self._joint_vertical_final_z_guard
        )
        roll_error, pitch_error = self._roll_pitch_errors(
            actual,
            self._aligned_hardware_observation_transform,
        )
        actual_rpy = rotation_to_rpy_degrees(actual[:3, :3])
        yaw_error = float(
            (actual_rpy[2] - self._aligned_hardware_yaw_deg + 180.0)
            % 360.0
            - 180.0
        )
        self._publish(
            'FINAL_ERROR_REPORT: actual_xyz=['
            f'{actual[0, 3] * 1000.0:+.1f}, '
            f'{actual[1, 3] * 1000.0:+.1f}, '
            f'{actual[2, 3] * 1000.0:+.1f}]mm, '
            'error_xy=['
            f'{error_x * 1000.0:+.1f}, {error_y * 1000.0:+.1f}]mm, '
            f'xy_norm={error_xy * 1000.0:.1f}mm, '
            f'z_remaining={remaining_z * 1000.0:+.1f}mm, '
            f'z_guard_error={guard_error * 1000.0:+.1f}mm, '
            f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg, '
            f'yaw_drift={yaw_error:+.2f}deg'
        )

    def _execute_insert_step_once(self) -> None:
        """10mm 최종 목표를 향해 최대 0.5mm 단발 삽입한다."""
        if not self._enable_final_insertion:
            raise ValueError('enable_final_insertion=false')
        if (
            not self._insertion_ready
            or self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
        ):
            raise ValueError(
                'descend_joint_z_to_guard로 안전 여유에 도달한 '
                '정렬 결과가 먼저 필요합니다'
            )

        current = self._current_hardware_transform()
        current_z = float(current[2, 3])
        remaining = current_z - self._aligned_hardware_final_target_z
        if remaining > self._joint_vertical_final_z_guard:
            self._insertion_ready = False
            raise RuntimeError(
                '삽입 시작 위치가 Z guard 밖입니다: '
                f'remaining={remaining * 1000.0:.1f}mm'
            )
        if remaining <= self._final_insertion_target_tolerance:
            self._insertion_ready = False
            self._publish(
                'EXECUTED: 10mm 삽입 목표에 이미 도달했습니다: '
                f'remaining={remaining * 1000.0:.2f}mm'
            )
            return

        xy_error = xy_residual_m(current[:2, 3], self._aligned_hardware_xy)
        roll_error, pitch_error = self._roll_pitch_errors(
            current,
            self._aligned_hardware_observation_transform,
        )
        if xy_error > self._joint_vertical_xy_tolerance:
            raise RuntimeError(
                f'삽입 전 XY 허용오차 초과: {xy_error * 1000.0:.1f}mm'
            )
        if (
            max(roll_error, pitch_error)
            > self._joint_vertical_roll_pitch_tolerance
        ):
            raise RuntimeError(
                '삽입 전 Roll/Pitch 허용오차 초과: '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg'
            )

        command_step = min(self._final_insertion_step, remaining)
        target = current.copy()
        target[2, 3] = current_z - command_step
        self._publish(
            '단발 삽입 승인 실행: '
            f'command={command_step * 1000.0:.2f}mm, '
            f'remaining={remaining * 1000.0:.2f}mm, '
            'force/contact sensing은 없습니다'
        )
        self._send_cartesian_transform(
            target,
            'USB 10mm 단발 삽입',
            lock_z=False,
            lock_roll_pitch=True,
            speed=self._final_insertion_speed,
            mode=self._final_z_descent_cartesian_mode,
        )
        time.sleep(self._settle)
        actual = self._current_hardware_transform()
        actual_step = current_z - float(actual[2, 3])
        final_remaining = float(
            actual[2, 3] - self._aligned_hardware_final_target_z
        )
        final_xy = xy_residual_m(actual[:2, 3], self._aligned_hardware_xy)
        final_roll, final_pitch = self._roll_pitch_errors(
            actual,
            self._aligned_hardware_observation_transform,
        )
        self._publish(
            '단발 삽입 실측: '
            f'command={command_step * 1000.0:.2f}mm, '
            f'actual={actual_step * 1000.0:.2f}mm, '
            f'remaining={final_remaining * 1000.0:.2f}mm, '
            f'xy={final_xy * 1000.0:.2f}mm, '
            f'roll={final_roll:.2f}deg, pitch={final_pitch:.2f}deg'
        )
        if actual_step <= 0.0:
            self._insertion_ready = False
            raise RuntimeError('삽입 명령 후 실제 하강이 확인되지 않았습니다')
        if actual_step > self._final_insertion_maximum_actual_step:
            self._insertion_ready = False
            raise RuntimeError(
                '단발 삽입 실제 이동량이 하드 한계를 초과했습니다: '
                f'actual={actual_step * 1000.0:.2f}mm'
            )
        if final_remaining < -self._final_insertion_target_tolerance:
            self._insertion_ready = False
            raise RuntimeError(
                '10mm 삽입 목표를 초과했습니다: '
                f'overshoot={-final_remaining * 1000.0:.2f}mm'
            )
        if (
            final_xy > self._joint_vertical_xy_tolerance
            or max(final_roll, final_pitch)
            > self._joint_vertical_roll_pitch_tolerance
        ):
            self._insertion_ready = False
            raise RuntimeError('삽입 후 XY 또는 Roll/Pitch 허용오차 초과')

        reached = final_remaining <= self._final_insertion_target_tolerance
        self._insertion_ready = not reached
        if reached:
            self._publish('EXECUTED: USB 10mm 삽입 목표 도달')
        else:
            self._publish(
                'EXECUTED: 단발 삽입 완료. 상태를 직접 '
                '확인한 뒤 insert_step_once를 다시 승인하세요'
            )

    def _execute_descend_z_p_to_target(self) -> None:
        """YAML의 목표 Z까지 안전검사를 거쳐 P제어 하강을 반복한다."""
        if (
            not self._alignment_ready_for_descent
            or self._aligned_hardware_xy is None
            or self._aligned_hardware_observation_transform is None
            or self._aligned_hardware_final_target_z is None
        ):
            raise ValueError('성공한 execute_once 정렬 결과가 먼저 필요합니다')
        start_z = float(self._current_hardware_transform()[2, 3])
        final_target_z = self._aligned_hardware_final_target_z
        if start_z <= final_target_z:
            self._publish(
                'EXECUTED: 현재 Z가 P제어 목표 Z 이하이므로 하강을 생략합니다: '
                f'current_z={start_z * 1000.0:.1f}mm, '
                f'target_z={final_target_z * 1000.0:.1f}mm'
            )
            return

        self._publish(
            '연속 Z P제어 하강 시작: '
            f'current_z={start_z * 1000.0:.1f}mm, '
            f'target_z={final_target_z * 1000.0:.1f}mm, '
            f'z_kp={self._final_z_p_z_kp:.2f}, '
            f'max_step={self._final_z_descent_step * 1000.0:.1f}mm, '
            f'max_cycles={self._final_z_p_max_cycles}'
        )
        completed_cycles = 0
        stop_reason = '목표 Z 도달'
        for cycle in range(1, self._final_z_p_max_cycles + 1):
            current_z = float(self._current_hardware_transform()[2, 3])
            remaining = current_z - final_target_z
            if remaining < self._final_z_descent_minimum_progress:
                stop_reason = '포트 기반 최종 Z 도달'
                break
            self._publish(
                f'연속 Z P제어 {cycle}/{self._final_z_p_max_cycles}: '
                f'current_z={current_z * 1000.0:.1f}mm, '
                f'remaining={remaining * 1000.0:.1f}mm'
            )
            self._execute_descend_z_p_once()
            completed_cycles = cycle
        else:
            stop_reason = '최대 반복 횟수 도달'

        final_z = float(self._current_hardware_transform()[2, 3])
        self._publish(
            'EXECUTED: 연속 Z P제어 하강 종료: '
            f'cycles={completed_cycles}, '
            f'start_z={start_z * 1000.0:.1f}mm, '
            f'final_z={final_z * 1000.0:.1f}mm, '
            f'target_z={final_target_z * 1000.0:.1f}mm, '
            f'reason={stop_reason}'
        )

    @staticmethod
    def _wrapped_error_deg(target_deg: float, actual_deg: float) -> float:
        return (target_deg - actual_deg + 180.0) % 360.0 - 180.0

    def _finalize_coupled_alignment(
        self,
        frozen_xy,
        observation_transform,
        *,
        context: str,
    ) -> float:
        """XY와 초기 관측 Roll/Pitch를 함께 만족할 때만 완료한다."""
        for cycle in range(self._final_coupled_validation_max_cycles + 1):
            current = self._current_flange()
            residual = xy_residual_m(current[:2, 3], frozen_xy)
            roll_error, pitch_error = self._roll_pitch_errors(
                current,
                observation_transform,
            )
            xy_ok = residual <= self._tracking_tolerance
            roll_pitch_ok = (
                max(roll_error, pitch_error)
                <= self._roll_pitch_tolerance
            )
            self._publish(
                f'{context} 결합 확인 '
                f'{cycle}/{self._final_coupled_validation_max_cycles}: '
                f'xy={residual * 1000.0:.1f}mm/'
                f'{self._tracking_tolerance * 1000.0:.1f}mm, '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg/'
                f'{self._roll_pitch_tolerance:.2f}deg'
            )
            if xy_ok and roll_pitch_ok:
                self._publish(
                    f'{context} 결합 수렴 완료: '
                    f'xy={residual * 1000.0:.1f}mm, '
                    f'roll={roll_error:.2f}deg, '
                    f'pitch={pitch_error:.2f}deg'
                )
                return residual
            if cycle >= self._final_coupled_validation_max_cycles:
                raise RuntimeError(
                    f'{context} 결합 보정 최대 횟수 후에도 허용오차 초과: '
                    f'xy={residual * 1000.0:.1f}mm, '
                    f'roll={roll_error:.2f}deg, '
                    f'pitch={pitch_error:.2f}deg'
                )
            if not roll_pitch_ok:
                self._recover_roll_pitch(
                    observation_transform,
                    f'{context} 결합 보정 {cycle + 1}',
                )
            residual = xy_residual_m(
                self._current_flange()[:2, 3],
                frozen_xy,
            )
            if residual > self._tracking_tolerance:
                self._refine_xy_to_frozen_target(
                    frozen_xy,
                    observation_transform,
                    phase=f'coupled_xy_refine_{cycle + 1}',
                    initial_improvement=None,
                )
        raise RuntimeError(f'{context} 결합 정렬 내부 상태 오류')

    def _refine_xy_to_frozen_target(
        self,
        frozen_xy,
        observation_transform,
        *,
        phase: str,
        initial_improvement: float | None,
    ) -> float:
        residual = xy_residual_m(self._current_flange()[:2, 3], frozen_xy)
        improvement = initial_improvement
        cycle = 0
        while residual > self._tracking_tolerance:
            if cycle >= self._refine_max_cycles:
                raise RuntimeError(
                    f'{phase} 최대 {self._refine_max_cycles}회 후에도 '
                    f'XY 잔여오차가 {residual * 1000.0:.1f}mm입니다'
                )
            if (
                improvement is not None
                and improvement < self._minimum_improvement
            ):
                message = (
                    '로봇좌표 XY 오차 감소 부족: '
                    f'improvement={improvement * 1000.0:+.1f}mm, '
                    f'minimum={self._minimum_improvement * 1000.0:.1f}mm'
                )
                if self._stop_on_insufficient_improvement:
                    raise RuntimeError(message)
                self._publish(
                    f'WARNING: {message}. 최대 보정 횟수까지 계속합니다'
                )
            cycle += 1
            current = self._current_flange()
            hardware_current = self._current_hardware_transform()
            if self._proportional_control:
                step_xy, distance = proportional_xy_target(
                    current[:2, 3],
                    frozen_xy,
                    self._robot_xy_kp,
                    self._refine_maximum_step,
                    self._minimum_xy_step,
                )
            else:
                step_xy, distance = limited_xy_target(
                    current[:2, 3], frozen_xy, self._refine_maximum_step
                )
            # frozen X/Y는 PBVS와 flange TF가 표현하는 g_base 절대 목표를
            # 그대로 사용한다. Z/R/P/Yaw는 TF endpoint를 복사하지 않고
            # 제조사 get_coords의 최신 실제 자세를 사용해, 큰 coarse 뒤
            # 실제로 내려간 Z에서 다시 높은 TF Z를 요구하지 않는다.
            correction_target = hardware_current.copy()
            correction_target[:2, 3] = step_xy
            self._publish(
                f'{phase} {cycle}/{self._refine_max_cycles}: '
                f'to_frozen_target={distance * 1000.0:.1f}mm, '
                'endpoint_z_rpy=manufacturer_actual'
            )
            previous_residual = residual
            self._send_cartesian_transform(
                correction_target,
                f'{phase}_{cycle}',
                lock_z=False,
                lock_roll_pitch=False,
            )
            time.sleep(self._settle)
            self._recover_roll_pitch(
                observation_transform,
                f'{phase} {cycle} 후',
            )
            residual = xy_residual_m(
                self._current_flange()[:2, 3], frozen_xy
            )
            improvement = previous_residual - residual
            self._publish(
                f'{phase} {cycle} 후 XY 잔여오차='
                f'{residual * 1000.0:.1f}mm, '
                f'improvement={improvement * 1000.0:+.1f}mm'
            )
        return residual

    def _send_cartesian_transform(
        self,
        transform,
        label: str,
        *,
        lock_z: bool | None = None,
        lock_roll_pitch: bool | None = None,
        speed: int | None = None,
        mode: int | None = None,
        accept_timeout_xy_residual_m: float | None = None,
    ) -> PoseStamped:
        effective_lock_z = self._lock_z if lock_z is None else lock_z
        effective_lock_roll_pitch = (
            self._lock_roll_pitch
            if lock_roll_pitch is None
            else lock_roll_pitch
        )
        effective_speed = self._cartesian_speed if speed is None else speed
        effective_mode = self._cartesian_mode if mode is None else mode
        target = PoseStamped()
        target.header.frame_id = self._base_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose = transform_to_pose(transform)
        roll, pitch, yaw = rotation_to_rpy_degrees(transform[:3, :3])
        self._publish(
            f'{self._warning_delay:.1f}초 후 {label}: '
            f'xyz=[{transform[0, 3] * 1000.0:+.1f}, '
            f'{transform[1, 3] * 1000.0:+.1f}, '
            f'{transform[2, 3] * 1000.0:+.1f}]mm, '
            f'rpy=[{roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}]deg, '
            f'speed={effective_speed}, mode={effective_mode}, '
            f'lock_z={effective_lock_z}, '
            f'lock_roll_pitch={effective_lock_roll_pitch}'
        )
        time.sleep(self._warning_delay)
        return self._send_cartesian(
            target,
            lock_z=effective_lock_z,
            lock_roll_pitch=effective_lock_roll_pitch,
            speed=effective_speed,
            mode=effective_mode,
            accept_timeout_xy_residual_m=accept_timeout_xy_residual_m,
        )

    def _send_cartesian(
        self,
        target: PoseStamped,
        *,
        lock_z: bool,
        lock_roll_pitch: bool,
        speed: int,
        mode: int,
        accept_timeout_xy_residual_m: float | None,
    ) -> PoseStamped:
        if not self._cartesian_action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {CARTESIAN_ACTION}')
        goal = CartesianMove.Goal()
        goal.target = target
        goal.speed = speed
        goal.mode = mode
        goal.lock_z = lock_z
        goal.lock_roll_pitch = lock_roll_pitch
        handle = self._wait_future(
            self._cartesian_action.send_goal_async(goal), 5.0
        )
        if not handle.accepted:
            raise RuntimeError('Cartesian 목표가 거절됐습니다')
        wrapped = self._wait_future(
            handle.get_result_async(), self._cartesian_timeout
        )
        if not wrapped.result.success:
            message = wrapped.result.message
            if (
                accept_timeout_xy_residual_m is not None
                and 'Cartesian 목표에 도달하지 못했습니다' in message
            ):
                target_transform = pose_to_transform(target.pose)
                actual_transform = pose_to_transform(
                    wrapped.result.actual.pose
                )
                residual = xy_residual_m(
                    actual_transform[:2, 3],
                    target_transform[:2, 3],
                )
                if residual <= accept_timeout_xy_residual_m:
                    self._publish(
                        'WARNING: coarse 부분 도달을 수용하고 현재 자세에서 '
                        'XY refine을 계속합니다: '
                        f'xy_residual={residual * 1000.0:.1f}mm, '
                        f'limit='
                        f'{accept_timeout_xy_residual_m * 1000.0:.1f}mm'
                    )
                    return wrapped.result.actual
            raise RuntimeError(message)
        return wrapped.result.actual

    def _execute_initial_yaw(self, initial_axis: float) -> None:
        with self._lock:
            joints = (
                None
                if self._latest_joints is None
                else list(self._latest_joints)
            )
            joints_received_at = self._joints_received_at
        self._fresh(joints_received_at, '/joint_states')
        if joints is None:
            raise ValueError('Joint6 Yaw용 관절 입력이 없습니다')
        yaw_step, converged = calibrated_keypoint_joint_step_rad(
            initial_axis,
            self._desired_axis,
            self._keypoint_joint_gain,
            self._maximum_yaw_step,
            self._yaw_tolerance,
            self._keypoint_command_sign,
        )
        if converged:
            self._publish(
                f'초기 관측 Yaw {initial_axis:+.2f}deg가 허용값 안이므로 '
                'Joint6 회전을 생략합니다'
            )
            return
        if abs(math.degrees(yaw_step)) < self._minimum_yaw_step:
            self._publish(
                f'Joint6 Yaw 이동 {math.degrees(yaw_step):+.2f}deg가 '
                f'최소 유효값 {self._minimum_yaw_step:.2f}deg 미만이라 '
                '회전을 생략합니다'
            )
            return
        target_joints = list(joints)
        target_joints[-1] = joint6_yaw_target_rad(
            joints[-1],
            yaw_step,
            direction=self._joint6_direction,
            limit_deg=self._joint6_limit,
        )
        delta_deg = math.degrees(target_joints[-1] - joints[-1])
        self._publish(
            f'{self._warning_delay:.1f}초 후 초기 관측 Yaw: '
            f'image={initial_axis:+.2f}deg, '
            f'joint6_delta={delta_deg:+.2f}deg'
        )
        time.sleep(self._warning_delay)
        self._send_joint_target(target_joints)

    def _send_joint_target(
        self,
        target_joints: list[float],
        *,
        motion_seconds: float | None = None,
    ) -> None:
        effective_motion_seconds = (
            self._joint6_move_seconds
            if motion_seconds is None
            else motion_seconds
        )
        if not self._joint_action.wait_for_server(timeout_sec=3.0):
            raise RuntimeError(f'action server가 없습니다: {JOINT_ACTION}')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(
            seconds=effective_motion_seconds
        ).to_msg()
        goal.trajectory.points = [point]
        handle = self._wait_future(
            self._joint_action.send_goal_async(goal), 5.0
        )
        if not handle.accepted:
            raise RuntimeError('Joint6 목표가 거절됐습니다')
        wrapped = self._wait_future(
            handle.get_result_async(), effective_motion_seconds + 20.0
        )
        if (
            wrapped.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                wrapped.result.error_string
                or f'Joint6 error={wrapped.result.error_code}'
            )

    def _current_flange(self):
        try:
            message = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._flange_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise ValueError(f'현재 flange TF를 읽지 못했습니다: {error}') from error
        transform = message.transform
        return make_transform(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def _current_hardware_transform(self):
        with self._lock:
            message = self._latest_hardware_pose
            received_at = self._hardware_pose_received_at
        if message is None or received_at is None:
            raise ValueError(
                '제조사 Cartesian pose가 없습니다: '
                '/robot_arm/cartesian_pose_actual'
            )
        age = time.monotonic() - received_at
        if not 0.0 <= age <= self._hardware_pose_maximum_age:
            raise ValueError(
                '제조사 Cartesian pose가 오래됐습니다: '
                f'age={age:.3f}s, limit={self._hardware_pose_maximum_age:.3f}s'
            )
        return pose_to_transform(message.pose)

    def _wait_for_new_hardware_transform(
        self,
        previous_received_at: float | None,
        *,
        timeout_seconds: float,
    ):
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            with self._lock:
                received_at = self._hardware_pose_received_at
            if (
                received_at is not None
                and (
                    previous_received_at is None
                    or received_at > previous_received_at
                )
            ):
                return self._current_hardware_transform()
            time.sleep(0.05)
        raise RuntimeError(
            '관절 이동 후 새 제조사 Cartesian pose를 받지 못했습니다: '
            f'timeout={timeout_seconds:.1f}s'
        )

    def _roll_pitch_errors(
        self,
        actual_transform,
        reference_transform,
    ) -> tuple[float, float]:
        actual_rpy = rotation_to_rpy_degrees(actual_transform[:3, :3])
        reference_rpy = rotation_to_rpy_degrees(
            reference_transform[:3, :3]
        )
        roll_error = abs(
            (actual_rpy[0] - reference_rpy[0] + 180.0) % 360.0 - 180.0
        )
        pitch_error = abs(
            (actual_rpy[1] - reference_rpy[1] + 180.0) % 360.0 - 180.0
        )
        return roll_error, pitch_error

    def _recover_roll_pitch(
        self,
        observation_transform,
        context: str,
    ) -> None:
        for cycle in range(self._roll_pitch_recovery_max_cycles + 1):
            current = self._current_flange()
            roll_error, pitch_error = self._roll_pitch_errors(
                current,
                observation_transform,
            )
            if max(roll_error, pitch_error) <= self._roll_pitch_tolerance:
                self._publish(
                    f'{context} 초기 관측 Roll/Pitch 확인 완료: '
                    f'roll_error={roll_error:.2f}deg, '
                    f'pitch_error={pitch_error:.2f}deg'
                )
                return
            if not self._enable_roll_pitch_recovery:
                raise RuntimeError(
                    f'{context} 초기 관측 Roll/Pitch 허용오차 초과: '
                    f'roll={roll_error:.2f}deg, '
                    f'pitch={pitch_error:.2f}deg, '
                    f'limit={self._roll_pitch_tolerance:.2f}deg'
                )
            if cycle >= self._roll_pitch_recovery_max_cycles:
                raise RuntimeError(
                    f'{context} Roll/Pitch 복구 '
                    f'{self._roll_pitch_recovery_max_cycles}회 후에도 '
                    f'허용오차 초과: roll={roll_error:.2f}deg, '
                    f'pitch={pitch_error:.2f}deg, '
                    f'limit={self._roll_pitch_tolerance:.2f}deg'
                )
            recovery_target = current.copy()
            if self._proportional_control:
                recovery_target[:3, :3] = (
                    apply_proportional_observation_roll_pitch_with_current_yaw(
                        current,
                        observation_transform,
                        self._roll_pitch_kp,
                        self._pitch_correction_gain,
                    )[:3, :3]
                )
            else:
                recovery_target[:3, :3] = (
                    apply_proportional_observation_roll_pitch_with_current_yaw(
                        current,
                        observation_transform,
                        1.0,
                        self._pitch_correction_gain,
                    )[:3, :3]
                )
            attempt = cycle + 1
            self._publish(
                f'{context} Roll/Pitch 허용오차 초과: '
                f'roll={roll_error:.2f}deg, pitch={pitch_error:.2f}deg. '
                f'pitch_gain={self._pitch_correction_gain:.2f}. '
                f'자세 복구 {attempt}/'
                f'{self._roll_pitch_recovery_max_cycles} 후 XY를 재계산합니다'
            )
            self._send_cartesian_transform(
                recovery_target,
                f'roll_pitch_recovery_{attempt}',
                lock_z=False,
                lock_roll_pitch=False,
            )
            time.sleep(self._settle)

    def _wait_future(self, future, timeout_seconds: float):
        deadline = time.monotonic() + timeout_seconds
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                raise TimeoutError('ROS action 응답 시간이 초과됐습니다')
            time.sleep(0.05)
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrozenTargetExecutorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
