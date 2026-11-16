"""Single supervised straight approach using the existing RoboMaster ROS driver."""
import math
import signal
import time
import subprocess
import sys
import json
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range, Imu, JointState
from std_msgs.msg import Bool, String
from .motion_model import wrap_angle
from .state_estimator import StateEstimator
from .sensor_measurements import fresh, valid_range, tof_displacement, safe_command, forward_h
from .imu_processing import gyro_measurement, GyroBias
from .sensor_measurements import wheel_velocity, guard_displacement, guard_h, SettlingCheck
from .trial_logger import TrialLogger, STATE_NAMES, VARIANCE_NAMES

DEFAULTS = dict(
    enable_motion=False, estimation_mode='wheel_imu', start_distance_m=2.0,
    start_reference_verified=False, reference_point='front_guard',
    arrival_width_fraction=0.20, nominal_stopping_distance_m=0.20, physical_target_distance_m=0.26,
    tag_id=3, tag_family='tag36h11', tag_size_m=0.15,
    image_topic='camera/image_color', camera_info_topic='camera/camera_info',
    camera_calibration_file='',
    odom_topic='odom', tof_topic='range_0', imu_topic='imu', cmd_topic='cmd_vel',
    connected_topic='connected', arm_topic='arm_position', driver_node='robomaster',
    output_dir='~/ros2_ws/src/rm_phase3_odometry/results',
    generate_movement_plots=True,
    show_movement_plots=True,
    camera_timeout_s=1.0, detection_timeout_s=0.20, odom_timeout_s=0.3,
    tof_timeout_s=0.3, imu_timeout_s=0.3, max_dt_s=0.25,
    min_image_interval_s=0.08, control_rate_hz=20.,
    forward_speed_mps=0.14, bearing_gain=1.6, max_angular_speed_radps=0.30,
    steering_deadband_rad=0.01, min_angular_speed_radps=0.05,
    max_heading_deviation_rad=0.30, max_bearing_rad=0.30,
    max_elapsed_s=30., max_travel_m=2.2, max_lateral_speed_mps=0.05,
    max_forward_variance_m2=0.25, max_heading_variance_rad2=0.15,
    velocity_variance_m2ps2=0.0025, omega_variance_rad2ps2=0.01,
    wheel_velocity_variance_m2ps2=0.0025, wheel_omega_variance_rad2ps2=0.01,
    velocity_gate_nis=16., input_a_variance_m2ps4=0.25,
    input_alpha_variance_rad2ps4=1.,
    process_density=[0.0001, 0.0001, 0.0001, 0.0025, 0.01],
    tof_correction_enabled=False, tof_geometry_verified=False,
    tof_target_surface='', tof_offset_verified=False, tof_reference_offset_m=0.,
    tof_expected_frame='tof_0_link', tof_variance_m2=0.0025,
    tof_gate_nis=9., tof_max_heading_rad=0.10, tof_max_sync_s=0.08,
    tof_max_lateral_m=0.05, tof_initial_tolerance_m=0.10,
    tof_obstacle_protection_enabled=False, tof_obstacle_geometry_verified=False,
    tof_obstacle_distance_m=0.12,
    require_arm_pose=True, arm_x_m=0.105, arm_z_m=0.080,
    arm_tolerance_m=0.003, arm_timeout_s=0.5,
    base_frame='base_link', wheel_topic='joint_states_p',
    sensor_conventions_verified=False, guard_offset_verified=False,
    guard_offset_xy_m=[0., 0.], imu_bias_samples=50, imu_calibration_use_odom=False,
    suppress_stationary_wheel_noise=False,
    use_saved_gyro_bias=False, saved_gyro_bias_z_radps=0.,
    target_width_navigation=False,
    gyro_rate_variance_rad2ps2=0.01, gyro_gate_nis=9., wheel_timeout_s=.3, wheel_max_sync_s=.08,
    wheel_sync_alpha_bound_radps2=4.,
    stationary_linear_mps=.005, stationary_angular_radps=.03,
    settling_duration_s=.5, settling_timeout_s=2.,
)


def stamp_seconds(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9


def yaw_of(q):
    values = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values)-1) > 0.01:
        raise ValueError('Invalid orientation quaternion')
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def centering_rate(error_rad, gain, deadband_rad, minimum_radps, maximum_radps):
    """Signed yaw correction (rad/s) from bearing/heading error (rad)."""
    if not math.isfinite(error_rad) or abs(error_rad) <= deadband_rad:
        return 0.
    magnitude = min(maximum_radps, max(minimum_radps, gain*abs(error_rad)))
    return math.copysign(magnitude, error_rad)


class Phase3Nav(Node):
    def __init__(self):
        super().__init__('phase3_odometry')
        self.p = {key: self.declare_parameter(key, value).value for key, value in DEFAULTS.items()}
        self.check_parameters()
        self.begin = self.now_s()
        self.wall_begin = time.monotonic()
        self.motion_start_wall = None
        self.plot_executor = None
        self.camera_hold_wall = None
        self.last_tick_wall = self.wall_begin
        self.started = False
        self.state = 'WAITING'
        self.stop_reason = ''
        self.stop_wall = None
        self.settled_wall = None
        self.finalized = False
        self.estimator_fault = False
        self.estimator = None
        self.odom = None
        self.odom_origin = None
        self.odom_stamp = self.camera_stamp = self.detect_stamp = self.tof_stamp = self.imu_stamp = -1.
        self.last_image_stamp = -1.
        self.odom_received_wall = self.camera_received_wall = -1.
        self.connected = False
        self.driver_config_ok = False
        self.driver_query = None
        self.arm_stamp = -1.
        self.arm_ok = False
        self.width = 0.
        self.bearing = self.camera_distance = math.nan
        self.tag_heading_reference = None
        self.steering_error = math.nan
        self.tag_visible = False
        self.tof = math.nan
        self.tof_valid = False
        self.tof_frame_ok = False
        self.tof_initial_ok = False
        self.tof_last_used = -1.
        self.tof_geometry_valid = False
        self.imu = [math.nan]*6
        self.imu_quaternion = [math.nan]*4
        self.imu_frame = ''
        self.wheel_only_x = np.zeros(5)
        self.imu_valid = False
        self.imu_bias = GyroBias(self.p['imu_bias_samples'])
        if self.p['use_saved_gyro_bias']:
            bias = float(self.p['saved_gyro_bias_z_radps'])
            if not math.isfinite(bias) or abs(bias) > .1:
                raise ValueError('SAVED_GYRO_BIAS_INVALID')
            # This planar estimator uses only the vertical gyro correction.
            self.imu_bias.gyro = np.array([0., 0., bias])
            self.imu_bias.variance = np.full(3, .03**2)
            self.imu_bias.ready = True
        self.input_a = self.input_rate = math.nan
        self.imu_used_stamp = -1.
        self.gyro_correction_epoch = None
        self.gyro_measurement_variance = math.nan
        self.gyro_last_result = None
        self.gyro_last_accepted_stamp = -1.
        self.wheel_stamp = self.wheel_used_stamp = -1.
        self.wheel_raw = np.full(4, np.nan)
        self.wheel_body = np.full(3, np.nan)
        self.wheel_valid = False
        self.settling = SettlingCheck(self.p['stationary_linear_mps'],
            self.p['stationary_angular_radps'], self.p['settling_duration_s'])
        self.first_stop_stamp = None
        self.stationary_confirmed = False
        self.v_cmd = self.w_cmd = 0.
        self.camera_calibrated = False
        self.logger = TrialLogger(self.p['output_dir'], dict(self.p,
            architecture={'wheel_imu': 'ESC odometry prediction; independent gyro correction',
                          'wheel_only': 'ESC odometry prediction without corrections',
                          'odom_velocity': 'Original constant velocity prediction; driver odom velocity correction'}[self.p['estimation_mode']],
            marker_type='AprilTag, not QR', timestamp_source='ROS message header, driver receipt stamp',
            covariance_state_order=list(STATE_NAMES),
            wheel_input_units=['m/s','rad/s'],
            gyro_gain_row_units=['m*s/rad','m*s/rad','s','m/rad','1']))
        self.get_logger().info(f'Trial data: {self.logger.directory}; enable_motion={self.p["enable_motion"]}')
        self.record_commands = True
        self.cmd_pub = self.create_publisher(Twist, self.p['cmd_topic'], 10)
        self.state_pub = self.create_publisher(String, 'phase3/navigation_state', 1)
        self.create_subscription(String, 'phase3/camera_observation', self.on_camera_observation, 1)
        for message, topic, callback in (
                (Odometry, 'odom_topic', self.on_odom), (Range, 'tof_topic', self.on_tof),
                (Imu, 'imu_topic', self.on_imu), (JointState, 'wheel_topic', self.on_wheels), (PointStamped, 'arm_topic', self.on_arm)):
            self.create_subscription(message, self.p[topic], callback, qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, self.p['connected_topic'], self.on_connected, qos)
        self.driver_parameters = AsyncParameterClient(self, self.p['driver_node'])
        self.create_timer(.1, self.verify_driver_configuration)
        self.create_timer(1/self.p['control_rate_hz'], self.tick)
        self.create_timer(2., self.report_status)
        self.publish_command(0., 0., False)

    def check_parameters(self):
        p = self.p
        if p['estimation_mode'] not in ('odom_velocity', 'wheel_imu', 'wheel_only'):
            raise ValueError('Unknown estimator architecture')
        if p['estimation_mode'] != 'odom_velocity' and p['tof_correction_enabled']:
            raise ValueError('Optional ToF framework remains legacy-only; verify geometry before use')
        if (len(p['guard_offset_xy_m']) != 2 or not np.isfinite(p['guard_offset_xy_m']).all()
                or p['imu_bias_samples'] < 10):
            raise ValueError('Invalid guard geometry or calibration batch')
        if not 1 <= p['start_distance_m'] <= 2 or p['reference_point'] not in ('front_guard', p['base_frame']):
            raise ValueError('Use measured start distance 1–2 m and a documented physical reference')
        if not math.isfinite(p['arrival_width_fraction']) or not 0 < p['arrival_width_fraction'] < 1:
            raise ValueError('arrival_width_fraction must be between zero and one')
        positive = ['control_rate_hz', 'max_dt_s', 'camera_timeout_s', 'odom_timeout_s',
                    'tof_timeout_s', 'velocity_variance_m2ps2', 'omega_variance_rad2ps2',
                    'tof_variance_m2', 'max_elapsed_s', 'max_travel_m', 'velocity_gate_nis',
                    'tof_gate_nis', 'max_forward_variance_m2', 'max_heading_variance_rad2',
                    'imu_timeout_s', 'wheel_timeout_s', 'wheel_max_sync_s',
                    'gyro_rate_variance_rad2ps2', 'stationary_linear_mps',
                    'stationary_angular_radps', 'settling_duration_s', 'settling_timeout_s',
                    'wheel_sync_alpha_bound_radps2',
                    'nominal_stopping_distance_m', 'physical_target_distance_m', 'gyro_gate_nis',
                    'wheel_velocity_variance_m2ps2', 'wheel_omega_variance_rad2ps2']
        if any(not math.isfinite(p[key]) or p[key] <= 0 for key in positive):
            raise ValueError('Timeouts, rates, limits and variances must be finite and positive')
        if not 0 < p['forward_speed_mps'] <= 0.16 or not 0 < p['max_angular_speed_radps'] <= 0.30:
            raise ValueError('Command limits exceed Phase 2 operating range')
        if (not np.isfinite([p['bearing_gain'], p['steering_deadband_rad'], p['min_angular_speed_radps']]).all()
                or p['bearing_gain'] <= 0 or p['steering_deadband_rad'] < 0 or
                not 0 <= p['min_angular_speed_radps'] <= p['max_angular_speed_radps']):
            raise ValueError('Invalid centering gain, deadband or minimum yaw rate')
        if not np.isfinite([p['input_a_variance_m2ps4'], p['input_alpha_variance_rad2ps4'], p['tof_reference_offset_m']]).all() or min(p['input_a_variance_m2ps4'], p['input_alpha_variance_rad2ps4']) < 0:
            raise ValueError('Finite nonnegative input variances and finite ToF offset required')
        if (len(p['process_density']) != 5 or not np.isfinite(p['process_density']).all()
                or min(p['process_density']) < 0):
            raise ValueError('Five finite nonnegative process noise densities required')
        if p['tof_correction_enabled'] and not (p['tof_geometry_verified'] and
                p['tof_offset_verified'] and p['tof_target_surface'].strip()):
            raise ValueError('ToF correction requires measured mounting offset and verified target surface')
        if p['tof_obstacle_protection_enabled'] and not p['tof_obstacle_geometry_verified']:
            raise ValueError('Verify obstacle sensing geometry before enabling it')

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def report_status(self):
        if self.finalized:
            return
        self.state_pub.publish(String(data=self.state))
        now = self.now_s()
        camera_age = now-self.camera_stamp if self.camera_stamp > 0 else math.inf
        odom_age = now-self.odom_stamp if self.odom_stamp > 0 else math.inf
        self.get_logger().info(
            f'{self.state}: motion_enabled={self.p["enable_motion"]}; '
            f'camera_age={camera_age:.2f}s; odom_age={odom_age:.2f}s; '
            f'tag_visible={self.tag_visible}; bearing={self.bearing:.3f}rad; '
            f'width={self.width:.3f}; arm_ready={self.arm_ready(now)}; '
            f'wheel_max={np.max(np.abs(self.wheel_raw)):.3f}rad/s; '
            f'wheel_valid={self.wheel_valid}; '
            f'gyro_bias_samples={len(self.imu_bias.batch)}/{self.imu_bias.samples}; '
            f'command=({self.v_cmd:.3f}m/s,{self.w_cmd:.3f}rad/s)')

    def publish_command(self, v, w, healthy):
        previous = (self.v_cmd, self.w_cmd)
        self.v_cmd, self.w_cmd = safe_command(self.p['enable_motion'], healthy, v, w)
        msg = Twist()
        msg.linear.x, msg.angular.z = self.v_cmd, self.w_cmd
        self.last_command_stamp = self.now_s()
        self.cmd_pub.publish(msg)
        if getattr(self, 'record_commands', False) and previous != (self.v_cmd, self.w_cmd):
            self.record('command', self.last_command_stamp)

    def verify_driver_configuration(self):
        if self.driver_query is not None or not self.driver_parameters.services_are_ready():
            return
        self.driver_query = self.driver_parameters.get_parameters([
            'chassis.odom_twist_in_odom', 'chassis.odom_twist_from_pose_diff', 'chassis.timeout',
            'chassis.force_level', 'chassis.imu.include_orientation'])
        self.driver_query.add_done_callback(self.on_driver_configuration)

    def on_driver_configuration(self, future):
        try:
            values = future.result().values
            self.driver_config_ok = (len(values) == 5 and values[0].type == 1 and
                values[1].type == 1 and values[2].type == 3 and
                not values[0].bool_value and not values[1].bool_value and
                0 < values[2].double_value <= 0.5)
            if not self.driver_config_ok and self.started:
                self.stop('DRIVER_CONFIGURATION_UNSUPPORTED')
        except Exception:
            self.driver_config_ok = False
        finally:
            self.driver_query = None

    def on_connected(self, msg):
        self.connected = bool(msg.data)
        if not self.connected and self.started:
            self.stop('DRIVER_DISCONNECTED')

    def on_arm(self, msg):
        stamp = stamp_seconds(msg)
        if stamp <= self.arm_stamp or not fresh(stamp, self.now_s(), self.p['arm_timeout_s']):
            return
        self.arm_stamp = stamp
        self.arm_ok = (msg.header.frame_id == 'arm_base_link' and
                       math.isfinite(msg.point.x) and math.isfinite(msg.point.z) and
                       abs(msg.point.x-self.p['arm_x_m']) <= self.p['arm_tolerance_m']+1e-9 and
                       abs(msg.point.z-self.p['arm_z_m']) <= self.p['arm_tolerance_m']+1e-9)

    def on_camera_observation(self, msg):
        """Receive stamped camera results; this node never decodes or processes images."""
        try:
            observation = json.loads(msg.data)
            stamp = float(observation['stamp_s'])
            if stamp <= self.camera_stamp or not fresh(stamp, self.now_s(), self.p['camera_timeout_s']):
                return
            if int(observation['tag_id']) != self.p['tag_id']:
                raise ValueError('Camera target ID does not match navigation')
            visible = bool(observation['detected'])
            width = float(observation['bbox_width_fraction'])
            bearing = float(observation['bearing_rad']) if visible else math.nan
            distance = observation['distance_m']
            if not math.isfinite(width) or not 0 <= width <= 1 or (visible and not math.isfinite(bearing)):
                raise ValueError('Invalid camera observation')
            self.camera_stamp = stamp
            self.camera_received_wall = time.monotonic()
            self.camera_calibrated = bool(observation['calibrated'])
            self.tag_visible, self.width, self.bearing = visible, width, bearing
            self.camera_distance = float(distance) if distance is not None else math.nan
            if visible:
                self.detect_stamp = stamp
                if self.estimator is not None:
                    # Hold the latest tag-directed heading during missed detections,
                    # rather than undoing camera corrections toward the start yaw.
                    self.tag_heading_reference = wrap_angle(self.estimator.x[2]+bearing)
                if width >= self.p['arrival_width_fraction'] and self.started:
                    self.stop('ARRIVED_WIDTH')
        except (ValueError, KeyError, TypeError) as exc:
            self.tag_visible = False
            self.publish_command(0., 0., False)
            self.get_logger().warn(f'Camera observation rejected: {exc}', throttle_duration_sec=2.)

    def on_tof(self, msg):
        stamp = stamp_seconds(msg)
        if stamp <= self.tof_stamp or not fresh(stamp, self.now_s(), self.p['tof_timeout_s']):
            return
        self.tof_stamp, self.tof = stamp, float(msg.range)
        self.tof_frame_ok = msg.header.frame_id == self.p['tof_expected_frame']
        self.tof_valid = self.tof_frame_ok and valid_range(self.tof, msg.min_range, msg.max_range)
        if self.tof_valid and not self.started and self.p['tof_correction_enabled']:
            self.tof_initial_ok = abs(self.tof+self.p['tof_reference_offset_m']-
                                      self.p['start_distance_m']) <= self.p['tof_initial_tolerance_m']
        if self.tof_valid and self.p['tof_obstacle_protection_enabled'] and self.started:
            if self.tof <= self.p['tof_obstacle_distance_m']:
                self.stop('OBSTACLE_TOF')

    def on_wheels(self, msg):
        """Convert ESC rad/s; predict pose/covariance in wheel modes, then correct with gyro."""
        if self.finalized:
            return
        stamp, now = stamp_seconds(msg), self.now_s()
        if not any(name.endswith('_wheel_joint') for name in msg.name):
            return
        try:
            if stamp <= self.wheel_stamp or not fresh(stamp, now, self.p['wheel_timeout_s']):
                raise ValueError('WHEEL_TIMESTAMP_REJECTED')
            raw, body = wheel_velocity(list(msg.name), list(msg.velocity))
            if self.p.get('suppress_stationary_wheel_noise', False):
                # Retain raw ESC readings for diagnostics. Suppress their effect
                # only with independent fresh evidence of an uncommanded rest.
                gyro = np.asarray(self.imu[3:], dtype=float)
                if self.imu_bias.ready:
                    gyro = gyro-self.imu_bias.gyro
                if (self.v_cmd == 0. and self.w_cmd == 0.
                        and self.odom is not None
                        and fresh(self.odom_stamp, now, self.p['odom_timeout_s'])
                        and fresh(self.imu_stamp, now, self.p['imu_timeout_s'])
                        and abs(self.odom[3]) < .005 and abs(self.odom[4]) < .03
                        and np.isfinite(gyro).all() and np.max(np.abs(gyro)) < .03):
                    body = np.zeros(3)
            if not self.p.get('target_width_navigation', False) and abs(body[1]) > self.p['max_lateral_speed_mps']:
                raise ValueError('WHEEL_LATERAL_MOTION')
            from .motion_model import wheel_motion_model
            if self.wheel_stamp > 0:
                dt = stamp-self.wheel_stamp
                if not 0 < dt <= self.p['max_dt_s']:
                    raise ValueError('WHEEL_ODOMETRY_GAP')
                self.wheel_only_x = wheel_motion_model(self.wheel_only_x, body[[0, 2]], dt)
            else:
                if max(abs(body[0]), abs(body[2])) > .01:
                    raise ValueError('INITIAL_WHEELS_NOT_STATIONARY')
                self.wheel_only_x[3:] = body[[0, 2]]
            self.wheel_stamp, self.wheel_raw, self.wheel_body = stamp, raw, body
            self.wheel_valid = True
            if self.p['estimation_mode'] == 'odom_velocity':
                self.record('wheel_measurement', stamp)
                return
            if self.estimator is None:
                self.estimator = StateEstimator(np.zeros(5), process_density=self.p['process_density'],
                                                max_dt=self.p['max_dt_s'])
            Qwheel = np.diag([self.p['wheel_velocity_variance_m2ps2'], self.p['wheel_omega_variance_rad2ps2']])
            dt, predicted, P = self.estimator.predict_wheels_to(stamp, now, body[[0, 2]], Qwheel,
                                                              self.p['wheel_timeout_s'])
            self.wheel_used_stamp = stamp
            self.record('estimator_prediction', stamp, dt, predicted, P)
            self.apply_pending_gyro()
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.wheel_valid = False
            self.record('wheel_rejected', now)
            if self.started and self.p['estimation_mode'] != 'odom_velocity':
                self.estimator_fault = True
                self.stop(str(exc))

    def on_imu(self, msg):
        """Acquire independent gyro, estimate stationary bias, and request correction only."""
        stamp, now = stamp_seconds(msg), self.now_s()
        if self.finalized or stamp <= self.imu_stamp or not fresh(stamp, now, self.p['imu_timeout_s']):
            return
        if self.imu_stamp > 0 and stamp-self.imu_stamp > self.p['imu_timeout_s']:
            self.imu_bias.batch.clear()
        self.imu_stamp = stamp
        a, w, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        self.imu = [a.x, a.y, a.z, w.x, w.y, w.z]
        self.imu_quaternion, self.imu_frame = [q.x, q.y, q.z, q.w], msg.header.frame_id
        self.imu_valid = False
        if self.p['estimation_mode'] != 'wheel_imu':
            self.record('imu_diagnostic', stamp)
            return
        try:
            if not self.p['sensor_conventions_verified']:
                raise ValueError('SENSOR_CONVENTIONS_UNVERIFIED')
            if msg.angular_velocity_covariance[0] < 0:
                raise ValueError('GYRO_UNAVAILABLE')
            gyro = gyro_measurement(self.imu[3:], msg.header.frame_id, self.p['base_frame'])
            still = (self.wheel_valid and fresh(self.wheel_stamp, now, self.p['wheel_timeout_s'])
                     and np.max(np.abs(self.wheel_raw)) < .05)
            if self.p.get('imu_calibration_use_odom', False):
                still = (self.odom is not None
                         and fresh(self.odom_stamp, now, self.p['odom_timeout_s'])
                         and abs(self.odom[3]) < .005 and abs(self.odom[4]) < .03
                         and np.max(np.abs(gyro)) < .05)
            calibrating = not self.imu_bias.ready
            self.imu_bias.observe(gyro, still)
            if calibrating:
                self.imu_used_stamp = stamp  # Calibration data are not also independent corrections.
                self.record('imu_calibrating', stamp)
                return
            self.input_rate = float((gyro-self.imu_bias.gyro)[2])
            declared_variance = float(msg.angular_velocity_covariance[8])
            if not math.isfinite(declared_variance) or declared_variance < 0:
                raise ValueError('GYRO_COVARIANCE_INVALID')
            self.gyro_measurement_variance = max(declared_variance,
                self.p['gyro_rate_variance_rad2ps2']) + self.imu_bias.variance[2]
            self.imu_valid = True
            self.record('imu_measurement', stamp)
            self.apply_pending_gyro()
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.imu_bias.batch.clear()
            self.record('imu_rejected', stamp)
            if self.started:
                self.estimator_fault = True
                self.stop(str(exc))

    def apply_pending_gyro(self):
        """Consume one preceding fresh gyro per wheel epoch, using H=[0,0,0,0,1].

        Receipt-time skew is approximated by extra rate variance; no prediction
        occurs here, no wheel observation correction, and no driver odom fusion.
        """
        if (self.p['estimation_mode'] != 'wheel_imu' or self.estimator is None or
                not self.imu_valid or self.imu_stamp <= self.imu_used_stamp or
                self.gyro_correction_epoch == self.estimator.stamp):
            return
        skew = self.estimator.stamp-self.imu_stamp
        if not (0 <= skew <= self.p['wheel_max_sync_s'] and
                fresh(self.imu_stamp, self.now_s(), self.p['imu_timeout_s'])):
            return
        predicted, P = self.estimator.x.copy(), self.estimator.P.copy()
        variance = self.gyro_measurement_variance + (skew*self.p['wheel_sync_alpha_bound_radps2'])**2
        self.gyro_last_result = self.estimator.correct_gyro(self.input_rate, variance, self.p['gyro_gate_nis'])
        self.imu_used_stamp = self.imu_stamp
        self.gyro_correction_epoch = self.estimator.stamp
        self.record('estimator_correction', self.estimator.stamp, 0., predicted, P,
                    gyro=self.gyro_last_result)
        if self.gyro_last_result[0]:
            self.gyro_last_accepted_stamp = self.imu_stamp
        # A gated outlier is discarded by the estimator. Continue prediction
        # while the most recent accepted correction is still fresh; health_reason
        # stops motion if disagreement persists beyond the IMU timeout.

    def arm_ready(self, now):
        return (not self.p['require_arm_pose'] or
                (self.arm_ok and fresh(self.arm_stamp, now, self.p['arm_timeout_s'])))

    def on_odom(self, msg):
        if self.finalized:
            return
        now, stamp = self.now_s(), stamp_seconds(msg)
        try:
            if not fresh(stamp, now, self.p['odom_timeout_s']) or stamp <= self.odom_stamp:
                raise ValueError('ODOM_TIMESTAMP_REJECTED')
            pos, q, twist = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
            yaw = yaw_of(q)
            values = [pos.x, pos.y, yaw, twist.linear.x, twist.angular.z]
            if not np.isfinite(values+[twist.linear.y]).all():
                raise ValueError('ODOM_INVALID')
            if not self.p.get('target_width_navigation', False) and abs(twist.linear.y) > self.p['max_lateral_speed_mps']:
                raise ValueError('LATERAL_MOTION')
            if self.p['estimation_mode'] != 'odom_velocity':
                if self.odom_origin is None:
                    self.odom_origin = np.array(values[:3])
                self.odom, self.odom_stamp = values, stamp
                self.odom_received_wall = time.monotonic()
                self.record('odom_diagnostic', stamp)
                return
            if self.estimator is None:
                if abs(twist.linear.x) > 0.01 or abs(twist.angular.z) > 0.03:
                    raise ValueError('INITIAL_ROBOT_NOT_STATIONARY')
                self.odom_origin = np.array(values[:3])
                self.estimator = StateEstimator([0., 0., yaw, 0., 0.],
                    input_variance=(self.p['input_a_variance_m2ps4'], self.p['input_alpha_variance_rad2ps4']),
                    process_density=self.p['process_density'], max_dt=self.p['max_dt_s'])
            dt, predicted, predicted_P = self.estimator.predict_to(stamp, now, timeout=self.p['odom_timeout_s'])
            self.odom = values
            self.odom_stamp = stamp
            self.odom_received_wall = time.monotonic()
            # Use the driver's covariance, with conservative configurable lower bounds.
            C = np.asarray(msg.twist.covariance).reshape(6, 6)[np.ix_([0, 5], [0, 5])]
            floor = np.diag([self.p['velocity_variance_m2ps2'], self.p['omega_variance_rad2ps2']])
            if not np.isfinite(C).all() or not np.allclose(C, C.T) or np.linalg.eigvalsh(C).min() < 0:
                raise ValueError('ODOM_COVARIANCE_INVALID')
            R = C + floor
            velocity_result = self.estimator.correct_velocity(values[3], values[4], R, self.p['velocity_gate_nis'])
            if not velocity_result[0] and self.started:
                self.estimator_fault = True
                self.stop('ODOM_INNOVATION_REJECTED')
            tof_result = None
            self.tof_geometry_valid = self.tof_geometry_ok(stamp)
            # Consume each range once, at the next synchronized odom time. Never rewind the filter.
            if self.p['tof_correction_enabled'] and self.tof_geometry_valid and self.tof_stamp > self.tof_last_used:
                self.tof_last_used = self.tof_stamp
                z = tof_displacement(self.p['start_distance_m'], self.tof, self.p['tof_reference_offset_m'])
                tof_result = self.estimator.correct_tof(z, self.p['tof_variance_m2'], self.p['tof_gate_nis'])
            self.record('estimator_update', stamp, dt, predicted, predicted_P, velocity_result, tof_result)
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=1.)
            if self.started:
                self.estimator_fault = True
                self.stop(str(exc))
            self.record('odom_rejected', now)

    def tof_geometry_ok(self, stamp):
        if self.estimator is None or not self.tof_valid:
            return False
        x = self.estimator.x
        delta = x[:2]-self.estimator.origin[:2]
        lateral = -math.sin(self.estimator.theta0)*delta[0]+math.cos(self.estimator.theta0)*delta[1]
        return (self.p['tof_geometry_verified'] and self.p['tof_offset_verified'] and
                bool(self.p['tof_target_surface'].strip()) and self.tof_initial_ok and
                fresh(self.tof_stamp, self.now_s(), self.p['tof_timeout_s']) and
                0 <= stamp-self.tof_stamp <= self.p['tof_max_sync_s'] and
                abs(wrap_angle(x[2]-self.estimator.theta0)) <= self.p['tof_max_heading_rad'] and
                abs(lateral) <= self.p['tof_max_lateral_m'] and
                self.tag_visible and fresh(self.detect_stamp, self.now_s(), self.p['detection_timeout_s']))

    def health_reason(self, now):
        if not self.connected or self.cmd_pub.get_subscription_count() == 0:
            return 'DRIVER_MISSING'
        if not self.driver_config_ok:
            return 'DRIVER_CONFIGURATION_UNVERIFIED'
        if not fresh(self.odom_stamp, now, self.p['odom_timeout_s']) or time.monotonic()-self.odom_received_wall > self.p['odom_timeout_s']:
            return 'ODOM_STALE'
        if self.p['estimation_mode'] != 'odom_velocity':
            if not self.p['sensor_conventions_verified']:
                return 'SENSOR_CONVENTIONS_UNVERIFIED'
            if self.p['estimation_mode'] == 'wheel_imu':
                if not self.imu_bias.ready:
                    if not fresh(self.imu_stamp, now, self.p['imu_timeout_s']):
                        return 'IMU_STALE_OR_MISSING'
                    if not self.wheel_valid or not fresh(self.wheel_stamp, now, self.p['wheel_timeout_s']):
                        return 'IMU_CALIBRATION_WHEELS_INVALID_OR_STALE'
                    if not self.p.get('imu_calibration_use_odom', False) and np.max(np.abs(self.wheel_raw)) >= .05:
                        return 'IMU_CALIBRATION_REQUIRES_STATIONARY_WHEELS'
                    return f'IMU_CALIBRATING_{len(self.imu_bias.batch)}_OF_{self.imu_bias.samples}'
                if not self.imu_valid or not fresh(self.imu_stamp, now, self.p['imu_timeout_s']):
                    return 'IMU_NOT_READY_OR_STALE'
                if not fresh(self.imu_used_stamp, now, self.p['imu_timeout_s']):
                    return 'GYRO_SYNCHRONIZATION_MISSING'
                if not fresh(getattr(self, 'gyro_last_accepted_stamp', -1.), now,
                             self.p['imu_timeout_s']):
                    return 'GYRO_CORRECTION_NOT_READY'
            if not self.wheel_valid or not fresh(self.wheel_stamp, now, self.p['wheel_timeout_s']):
                return 'WHEELS_INVALID_OR_STALE'
            if not fresh(self.wheel_used_stamp, now, self.p['wheel_timeout_s']):
                return 'WHEEL_SYNCHRONIZATION_MISSING'
            if not self.p['guard_offset_verified'] or not self.p['start_reference_verified']:
                return 'PHYSICAL_REFERENCE_UNVERIFIED'
        if not self.arm_ready(now):
            return 'ARM_POSE_UNVERIFIED'
        if self.estimator is None or not self.camera_calibrated:
            return 'ESTIMATOR_OR_CALIBRATION_MISSING'
        try:
            self.estimator.validate()
        except ValueError:
            return 'ESTIMATOR_INVALID'
        H = guard_h(self.estimator.x, self.estimator.theta0, self.p['guard_offset_xy_m']) if self.p['reference_point'] == 'front_guard' else forward_h(self.estimator.theta0)
        if float((H@self.estimator.P@H.T)[0, 0]) > self.p['max_forward_variance_m2'] or self.estimator.P[2, 2] > self.p['max_heading_variance_rad2']:
            return 'ESTIMATOR_UNCERTAINTY'
        if not self.p.get('target_width_navigation', False) and abs(wrap_angle(self.estimator.x[2]-self.estimator.theta0)) > self.p['max_heading_deviation_rad']:
            return 'HEADING_DEVIATION'
        if not self.p.get('target_width_navigation', False) and (abs(self.estimator.forward()) > self.p['max_travel_m'] or (self.odom is not None and np.linalg.norm(np.array(self.odom[:2])-self.odom_origin[:2]) > self.p['max_travel_m'])):
            return 'TRAVEL_LIMIT'
        if self.p['tof_obstacle_protection_enabled'] and not (self.tof_valid and fresh(self.tof_stamp, now, self.p['tof_timeout_s'])):
            return 'TOF_SAFETY_STALE'
        if self.p['tof_correction_enabled'] and not self.tof_initial_ok:
            return 'TOF_INITIAL_REFERENCE_MISMATCH'
        if not fresh(self.camera_stamp, now, self.p['camera_timeout_s']) or time.monotonic()-self.camera_received_wall > self.p['camera_timeout_s']:
            return 'CAMERA_STALE'
        return ''

    def hold_camera(self, reason, wall):
        """Hold zero on missing observations without finalizing or resetting the trial."""
        self.publish_command(0., 0., False)
        self.state = 'HOLD_'+reason
        if self.camera_hold_wall is None:
            self.camera_hold_wall = wall
            self.get_logger().info(f'Navigation paused; waiting for sensor recovery: {reason}')
            self.record('camera_hold', self.now_s())

    def tick(self):
        wall, now = time.monotonic(), self.now_s()
        delayed = wall-self.last_tick_wall > self.p['odom_timeout_s']
        self.last_tick_wall = wall
        if self.finalized:
            self.publish_command(0., 0., False)
            return
        if self.stop_reason:
            self.publish_command(0., 0., False)
            if self.p['estimation_mode'] != 'odom_velocity':
                sensor_stamp, v, w = self.wheel_stamp, float(np.max(np.abs(self.wheel_raw)))*.05, self.wheel_body[2]
                valid = self.wheel_valid
                if self.p['estimation_mode'] == 'wheel_imu':
                    w = max(abs(w), abs(self.input_rate)) if self.imu_valid else math.nan
            else:
                sensor_stamp = self.odom_stamp
                v, w = self.odom[3:5] if self.odom is not None else (math.nan, math.nan)
                valid = self.odom is not None
            self.stationary_confirmed = self.settling.update(sensor_stamp, now,
                v if valid else math.nan, w, self.p['wheel_timeout_s'])
            if self.stationary_confirmed:
                self.settled_wall = self.settled_wall or wall
                self.finish()
            elif wall-self.stop_wall >= self.p['settling_timeout_s']:
                self.finish()
            return
        reason = self.health_reason(now)
        if self.started and delayed:
            reason = 'CONTROL_TIMER_DELAY'
        active_elapsed = (wall-self.motion_start_wall if self.motion_start_wall is not None else 0.)
        if self.camera_hold_wall is not None:
            active_elapsed -= wall-self.camera_hold_wall
        if not self.p.get('target_width_navigation', False) and self.motion_start_wall is not None and active_elapsed > self.p['max_elapsed_s']:
            self.stop('TIME_LIMIT')
            return
        if reason:
            self.publish_command(0., 0., False)
            if self.started:
                if reason in ('CAMERA_STALE', 'GYRO_CORRECTION_NOT_READY'):
                    self.hold_camera(reason, wall)
                else:
                    self.stop(reason)
            else:
                self.state = 'WAITING_'+reason
            return
        if self.camera_hold_wall is not None:
            self.get_logger().info('Sensors recovered; resuming navigation')
            if self.motion_start_wall is not None:
                self.motion_start_wall += wall-self.camera_hold_wall
            self.camera_hold_wall = None
            self.record('camera_recovered', now)
        visible = self.tag_visible and fresh(self.detect_stamp, now, self.p['detection_timeout_s'])
        if not visible:
            if self.p.get('target_width_navigation', False):
                self.hold_camera('TAG_NOT_VISIBLE', wall)
                return
            if not self.started:
                self.publish_command(0., 0., False)
                self.state = 'WAITING_TAG'
                return
            # Never steer from a stale bearing. After initial acquisition, a
            # missed detection uses the fixed approach heading and measured odom.
            remaining = min(self.p['start_distance_m']-self.estimator.forward(),
                            self.odom_remaining())
            if remaining <= self.p['nominal_stopping_distance_m']:
                self.stop('TAG_LOST_DISTANCE_LIMIT')
                return
            self.state = 'CHASING_ODOMETRY' if self.p['enable_motion'] else 'OBSERVING'
            reference = (self.tag_heading_reference if self.tag_heading_reference is not None
                         else self.estimator.theta0)
            self.steering_error = wrap_angle(reference-self.estimator.x[2])
            w = centering_rate(self.steering_error, self.p['bearing_gain'],
                               self.p['steering_deadband_rad'], self.p['min_angular_speed_radps'],
                               self.p['max_angular_speed_radps'])
            self.publish_command(self.p['forward_speed_mps'], w, True)
            return
        if self.width >= self.p['arrival_width_fraction']:
            self.stop('ARRIVED_WIDTH')
            return
        if not self.p.get('target_width_navigation', False) and abs(self.bearing) > self.p['max_bearing_rad']:
            self.publish_command(0., 0., False)
            if self.started:
                self.stop('BEARING_DEVIATION')
            else:
                self.state = 'WAITING_ALIGNMENT'
            return
        if not self.started:
            # Keep the stationary initialized reference; do not reset on tag dropout.
            self.started = True
        if self.p['enable_motion'] and self.motion_start_wall is None:
            self.motion_start_wall = wall
        self.state = 'CHASING' if self.p['enable_motion'] else 'OBSERVING'
        self.steering_error = self.bearing
        w = centering_rate(self.steering_error, self.p['bearing_gain'],
                           self.p['steering_deadband_rad'], self.p['min_angular_speed_radps'],
                           self.p['max_angular_speed_radps'])
        self.publish_command(self.p['forward_speed_mps'], w, True)

    def stop(self, reason):
        self.publish_command(0., 0., False)
        if not self.stop_reason:
            self.stop_reason = reason
            self.state = 'ARRIVED' if reason == 'ARRIVED_WIDTH' else 'STOPPED'
            self.stop_wall = time.monotonic()
            self.first_stop_stamp = getattr(self, 'last_command_stamp', self.now_s())
            self.get_logger().info(f'Trial stopping: {reason}')
            self.record('stop', self.now_s())
            if reason == 'ARRIVED_WIDTH' and self.p['generate_movement_plots']:
                self.start_movement_plots(show=self.p['show_movement_plots'])

    def reference_forward(self, x=None):
        """Guard displacement (m), including rotation of a verified base-to-guard offset."""
        if self.estimator is None:
            return math.nan
        x = self.estimator.x if x is None else x
        if self.p['reference_point'] == 'front_guard':
            if not self.p['guard_offset_verified'] and abs(wrap_angle(x[2]-self.estimator.theta0)) > 1e-6:
                return math.nan
            return guard_displacement(x, self.estimator.origin, self.p['guard_offset_xy_m'])
        return self.estimator.forward(x)

    def odom_remaining(self):
        if self.odom is None:
            return math.nan
        dx, dy = np.asarray(self.odom[:2])-self.odom_origin[:2]
        forward = dx*math.cos(self.odom_origin[2])+dy*math.sin(self.odom_origin[2])
        if self.p['reference_point'] == 'front_guard':
            if not self.p['guard_offset_verified'] and abs(wrap_angle(self.odom[2]-self.odom_origin[2])) > 1e-6:
                return math.nan
            forward = guard_displacement(self.odom[:3], self.odom_origin, self.p['guard_offset_xy_m'])
        return self.p['start_distance_m']-forward

    def record(self, event, stamp, dt=0., predicted=None, predicted_P=None, velocity=None, tof=None, gyro=None):
        now = self.now_s()
        row = dict(trial_number=self.logger.number, start_distance_m=self.p['start_distance_m'], timestamp_s=stamp, elapsed_s=stamp-self.begin, dt_s=dt, event=event,
            navigation_state=self.state, tag_id=self.p['tag_id'] if self.tag_visible else '',
            tag_detected=self.tag_visible, bbox_width_fraction=self.width,
            camera_distance_m=self.camera_distance, camera_bearing_rad=self.bearing,
            steering_heading_reference_rad=(self.tag_heading_reference
                                            if self.tag_heading_reference is not None else math.nan),
            steering_error_rad=self.steering_error,
            tof_raw_m=self.tof, tof_corrected_m=self.tof+self.p['tof_reference_offset_m'],
            tof_valid=self.tof_valid and fresh(self.tof_stamp, now, self.p['tof_timeout_s']),
            tof_age_s=now-self.tof_stamp if self.tof_stamp > 0 else math.nan,
            camera_stamp_s=self.camera_stamp if self.camera_stamp>0 else math.nan,
            tof_stamp_s=self.tof_stamp if self.tof_stamp>0 else math.nan,
            imu_stamp_s=self.imu_stamp if self.imu_stamp>0 else math.nan,
            command_v_mps=self.v_cmd, command_omega_radps=self.w_cmd,
            estimation_mode=self.p['estimation_mode'], active_updates='', stop_reason=self.stop_reason,
            odom_valid=fresh(self.odom_stamp, now, self.p['odom_timeout_s']),
            camera_valid=fresh(self.camera_stamp, now, self.p['camera_timeout_s']), connected=self.connected,
            imu_valid=fresh(self.imu_stamp, now, self.p['imu_timeout_s']) and np.isfinite(self.imu).all(),
            tof_geometry_valid=self.tof_geometry_valid,
            input_a_mps2=0. if self.p['estimation_mode']=='odom_velocity' else math.nan,
            input_alpha_radps2=0. if self.p['estimation_mode']=='odom_velocity' else math.nan,
            odom_only_remaining_m=self.odom_remaining())
        row.update(imu_valid=(self.imu_valid and fresh(self.imu_stamp, now, self.p['imu_timeout_s'])
                              if self.p['estimation_mode'] == 'wheel_imu' else False),
            imu_forward_acceleration_mps2=self.input_a, imu_yaw_rate_radps=self.input_rate,
            imu_accel_bias_x_mps2=math.nan,
            imu_accel_bias_y_mps2=math.nan, imu_accel_bias_z_mps2=math.nan,
            imu_gyro_bias_z_radps=self.imu_bias.gyro[2],
            wheel_stamp_s=self.wheel_stamp if self.wheel_stamp > 0 else math.nan,
            wheel_valid=self.wheel_valid and fresh(self.wheel_stamp, now, self.p['wheel_timeout_s']),
            wheel_v_mps=self.wheel_body[0], wheel_lateral_mps=self.wheel_body[1],
            wheel_omega_radps=self.wheel_body[2],
            wheel_only_heading_rad=self.wheel_only_x[2] if self.wheel_valid else math.nan,
            wheel_used_stamp_s=self.wheel_used_stamp if self.wheel_used_stamp>0 else math.nan,
            imu_used_stamp_s=self.imu_used_stamp if self.imu_used_stamp>0 else math.nan,
            gyro_measurement_variance_rad2ps2=self.gyro_measurement_variance,
            wheel_only_forward_m=(guard_displacement(self.wheel_only_x, np.zeros(5), self.p['guard_offset_xy_m'])
                                  if self.wheel_valid else math.nan),
            wheel_only_remaining_m=(self.p['start_distance_m']-guard_displacement(
                self.wheel_only_x, np.zeros(5), self.p['guard_offset_xy_m']) if self.wheel_valid else math.nan),
            imu_frame=self.imu_frame,
            settling_phase=('stationary_confirmed' if self.stationary_confirmed else
                            ('settling' if self.settling.begin is not None else 'braking')
                            if self.stop_reason else 'approach'),
            first_stop_command_stamp_s=self.first_stop_stamp,
            stationary_confirmed=self.stationary_confirmed)
        row.update(zip(['imu_qx', 'imu_qy', 'imu_qz', 'imu_qw'], self.imu_quaternion))
        row.update(zip(['wheel_fr_radps', 'wheel_fl_radps', 'wheel_rl_radps', 'wheel_rr_radps'], self.wheel_raw))
        row['odom_stamp_s'] = self.odom_stamp if self.odom_stamp > 0 else math.nan
        row.update(zip(['imu_ax_mps2', 'imu_ay_mps2', 'imu_az_mps2', 'imu_wx_radps', 'imu_wy_radps', 'imu_wz_radps'], self.imu))
        if self.odom is not None:
            row.update(zip(['odom_x_m', 'odom_y_m', 'odom_heading_rad', 'odom_v_mps', 'odom_omega_radps'], self.odom))
        if self.estimator is not None:
            if predicted is None:
                predicted, predicted_P = self.estimator.x, self.estimator.P
            for phase, x, P in [('predicted', predicted, predicted_P), ('corrected', self.estimator.x, self.estimator.P)]:
                row.update({f'{phase}_{name}': float(value) for name, value in zip(STATE_NAMES, x)})
                row.update({f'{phase}_P_{i}{j}': float(P[i, j]) for i in range(5) for j in range(5)})
                row.update({f'{phase}_variance_{name}': float(P[i, i]) for i, name in enumerate(VARIANCE_NAMES)})
                s = self.reference_forward(x)
                row[f'{phase}_forward_m'] = s
                row[f'{phase}_remaining_m'] = self.p['start_distance_m']-s
        active = []
        if velocity is not None:
            accepted, e, K, nis = velocity
            row.update(velocity_update_accepted=accepted, velocity_innovation_mps=e[0], omega_innovation_radps=e[1], velocity_nis=nis)
            row.update({f'velocity_K_{i}{j}': K[i, j] for i in range(5) for j in range(2)})
            if accepted:
                active.append('odom_velocity')
        if gyro is not None:
            accepted, e, K, nis = gyro
            row.update(gyro_update_accepted=accepted, gyro_innovation_radps=e[0], gyro_nis=nis)
            row.update({f'gyro_K_{i}': K[i, 0] for i in range(5)})
            if accepted:
                active.append('imu_gyro')
        if tof is not None:
            accepted, e, K, nis = tof
            row.update(tof_update_accepted=accepted, tof_innovation_m=e[0], tof_nis=nis)
            row.update({f'tof_K_{i}': K[i, 0] for i in range(5)})
            if accepted:
                active.append('tof_displacement')
        row['active_updates'] = '+'.join(active)
        self.logger.record(row)

    def finish(self):
        if self.finalized:
            return
        self.publish_command(0., 0., False)
        if self.stop_reason == 'ARRIVED_WIDTH':
            if self.estimator_fault:
                self.stop_reason = 'ARRIVAL_ESTIMATOR_FAULT'
            elif not self.stationary_confirmed:
                self.stop_reason = 'ARRIVAL_REST_UNVERIFIED'
        self.record('final', self.now_s())
        summary = dict(stop_reason=self.stop_reason or 'MANUAL_SHUTDOWN',
            first_stop_command_stamp_s=self.first_stop_stamp,
                       final_stationary_stamp_s=self.settling.confirmed_stamp if self.stationary_confirmed else '',
                       final_estimator_stamp_s=self.estimator.stamp if self.estimator else '',
                       stationary_confirmed=self.stationary_confirmed,
                       wheel_only_final_distance_m=(self.p['start_distance_m']-guard_displacement(
                           self.wheel_only_x, np.zeros(5), self.p['guard_offset_xy_m'])
                           if self.stationary_confirmed and self.wheel_valid else ''),
                       odom_only_final_distance_m=self.odom_remaining())
        if self.estimator is not None:
            x, P = self.estimator.x, self.estimator.P
            H = guard_h(self.estimator.x, self.estimator.theta0, self.p['guard_offset_xy_m']) if self.p['reference_point'] == 'front_guard' else forward_h(self.estimator.theta0)
            summary.update(estimated_final_distance_m=self.p['start_distance_m']-self.reference_forward(),
                final_x_m=x[0], final_y_m=x[1], final_heading_rad=x[2],
                final_forward_variance_m2=float((H@P@H.T)[0, 0]), final_P_xx_m2=P[0, 0],
                final_P_yy_m2=P[1, 1], final_P_heading_rad2=P[2, 2])
            if self.tof_geometry_ok(self.odom_stamp):
                summary['tof_final_distance_m'] = self.tof+self.p['tof_reference_offset_m']
        final_valid = (self.stationary_confirmed and not self.estimator_fault and
                       self.estimator is not None and self.estimator.stamp is not None and
                       fresh(self.estimator.stamp, self.now_s(), self.p['imu_timeout_s']))
        if self.p['estimation_mode'] == 'wheel_imu':
            final_valid = (final_valid and self.imu_valid and fresh(self.imu_used_stamp, self.now_s(),
                           self.p['imu_timeout_s']) and self.gyro_last_result is not None and self.gyro_last_result[0])
        final_valid = final_valid and math.isfinite(summary.get('estimated_final_distance_m', math.nan))
        summary['final_estimate_valid'] = final_valid
        if not final_valid:
            summary['unconfirmed_last_distance_m'] = summary.pop('estimated_final_distance_m', '')
        summary['estimation_mode'] = self.p['estimation_mode']
        self.logger.finish(summary)
        self.finalized = True
        self.get_logger().info(f'Trial finalized: {self.logger.directory}; restart manually for another trial')
        if self.p['generate_movement_plots']:
            self.start_movement_plots()

    def start_movement_plots(self, show=False):
        """Queue rendering outside ROS callbacks; arrival and final refresh run in order."""
        if self.odom is None and self.estimator is None:
            self.get_logger().info('Plots skipped: no valid estimator or odometry samples')
            return
        if self.plot_executor is None:
            self.plot_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='movement_plots')
        self.plot_executor.submit(self.render_movement_plots, show)
        self.get_logger().info(f'Movement plots generating in {self.logger.directory}/plots')

    def render_movement_plots(self, show):
        """Worker launches a non-ROS renderer; single worker prevents overlapping writes."""
        command = [sys.executable, '-m', 'rm_phase3_odometry.trial_analysis',
                   'plot', str(self.logger.directory)]
        if show:
            command.append('--show')
        with (self.logger.directory/'plot_generation.log').open('a') as output:
            try:
                subprocess.run(command, stdout=output, stderr=subprocess.STDOUT,
                               start_new_session=True, check=False)
            except OSError as exc:
                output.write(f'Could not generate movement plots: {exc}\n')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def request_shutdown(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    node = None
    try:
        node = Phase3Nav()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop('MANUAL_SHUTDOWN')
                node.finish()
            else:
                # ROS may already be shut down; the driver command timeout is the fallback.
                node.stop_reason = node.stop_reason or 'ROS_SHUTDOWN'
                original = node.publish_command
                node.publish_command = lambda *args: None
                node.finish()
                node.publish_command = original
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
