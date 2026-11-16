"""One exclusive directory per invocation; raw telemetry is append-only."""
import csv
import json
import os
from datetime import datetime, timezone
from pathlib import Path

STATE_NAMES = ('x_m', 'y_m', 'heading_rad', 'v_mps', 'omega_radps')
VARIANCE_NAMES = ('x_m2', 'y_m2', 'heading_rad2', 'v_m2ps2', 'omega_rad2ps2')
COLUMNS = [
    'trial_id', 'timestamp_s', 'elapsed_s', 'dt_s', 'event', 'navigation_state',
    'tag_id', 'tag_detected', 'bbox_width_fraction', 'camera_distance_m',
    'camera_bearing_rad', 'tof_raw_m', 'tof_corrected_m', 'tof_valid', 'tof_age_s',
    'steering_heading_reference_rad', 'steering_error_rad',
    'odom_x_m', 'odom_y_m', 'odom_heading_rad', 'odom_v_mps', 'odom_omega_radps',
    'odom_stamp_s',
    'command_v_mps', 'command_omega_radps', 'imu_ax_mps2', 'imu_ay_mps2',
    'imu_az_mps2', 'imu_wx_radps', 'imu_wy_radps', 'imu_wz_radps', 'imu_valid',
    'estimation_mode', 'active_updates', 'odom_valid', 'camera_valid', 'connected',
    'predicted_forward_m', 'corrected_forward_m', 'predicted_remaining_m',
    'corrected_remaining_m', 'odom_only_remaining_m', 'stop_reason',
    'velocity_innovation_mps', 'omega_innovation_radps', 'velocity_nis',
    'tof_innovation_m', 'tof_nis', 'tof_update_accepted', 'velocity_update_accepted',
    'tof_geometry_valid', 'tof_stamp_s', 'imu_stamp_s', 'camera_stamp_s',
    'input_a_mps2', 'input_alpha_radps2',
] + [f'{phase}_{name}' for phase in ('predicted', 'corrected') for name in STATE_NAMES]
# Full P and K let the student inspect cross-covariance, not just diagonal shrinkage.
COLUMNS += [f'{phase}_P_{i}{j}' for phase in ('predicted', 'corrected')
            for i in range(5) for j in range(5)]
COLUMNS += [f'{phase}_variance_{name}' for phase in ('predicted', 'corrected') for name in VARIANCE_NAMES]
COLUMNS += [f'velocity_K_{i}{j}' for i in range(5) for j in range(2)]
COLUMNS += [f'tof_K_{i}' for i in range(5)]
COLUMNS += ['imu_forward_acceleration_mps2', 'imu_yaw_rate_radps',
            'imu_accel_bias_x_mps2', 'imu_accel_bias_y_mps2', 'imu_accel_bias_z_mps2',
            'imu_gyro_bias_z_radps', 'wheel_fr_radps', 'wheel_fl_radps',
            'wheel_rl_radps', 'wheel_rr_radps', 'wheel_v_mps', 'wheel_lateral_mps',
            'wheel_omega_radps', 'wheel_stamp_s', 'wheel_valid',
            'imu_qx', 'imu_qy', 'imu_qz', 'imu_qw', 'imu_frame',
            'first_stop_command_stamp_s', 'stationary_confirmed', 'settling_phase',
            'wheel_only_forward_m', 'wheel_only_remaining_m', 'wheel_used_stamp_s']
COLUMNS += ['wheel_only_heading_rad', 'trial_number', 'start_distance_m', 'imu_used_stamp_s',
            'gyro_measurement_variance_rad2ps2', 'gyro_update_accepted',
            'gyro_innovation_radps', 'gyro_nis'] + [f'gyro_K_{i}' for i in range(5)]
SUMMARY_COLUMNS = [
    'trial_number', 'trial_id', 'enable_motion', 'start_distance_m', 'stopping_width_fraction',
    'estimation_mode', 'physical_target_distance_m', 'stopping_target_error_m',
    'estimated_final_distance_m', 'odom_only_final_distance_m', 'tof_final_distance_m',
    'final_x_m', 'final_y_m', 'final_heading_rad', 'final_forward_variance_m2',
    'final_P_xx_m2', 'final_P_yy_m2', 'final_P_heading_rad2', 'stop_reason',
    'actual_measured_final_distance_m', 'signed_error_m', 'absolute_error_m',
    'first_stop_command_stamp_s', 'final_stationary_stamp_s', 'final_estimator_stamp_s',
    'stationary_confirmed', 'final_estimate_valid', 'unconfirmed_last_distance_m', 'wheel_only_final_distance_m',
]


def write_summary(path, row):
    """Atomically replace derived summary, never the raw telemetry."""
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerow({key: row.get(key, '') for key in SUMMARY_COLUMNS})
    os.replace(temporary, path)


class TrialLogger:
    def __init__(self, root, metadata):
        root = Path(root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        numbers = []
        for path in root.glob('trial_*'):
            try:
                numbers.append(int(path.name.split('_')[1]))
            except (ValueError, IndexError):
                pass
        self.number = max(numbers, default=0)+1
        while True:
            self.trial_id = f'trial_{self.number:03d}_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
            self.directory = root/self.trial_id
            try:
                self.directory.mkdir(exist_ok=False)
                break
            except FileExistsError:
                self.number += 1
        metadata = dict(metadata, trial_id=self.trial_id, trial_number=self.number,
                        created_utc=datetime.now(timezone.utc).isoformat(),
                        measurement_source='independent physical measurement, initially blank')
        (self.directory/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
        self.stream = (self.directory/'telemetry.csv').open('x', newline='')
        self.writer = csv.DictWriter(self.stream, COLUMNS)
        self.writer.writeheader()
        self.stream.flush()
        self.closed = False
        self.summary = dict(trial_number=self.number, trial_id=self.trial_id,
                            enable_motion=metadata.get('enable_motion', False),
                            estimation_mode=metadata.get('estimation_mode', ''),
                            physical_target_distance_m=metadata.get('physical_target_distance_m', .26),
                            start_distance_m=metadata['start_distance_m'],
                            stopping_width_fraction=metadata['arrival_width_fraction'],
                            stop_reason='INCOMPLETE')
        write_summary(self.directory/'trial_summary.csv', self.summary)

    def record(self, row):
        if self.closed:
            return
        row = dict(row, trial_id=self.trial_id)
        self.writer.writerow({key: row.get(key, '') for key in COLUMNS})
        self.stream.flush()

    def finish(self, summary):
        if self.closed:
            return
        self.summary.update(summary)
        write_summary(self.directory/'trial_summary.csv', self.summary)
        self.stream.close()
        self.closed = True
