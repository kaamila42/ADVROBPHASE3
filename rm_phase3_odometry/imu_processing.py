"""Gyro-only measurement preprocessing; accelerometer is diagnostic, never integrated."""
import numpy as np


def gyro_measurement(gyro, frame, expected_frame='base_link'):
    """ROS base vertical yaw rate (rad/s) from verified driver body gyro.

    The installed driver applies SDK signs (1,-1,-1) and publishes base_link.
    Wheel kinematics also use base_link axes. Require the exact verified frame;
    reject unknown mounting transforms instead of assuming coincident axes.
    This planar model requires level operation; no fused attitude is observed.
    """
    values = np.asarray(gyro, dtype=float)
    if frame != expected_frame or values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError('GYRO_FRAME_OR_VALUES_INVALID')
    if np.max(np.abs(values)) > 3.:
        raise ValueError('GYRO_OUT_OF_RANGE')
    return values.copy()


class GyroBias:
    """Stationary batch mean (rad/s); bias freezes for the complete moving trial."""
    def __init__(self, samples=50):
        self.samples = samples
        self.batch = []
        self.gyro = np.full(3, np.nan)
        self.variance = np.full(3, np.nan)
        self.ready = False

    def observe(self, gyro, stationary):
        """Collect still-wheel samples across pauses; reject gross bias/vibration."""
        if self.ready:
            return
        values = np.asarray(gyro, dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            self.batch.clear()
            return
        if not stationary:
            return
        self.batch.append(values.copy())
        if len(self.batch) >= self.samples:
            data = np.asarray(self.batch)
            self.batch.clear()
            if np.max(data.std(axis=0)) > .03 or np.max(np.abs(data.mean(axis=0))) > .1:
                raise ValueError('GYRO_STATIONARY_CALIBRATION_REJECTED')
            self.gyro = data.mean(axis=0)
            self.variance = data.var(axis=0, ddof=1)/len(data)
            self.ready = True
