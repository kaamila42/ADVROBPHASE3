"""Validation and measurement functions; no ROS or SDK connection."""
import math
import numpy as np

VELOCITY_H = np.array([[0., 0., 0., 1., 0.], [0., 0., 0., 0., 1.]])


def fresh(stamp, now, timeout):
    """Accept positive, finite ROS timestamps with age in [0, timeout] seconds."""
    return math.isfinite(stamp) and stamp > 0 and 0 <= now-stamp <= timeout


def valid_range(value, lower, upper):
    """Driver uses max_range as an invalid return; exclude it."""
    return all(map(math.isfinite, (value, lower, upper))) and 0 < lower <= value < upper


def forward_h(theta0):
    """Displacement Jacobian in the initial heading direction, SI units."""
    return np.array([[math.cos(theta0), math.sin(theta0), 0., 0., 0.]])


def displacement(x, origin, theta0):
    """Signed forward displacement (m), positive even when odom X decreases."""
    return float((forward_h(theta0) @ (np.asarray(x)-np.asarray(origin)))[0])


def tof_displacement(d0, raw_range, offset):
    """d0 - (range + delta), all metres, same physical reference and surface."""
    return d0 - (raw_range + offset)


def safe_command(enabled, healthy, v, omega):
    """The single command gate: observation-only and unhealthy runs output zero."""
    if not enabled or not healthy or not np.isfinite([v, omega]).all():
        return 0., 0.
    return float(v), float(omega)


WHEEL_NAMES = ('front_right_wheel_joint', 'front_left_wheel_joint',
               'rear_left_wheel_joint', 'rear_right_wheel_joint')


def wheel_velocity(names, velocities, radius=.05, axis=.20):
    """JointState rad/s -> (vx m/s, vy m/s, yaw rad/s), installed EP geometry.

    The driver already flips SDK left speeds into forward-positive joint
    velocities. Do not flip them again. FR,FL,RL,RR; axis=lx+ly=.1+.1 m.
    """
    if len(names) != len(velocities) or len(set(names)) != len(names):
        raise ValueError('WHEEL_NAMES_INVALID')
    try:
        raw = np.array([velocities[names.index(n)] for n in WHEEL_NAMES], dtype=float)
    except ValueError as exc:
        raise ValueError('WHEEL_JOINTS_MISSING') from exc
    if not np.isfinite(raw).all() or np.max(np.abs(raw)) > 100 or radius <= 0 or axis <= 0:
        raise ValueError('WHEEL_SPEED_INVALID')
    fr, fl, rl, rr = raw * radius
    return raw, np.array([(fr+fl+rl+rr)/4, (fr-fl+rl-rr)/4,
                          (fr-fl-rl+rr)/(4*axis)])


def guard_displacement(x, origin, offset):
    """Forward displacement of guard p+R(theta)*offset, metres, initial heading axis."""
    def point(state):
        c, s = np.cos(state[2]), np.sin(state[2])
        return np.asarray(state[:2]) + np.array([[c, -s], [s, c]]) @ np.asarray(offset)
    delta = point(x)-point(origin)
    return float(delta @ np.array([np.cos(origin[2]), np.sin(origin[2])]))


class SettlingCheck:
    """Confirm stationary from distinct fresh velocity samples spanning duration (s)."""
    def __init__(self, linear=.005, angular=.03, duration=.5):
        self.linear, self.angular, self.duration = linear, angular, duration
        self.begin = None
        self.last = -1.
        self.confirmed_stamp = None

    def update(self, stamp, now, v, omega, timeout=.3):
        """Return confirmation using actual m/s and rad/s; gaps reset the interval."""
        if not fresh(stamp, now, timeout):
            self.begin = self.confirmed_stamp = None
            return False
        if not np.isfinite([v, omega]).all() or abs(v) >= self.linear or abs(omega) >= self.angular:
            self.begin = self.confirmed_stamp = None
            return False
        if stamp < self.last:
            self.begin = self.confirmed_stamp = None
            return False
        if stamp == self.last:
            return self.confirmed_stamp is not None
        gap = stamp-self.last > timeout
        self.last = stamp
        if self.begin is None or gap:
            self.begin = stamp
        if stamp-self.begin >= self.duration-1e-9:
            self.confirmed_stamp = stamp
        return self.confirmed_stamp is not None


def guard_h(x, theta0, offset):
    """Jacobian of guard forward displacement; heading uncertainty moves the guard (m/rad)."""
    H = forward_h(theta0)
    c, s = np.cos(x[2]), np.sin(x[2])
    derivative = np.array([[-s, -c], [c, -s]]) @ np.asarray(offset)
    H[0, 2] = derivative @ np.array([np.cos(theta0), np.sin(theta0)])
    return H
