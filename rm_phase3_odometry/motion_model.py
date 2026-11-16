"""Lecture 11A slides 13–15. State [px, py, theta, v, omega], SI units."""
import numpy as np


def wrap_angle(angle):
    """Return angle in radians in [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion_model(x, u, dt):
    """Euler prediction: x (m,m,rad,m/s,rad/s), u (m/s²,rad/s²), dt (s)."""
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)
    if x.shape != (5,) or u.shape != (2,) or not np.isfinite(np.r_[x, u, dt]).all() or dt <= 0:
        raise ValueError('Invalid motion model inputs')
    px, py, theta, v, omega = x
    return np.array([px + v*np.cos(theta)*dt, py + v*np.sin(theta)*dt,
                     wrap_angle(theta + omega*dt), v + u[0]*dt, omega + u[1]*dt])


def jacobians(x, dt):
    """Return state F (5x5) and input G (5x2), evaluated at the prior state."""
    theta, v = x[2:4]
    c, s = np.cos(theta), np.sin(theta)
    F = np.eye(5)
    F[0, 2], F[1, 2] = -v*s*dt, v*c*dt
    F[0, 3], F[1, 3], F[2, 4] = c*dt, s*dt, dt
    G = np.zeros((5, 2))
    G[3, 0] = G[4, 1] = dt
    return F, G

def wheel_motion_model(x, wheel, dt):
    """Wheel-driven trapezoidal velocities + midpoint pose (SI units).

    ds=(v+vw)*dt/2, dtheta=(omega+ww)*dt/2. Prior corrected rates therefore
    affect the next pose; endpoints become new measured wheel rates. Input is
    [vw m/s, ww rad/s], never commanded velocity or fused driver odometry.
    """
    x, wheel = np.asarray(x, dtype=float), np.asarray(wheel, dtype=float)
    if x.shape != (5,) or wheel.shape != (2,) or not np.isfinite(np.r_[x, wheel, dt]).all() or dt <= 0:
        raise ValueError('Invalid wheel prediction inputs')
    ds = (x[3]+wheel[0])*dt/2
    dtheta = (x[4]+wheel[1])*dt/2
    phi = x[2]+dtheta/2
    return np.array([x[0]+ds*np.cos(phi), x[1]+ds*np.sin(phi),
                     wrap_angle(x[2]+dtheta), wheel[0], wheel[1]])


def wheel_jacobians(x, wheel, dt):
    """F=df/dx (5x5), G=df/d[vw,ww] (5x2), matching wheel_motion_model."""
    d = dt/2
    ds = (x[3]+wheel[0])*d
    phi = x[2]+(x[4]+wheel[1])*d/2
    c, s = np.cos(phi), np.sin(phi)
    F = np.zeros((5, 5))
    F[0, 0] = F[1, 1] = F[2, 2] = 1.
    F[0, 2], F[1, 2] = -ds*s, ds*c
    F[0, 3], F[1, 3] = d*c, d*s
    F[0, 4], F[1, 4], F[2, 4] = -ds*s*d/2, ds*c*d/2, d
    G = np.zeros((5, 2))
    G[0, 0], G[1, 0] = d*c, d*s
    G[0, 1], G[1, 1], G[2, 1] = -ds*s*d/2, ds*c*d/2, d
    G[3, 0] = G[4, 1] = 1.
    return F, G
