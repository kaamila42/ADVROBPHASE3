"""Five-state EKF, preserving position–velocity correlations (Lecture 11B)."""
import numpy as np
from .motion_model import motion_model, jacobians, wrap_angle
from .sensor_measurements import VELOCITY_H, forward_h, displacement, fresh


class StateEstimator:
    def __init__(self, x, P=None, input_variance=(0.25, 1.0),
                 process_density=(0.0001, 0.0001, 0.0001, 0.0025, 0.01),
                 max_dt=0.25):
        self.x = np.asarray(x, dtype=float).copy()
        self.P = np.diag([0.0001, 0.0001, 0.001, 0.01, 0.04]) if P is None else np.array(P, dtype=float)
        self.Qu = np.diag(input_variance)
        self.Qp_density = np.diag(process_density)
        self.max_dt = max_dt
        self.stamp = None
        self.origin = self.x.copy()
        self.theta0 = self.x[2]
        self.validate()

    def validate(self):
        """Reject nonfinite, asymmetric or non-positive-semidefinite covariance."""
        if (self.x.shape != (5,) or self.P.shape != (5, 5) or
                not np.isfinite(self.x).all() or not np.isfinite(self.P).all() or
                not np.allclose(self.P, self.P.T, atol=1e-10) or
                np.linalg.eigvalsh(self.P).min() < -1e-10):
            raise ValueError('Invalid estimator state/covariance')

    def predict(self, u, dt):
        """Advance SI state; P-=F P Fᵀ+G Qu Gᵀ+Qp_density*dt.

        Qu: acceleration variance (m²/s⁴, rad²/s⁴).
        Qp_density: state variance per second; P entries use state-unit products.
        Returns independent snapshots of predicted state and full covariance.
        """
        if not 0 < dt <= self.max_dt:
            raise ValueError('Unreasonable prediction interval')
        F, G = jacobians(self.x, dt)
        self.x = motion_model(self.x, u, dt)
        self.P = F @ self.P @ F.T + G @ self.Qu @ G.T + self.Qp_density*dt
        self.P = (self.P+self.P.T)/2
        self.validate()
        return self.x.copy(), self.P.copy()

    def predict_wheels_to(self, stamp, now, wheel, Qwheel, timeout=.3):
        """Wheel input [m/s,rad/s]; P-=F P F.T+G Qwheel G.T+Qmodel*dt.

        First sample seeds rate states and their input covariance without moving
        pose. Every later wheel sample is used once. No gyro enters prediction.
        """
        from .motion_model import wheel_motion_model, wheel_jacobians
        wheel, Qwheel = np.asarray(wheel, dtype=float), np.asarray(Qwheel, dtype=float)
        if (wheel.shape != (2,) or Qwheel.shape != (2, 2) or
                not np.isfinite(np.r_[wheel, Qwheel.ravel()]).all() or
                not np.allclose(Qwheel, Qwheel.T) or np.linalg.eigvalsh(Qwheel).min() <= 0):
            raise ValueError('WHEEL_INPUT_OR_COVARIANCE_INVALID')
        if not fresh(stamp, now, timeout):
            raise ValueError('WHEEL_TIMESTAMP_REJECTED')
        if self.stamp is None:
            self.x[3:] = wheel
            self.P[3:, :] = 0.
            self.P[:, 3:] = 0.
            self.P[3:, 3:] = Qwheel
            self.stamp = stamp
            self.validate()
            return 0., self.x.copy(), self.P.copy()
        dt = stamp-self.stamp
        if not 0 < dt <= self.max_dt:
            raise ValueError('WHEEL_REPEATED_REVERSED_OR_GAPPED')
        F, G = wheel_jacobians(self.x, wheel, dt)
        self.x = wheel_motion_model(self.x, wheel, dt)
        self.P = F@self.P@F.T + G@Qwheel@G.T + self.Qp_density*dt
        self.P = (self.P+self.P.T)/2
        self.stamp = stamp
        self.validate()
        return dt, self.x.copy(), self.P.copy()

    def correct_gyro(self, yaw_rate, variance, gate=9.):
        """Independent bias-corrected vertical gyro (rad/s); H=[0,0,0,0,1]."""
        return self.correct([yaw_rate], [[0., 0., 0., 0., 1.]], [[variance]], gate=gate)

    def predict_to(self, stamp, now, u=(0., 0.), timeout=0.5):
        """Use message times; reject stale/repeated/reversed/gapped times without mutation."""
        if not fresh(stamp, now, timeout):
            raise ValueError('stale_or_future_timestamp')
        if self.stamp is None:
            self.stamp = stamp
            return 0., self.x.copy(), self.P.copy()
        dt = stamp-self.stamp
        if not 0 < dt <= self.max_dt:
            raise ValueError('repeated_reversed_or_gapped_timestamp')
        x, P = self.predict(u, dt)
        self.stamp = stamp
        return dt, x, P

    def correct(self, z, H, R, expected=None, gate=16.):
        """Linear observation update; R in measurement units squared; Joseph covariance.

        Returns accepted flag, innovation, gain, and normalized innovation squared.
        A rejected observation leaves both state and covariance unchanged.
        """
        z, H, R = np.asarray(z), np.asarray(H), np.asarray(R)
        if (not np.isfinite(z).all() or not np.isfinite(H).all() or
                not np.isfinite(R).all() or not np.allclose(R, R.T) or
                np.linalg.eigvalsh(R).min() <= 0):
            raise ValueError('Invalid measurement/noise')
        e = z - (H @ self.x if expected is None else np.asarray(expected))
        S = H @ self.P @ H.T + R
        K = np.linalg.solve(S, H @ self.P).T
        nis = float(e @ np.linalg.solve(S, e))
        accepted = nis <= gate
        if accepted:
            self.x += K @ e
            self.x[2] = wrap_angle(self.x[2])
            A = np.eye(5)-K@H
            self.P = A@self.P@A.T + K@R@K.T
            self.P = (self.P+self.P.T)/2
            self.validate()
        return accepted, e, K, nis

    def correct_velocity(self, v, omega, R, gate=16.):
        """Original driver body velocity observation (m/s,rad/s), Mode B only."""
        return self.correct(np.array([v, omega]), VELOCITY_H, R, gate=gate)

    def correct_tof(self, measured_displacement, variance, gate=9.):
        """Scalar forward displacement (m); caller verifies time and beam geometry."""
        return self.correct([measured_displacement], forward_h(self.theta0), [[variance]],
                            [self.forward()], gate)

    def forward(self, x=None):
        return displacement(self.x if x is None else x, self.origin, self.theta0)
