"""Estimator validation plots from recorded telemetry only; no inferred sensor values."""
import csv
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
from .movement_plots import finite_number


def plot_estimator(directory):
    """Save nine diagnostic panels for a new trial; unavailable series remain missing.

    X axes use ROS record time relative to the first estimator event, seconds.
    Gains are state-unit/(rad/s); covariance panels use m². No measured distance
    is synthesized and gyro correction rows are distinct from prediction rows.
    """
    directory = Path(directory)
    with (directory/'telemetry.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    events = {'estimator_prediction', 'estimator_correction', 'estimator_update', 'final'}
    data = [r for r in rows if r.get('event') in events and finite_number(r, 'timestamp_s') is not None]
    if not data:
        return dict(images=[], status='No estimator samples')
    start = float(data[0]['timestamp_s'])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 3, figsize=(16, 11))
    axes = axes.ravel()

    def curve(ax, key, label, subset=None):
        subset = data if subset is None else subset
        values = [(float(r['timestamp_s'])-start, finite_number(r, key)) for r in subset]
        values = [(t, v) for t, v in values if v is not None]
        if values:
            ax.plot(*np.asarray(values).T, label=label)

    specs = [
        [('wheel_only_forward_m', 'Raw wheel odometry')],
        [('predicted_forward_m', 'EKF predicted')],
        [('corrected_forward_m', 'EKF corrected')],
        [('wheel_only_heading_rad', 'Wheel'), ('corrected_heading_rad', 'EKF')],
        [('wheel_omega_radps', 'Wheel'), ('imu_yaw_rate_radps', 'Bias-corrected gyro')],
        [('predicted_remaining_m', 'Predicted'), ('corrected_remaining_m', 'Corrected')],
        [('gyro_innovation_radps', 'Gyro innovation')],
        [(f'gyro_K_{i}', name) for i, name in enumerate(['px (m s/rad)', 'py (m s/rad)',
            'theta (s)', 'v (m/rad)', 'omega (1)'])],
        [('predicted_P_00', 'Predicted Pxx'), ('corrected_P_00', 'Corrected Pxx'),
         ('predicted_P_11', 'Predicted Pyy'), ('corrected_P_11', 'Corrected Pyy')],
    ]
    labels = ['Wheel forward (m)', 'Predicted forward (m)', 'Corrected forward (m)',
              'Heading (rad)', 'Angular rate (rad/s)', 'Remaining distance (m)',
              'Innovation (rad/s)', 'Gain (units in legend)', 'Position covariance (m²)']
    for index, (ax, spec, label) in enumerate(zip(axes, specs, labels)):
        for key, legend in spec:
            curve(ax, key, legend)
        if not ax.lines:
            ax.text(.5, .5, 'Unavailable in this mode/log', ha='center', transform=ax.transAxes)
        else:
            ax.legend(fontsize=8)
        ax.set(xlabel='Time (s)', ylabel=label)
        ax.grid(True, alpha=.3)
    fig.suptitle('Recorded estimator prediction and correction — driver odometry is not ground truth')
    fig.tight_layout()
    output = directory/'estimator_plots'/datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    path = output/'estimator_validation.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return dict(images=[str(path)], directory=str(output), samples=len(data))
