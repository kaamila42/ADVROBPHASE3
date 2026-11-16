"""Command-reference versus measured odometry plots; no ROS or robot connection."""
import csv
import json
import math
import os
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
import numpy as np


def open_movement_plot(path):
    """Open the generated path image when a desktop session is available."""
    if not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        return 'saved; no desktop display available'
    opener = shutil.which('xdg-open')
    if opener is None:
        return 'saved; xdg-open unavailable'
    try:
        subprocess.Popen([opener, str(Path(path).resolve())], start_new_session=True)
    except OSError as exc:
        return f'saved; viewer could not open: {exc}'
    return 'path plot opened'


def finite_number(row, key):
    try:
        value = float(row[key])
    except (ValueError, KeyError, TypeError):
        return None
    return value if math.isfinite(value) else None


def integrate_twist(pose, v, omega, dt):
    """Integrate a constant recorded Twist over dt seconds; pose units m,m,rad."""
    x, y, yaw = pose
    if abs(omega) < 1e-9:
        return np.array([x+v*math.cos(yaw)*dt, y+v*math.sin(yaw)*dt, yaw+omega*dt])
    next_yaw = yaw+omega*dt
    return np.array([x+v/omega*(math.sin(next_yaw)-math.sin(yaw)),
                     y-v/omega*(math.cos(next_yaw)-math.cos(yaw)), next_yaw])


def movement_series(rows):
    """Build a common-time comparison in the initial forward/lateral frame.

    Actual position is driver odometry, not ground truth. Planned position is
    the piecewise constant commanded-Twist integral, not the EKF prediction.
    New logs record command changes; older logs approximate their timing from
    the telemetry samples. Invalid/repeated odometry timestamps are excluded.
    """
    actual = {}
    for row in rows:
        if str(row.get('odom_valid', '')).lower() != 'true':
            continue
        stamp = finite_number(row, 'odom_stamp_s')
        if stamp is None:
            if row.get('event') != 'estimator_update':
                continue
            stamp = finite_number(row, 'timestamp_s')
        values = [finite_number(row, key) for key in
                  ('odom_x_m', 'odom_y_m', 'odom_heading_rad', 'odom_v_mps', 'odom_omega_radps')]
        if stamp is not None and stamp > 0 and all(value is not None for value in values):
            actual[stamp] = values
    if len(actual) < 2:
        raise ValueError('At least two distinct valid odometry samples are needed for movement plots')
    times = np.array(sorted(actual))
    odom = np.array([actual[stamp] for stamp in times])
    changes = [row for row in rows if row.get('event') == 'command']
    command_source = 'command_change_events' if changes else 'sampled_telemetry_approximation'
    commands = {}
    for row in changes or rows:
        values = [finite_number(row, key) for key in
                  ('timestamp_s', 'command_v_mps', 'command_omega_radps')]
        if all(value is not None for value in values):
            commands[values[0]] = values[1:]
    if not commands:
        raise ValueError('No finite recorded commands are available')
    events = sorted(commands.items())
    index = 0
    command = np.zeros(2)
    start = times[0]
    while index < len(events) and events[index][0] <= start:
        command = np.array(events[index][1])
        index += 1
    pose, previous = np.zeros(3), start
    planned, rates = [], []
    for stamp in times:
        while index < len(events) and events[index][0] <= stamp:
            change_stamp, next_command = events[index]
            pose = integrate_twist(pose, *command, change_stamp-previous)
            previous = change_stamp
            command = np.array(next_command)
            index += 1
        pose = integrate_twist(pose, *command, stamp-previous)
        previous = stamp
        planned.append(pose.copy())
        rates.append(command.copy())
    theta0 = odom[0, 2]
    c, s = math.cos(theta0), math.sin(theta0)
    rotation = np.array([[c, s], [-s, c]])
    displacement = (odom[:, :2]-odom[0, :2]) @ rotation.T
    actual_heading = np.unwrap(odom[:, 2])-odom[0, 2]
    actual_pose = np.column_stack((displacement, actual_heading))
    return dict(timestamp_s=times, elapsed_s=times-start, planned=np.array(planned),
                actual=actual_pose, command=np.array(rates), measured_velocity=odom[:, 3:5],
                command_source=command_source)


def plot_movement(directory):
    """Write five PNGs plus a derived comparison CSV/JSON; raw telemetry stays untouched."""
    directory = Path(directory).expanduser()
    with (directory/'telemetry.csv').open() as stream:
        result = movement_series(list(csv.DictReader(stream)))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    generated_at = datetime.now().astimezone()
    stamp = generated_at.strftime('%Y%m%d_%H%M%S_%f')
    root = directory/'plots'
    root.mkdir(exist_ok=True)
    suffix = 0
    while True:
        output = root/(stamp if suffix == 0 else f'{stamp}_{suffix:02d}')
        try:
            output.mkdir(exist_ok=False)
            break
        except FileExistsError:
            suffix += 1
    t, planned, actual = result['elapsed_s'], result['planned'], result['actual']
    command, measured = result['command'], result['measured_velocity']
    error = actual-planned
    error[:, 2] = (error[:, 2]+np.pi) % (2*np.pi)-np.pi
    paths = []

    def save(fig, filename):
        path = output/filename
        fig.tight_layout()
        fig.savefig(path, dpi=160)
        plt.close(fig)
        paths.append(str(path))

    reference_label = 'Planned (recorded command integral)'
    actual_label = 'Actual (driver odometry)'
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.plot(planned[:, 0], planned[:, 1], '--', label=reference_label)
    ax.plot(actual[:, 0], actual[:, 1], label=actual_label)
    ax.scatter([0], [0], color='black', marker='o', label='Start')
    ax.scatter([actual[-1, 0]], [actual[-1, 1]], marker='x', color='tab:orange')
    ax.set(xlabel='Forward displacement (m)', ylabel='Lateral displacement (m)',
           title='Planned versus actual robot movement')
    ax.axis('equal'); ax.grid(True); ax.legend()
    save(fig, 'planned_vs_actual_path.png')

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(t, planned[:, i], '--', label=reference_label)
        ax.plot(t, actual[:, i], label=actual_label)
        ax.set_ylabel(('Forward (m)', 'Lateral (m)')[i]); ax.grid(True); ax.legend()
    axes[-1].set_xlabel('Time from first valid odometry (s)')
    fig.suptitle('Position versus time')
    save(fig, 'planned_vs_actual_position.png')

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, np.rad2deg(planned[:, 2]), '--', label=reference_label)
    ax.plot(t, np.rad2deg(actual[:, 2]), label=actual_label)
    ax.set(xlabel='Time (s)', ylabel='Heading relative to start (degrees)', title='Heading tracking')
    ax.grid(True); ax.legend()
    save(fig, 'planned_vs_actual_heading.png')

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, ax in enumerate(axes):
        ax.step(t, command[:, i], where='post', linestyle='--', label='Commanded')
        ax.plot(t, measured[:, i], label='Measured odometry')
        ax.set_ylabel(('Forward velocity (m/s)', 'Yaw rate (rad/s)')[i]); ax.grid(True); ax.legend()
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Commanded versus measured velocities')
    save(fig, 'planned_vs_actual_velocity.png')

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for i, ax in enumerate(axes):
        values = error[:, i] if i < 2 else np.rad2deg(error[:, i])
        ax.plot(t, values)
        ax.axhline(0, color='black', linewidth=.6)
        ax.set_ylabel(('Forward error (m)', 'Lateral error (m)', 'Heading error (deg)')[i])
        ax.grid(True)
    axes[-1].set_xlabel('Time (s)')
    fig.suptitle('Odometry minus commanded reference; not ground-truth estimation error')
    save(fig, 'movement_tracking_errors.png')

    columns = ['timestamp_s', 'elapsed_s', 'planned_forward_m', 'planned_lateral_m',
               'planned_heading_rad', 'odom_forward_m', 'odom_lateral_m', 'odom_relative_heading_rad',
               'command_v_mps', 'command_omega_radps', 'odom_v_mps', 'odom_omega_radps',
               'forward_tracking_error_m', 'lateral_tracking_error_m', 'heading_tracking_error_rad']
    with (output/'movement_comparison.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for i in range(len(t)):
            writer.writerow([result['timestamp_s'][i], t[i], *planned[i], *actual[i],
                             *command[i], *measured[i], *error[i]])
    summary = dict(generated_at=generated_at.isoformat(),
        valid_odom_samples=len(t), duration_s=float(t[-1]),
        reference='Piecewise constant recorded command integral, initialized at first odometry pose',
        command_timing=result['command_source'],
        actual='Driver odometry transformed to initial forward/lateral frame; not ground truth',
        final_planned_forward_m=float(planned[-1, 0]), final_planned_lateral_m=float(planned[-1, 1]),
        final_odom_forward_m=float(actual[-1, 0]), final_odom_lateral_m=float(actual[-1, 1]),
        final_tracking_error_m=float(np.linalg.norm(error[-1, :2])),
        caution='Commanded speeds are a reference only; they are not substituted for actual EKF velocity')
    (output/'movement_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return dict(directory=str(output), images=paths, **summary)
