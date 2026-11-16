"""Read-only historical audit plus explicitly synthetic A/B/C comparison; no ROS connection."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .motion_model import motion_model
from .state_estimator import StateEstimator
from .trial_analysis import error_statistics
from .sensor_measurements import SettlingCheck

MEASURED_M = [.28, .29, .27, .27, .26, .29, .27, .27]


def historical_audit(root):
    """Reproduce supplied baseline from last logged states, marking rest as unverified.

    Physical distances were measured later. Last rows are baseline diagnostics only;
    do not promote them to confirmed stationary snapshots or infer missing raw wheels.
    """
    rows=[]
    for number, measured in enumerate(MEASURED_M, 1):
        paths=list(Path(root).glob(f'trial_{number:03d}_*/telemetry.csv'))
        if len(paths)!=1:
            raise ValueError(f'Expected exactly one historical trial {number:03d}')
        with paths[0].open() as stream:
            data=list(csv.DictReader(stream))
        last=data[-1]
        wheel=float(last['odom_only_remaining_m'])
        ekf=float(last['corrected_remaining_m'])
        stop=next((float(r['timestamp_s']) for r in data if r['event']=='stop'), None)
        settling=SettlingCheck()
        for row in data:
            if stop is not None and float(row['timestamp_s'])>=stop:
                settling.update(float(row['odom_stamp_s']), float(row['timestamp_s']),
                                float(row['odom_v_mps']), float(row['odom_omega_radps']))
        rows.append(dict(trial_number=number, physical_final_m=measured,
            baseline_driver_odom_final_row_m=wheel, baseline_ekf_final_row_m=ekf,
            baseline_driver_error_m=wheel-measured, baseline_ekf_error_m=ekf-measured,
            logged_stop_reason=last['stop_reason'],
            stationary_interval_observed=settling.confirmed_stamp is not None,
            revised_ekf_final_m=None,
            replay_available=False,
            missing='raw ESC wheel velocities; IMU exists in logs, native acquisition times unavailable'))
    return dict(trials=rows,
        driver_odom_baseline=error_statistics([r['baseline_driver_error_m'] for r in rows]),
        legacy_ekf_baseline=error_statistics([r['baseline_ekf_error_m'] for r in rows]),
        note='Last-row baseline only, not verified final physical position. Driver odom is not raw wheel odometry.')


def synthetic_comparison():
    """A raw wheel, B original EKF, C wheel-prediction/gyro-correction, shared samples.

    Deterministic synthetic inputs only: these values are never trial data.
    """
    from .motion_model import wheel_motion_model
    rng=np.random.default_rng(672)
    dt=.1
    truth=np.zeros(5); wheel=np.zeros(5)
    legacy=StateEstimator(np.zeros(5)); revised=StateEstimator(np.zeros(5))
    Qwheel=np.diag([.0025,.01])
    revised.predict_wheels_to(1.,1.,[0.,0.],Qwheel)
    legacy.predict_to(1.,1.)
    rows=[]
    for i in range(1,161):
        t=1+i*dt
        v=.014*i if i<=10 else (.014*(150-i) if 141<=i<=150 else (.14 if i<141 else 0.))
        true_w=.03 if 30<=i<=60 else 0.
        truth=wheel_motion_model(truth,[v,true_w],dt)
        wheel_v=v+rng.normal(0,.008)
        wheel_w=true_w+rng.normal(0,.012)
        imu_w=true_w+rng.normal(0,.008)
        wheel=wheel_motion_model(wheel,[wheel_v,wheel_w],dt)
        legacy.predict_to(t,t)
        legacy.correct_velocity(wheel_v,imu_w,np.diag([.002525,.0109]))
        _,predicted,_=revised.predict_wheels_to(t,t,[wheel_v,wheel_w],Qwheel)
        revised.correct_gyro(imu_w,.01)
        rows.append(dict(timestamp_s=t, truth_forward_m=truth[0],
            A_forward_m=wheel[0], B_forward_m=legacy.x[0], C_forward_m=revised.x[0],
            truth_remaining_m=2.-truth[0], A_remaining_m=2.-wheel[0],
            B_remaining_m=2.-legacy.x[0], C_remaining_m=2.-revised.x[0],
            C_predicted_forward_m=predicted[0], C_heading_rad=revised.x[2]))
    return rows


def plot_historical_series(root, output):
    """Plot available baseline trajectories; physical measurement is a final reference only."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for number, measured in enumerate(MEASURED_M, 1):
        path=next(Path(root).glob(f'trial_{number:03d}_*/telemetry.csv'))
        with path.open() as stream:
            data=list(csv.DictReader(stream))
        data=[r for r in data if r['event']=='estimator_update']
        t0=float(data[0]['timestamp_s'])
        times=[float(r['timestamp_s'])-t0 for r in data]
        fig,axes=plt.subplots(2,1,sharex=True,figsize=(8,6))
        for key,label in [('odom_only_remaining_m','Driver odometry'),('corrected_remaining_m','Legacy EKF')]:
            remaining=[float(r[key]) for r in data]
            axes[0].plot(times,[2.-v for v in remaining],label=label)
            axes[1].plot(times,remaining,label=label)
        axes[0].axhline(2.-measured,color='black',linestyle=':',label='Physical FINAL displacement only')
        axes[1].axhline(measured,color='black',linestyle=':',label='Physical FINAL distance only')
        axes[0].set_title(f'Historical {number:03d}: revised EKF unavailable; final rest may be unverified')
        axes[0].set_ylabel('Forward displacement (m)'); axes[1].set_ylabel('Remaining distance (m)')
        axes[1].set_xlabel('Time since first estimator row (s)')
        for ax in axes: ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(Path(output)/f'historical_{number:03d}_timeseries.png'); plt.close(fig)


def evaluate(root, output):
    """Write derived CSV/plots into a new directory, leaving input trials untouched."""
    output=Path(output)
    # Refuse to place audit artifacts inside historical input directories.
    if Path(root).resolve() in (output.resolve(), *output.resolve().parents):
        raise ValueError('Output must be outside the historical results directory')
    output.mkdir(parents=True,exist_ok=False)
    report=historical_audit(root)
    rows=synthetic_comparison()
    for name, data in [('historical_baseline.csv',report['trials']), ('synthetic_comparison.csv',rows)]:
        with (output/name).open('x',newline='') as stream:
            writer=csv.DictWriter(stream,list(data[0])); writer.writeheader(); writer.writerows(data)
    physical=np.array(MEASURED_M)
    report['physical_stopping_accuracy'] = dict(mean_physical_distance_m=float(physical.mean()),
        mean_deviation_from_target_m=float(physical.mean()-.26),
        std_physical_distance_m=float(physical.std(ddof=1)), target_m=.26)
    report['synthetic_only_final_error_m']={method:rows[-1][f'{method}_remaining_m']-rows[-1]['truth_remaining_m'] for method in 'ABC'}
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,sharex=True,figsize=(8,6))
    for method in ['truth','A','B','C']:
        for ax,quantity in zip(axes,['forward','remaining']):
            ax.plot([r['timestamp_s']-1 for r in rows],[r[f'{method}_{quantity}_m'] for r in rows],label=method)
            ax.set_ylabel(f'{quantity} (m)'); ax.legend()
    axes[0].set_title('Synthetic A/B/C validation — no physical trial')
    axes[1].set_xlabel('Time (s)'); fig.tight_layout(); fig.savefig(output/'synthetic_comparison.png'); plt.close(fig)
    # Historical A is unavailable: driver odom is shown with its actual provenance.
    fig,ax=plt.subplots()
    trials=report['trials']; x=np.arange(1,9)
    for key,label in [('physical_final_m','Physical after stop'),
                      ('baseline_driver_odom_final_row_m','Driver odom last row'),
                      ('baseline_ekf_final_row_m','Legacy EKF last row')]:
        ax.plot(x,[r[key] for r in trials],'o-',label=label)
    ax.set(xlabel='Historical trial',ylabel='Remaining distance (m)',title='Baseline audit: raw wheel A and revised C unavailable; rest not uniformly verified')
    ax.plot([], [], linestyle='none', label='Raw wheel A / revised C: unavailable')
    ax.legend(); fig.tight_layout(); fig.savefig(output/'historical_baseline.png'); plt.close(fig)
    plot_historical_series(root, output)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results'); parser.add_argument('output')
    args=parser.parse_args()
    print(json.dumps(evaluate(args.results,args.output),indent=2))


if __name__=='__main__':
    main()
