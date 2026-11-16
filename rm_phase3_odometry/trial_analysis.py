"""Enter independent measurements and aggregate measured physical trials only."""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path
from .trial_logger import write_summary


def read_summary(path):
    with Path(path).open() as stream:
        return next(csv.DictReader(stream))


def enter_measurement(directory, measured):
    """Store manually measured metres and signed estimate-minus-measurement error."""
    if not math.isfinite(measured) or measured < 0:
        raise ValueError('Physical measurement must be finite and nonnegative')
    path = Path(directory)/'trial_summary.csv'
    row = read_summary(path)
    if row.get('final_estimate_valid') == 'False' or row.get('stationary_confirmed') == 'False':
        raise ValueError('Final stationary estimate is unconfirmed')
    estimated = float(row['estimated_final_distance_m'])
    if not math.isfinite(estimated):
        raise ValueError('No finite final estimate in this trial')
    row['actual_measured_final_distance_m'] = measured
    row['signed_error_m'] = estimated-measured
    row['absolute_error_m'] = abs(estimated-measured)
    row['stopping_target_error_m'] = measured-float(row.get('physical_target_distance_m') or .26)
    write_summary(path, row)


def error_statistics(errors):
    """Return N, MAE, signed min/max and numerical signed-error span, metres."""
    if not errors:
        return dict(N=0, MAE_m=None, min_error_m=None, max_error_m=None, span_m=None,
                    mean_signed_error_m=None, std_error_m=None)
    return dict(N=len(errors), MAE_m=sum(map(abs, errors))/len(errors),
                min_error_m=min(errors), max_error_m=max(errors), span_m=max(errors)-min(errors),
                mean_signed_error_m=statistics.mean(errors),
                std_error_m=statistics.stdev(errors) if len(errors)>1 else 0.)


def aggregate(root, plot=False):
    root = Path(root).expanduser()
    rows, errors = [], []
    for path in sorted(root.glob('trial_*/trial_summary.csv')):
        row = read_summary(path)
        # Observation-only, interrupted and safety-stop runs are not completed arrival trials.
        if row.get('enable_motion', '').lower() != 'true' or row.get('stop_reason') != 'ARRIVED_WIDTH':
            continue
        if row.get('final_estimate_valid') == 'False' or row.get('stationary_confirmed') == 'False':
            continue
        try:
            estimated = float(row['estimated_final_distance_m'])
            measured = float(row['actual_measured_final_distance_m'])
        except (ValueError, KeyError):
            continue
        if not all(map(math.isfinite, (estimated, measured))) or measured < 0:
            continue
        error = estimated-measured
        errors.append(error)
        rows.append(dict(trial_id=row['trial_id'], estimation_mode=row.get('estimation_mode', '') or 'legacy_unspecified', estimated_final_distance_m=estimated,
                         measured_final_distance_m=measured, signed_error_m=error,
                         absolute_error_m=abs(error)))
    stats = error_statistics(errors)
    columns = ['trial_id', 'estimation_mode', 'estimated_final_distance_m', 'measured_final_distance_m',
               'signed_error_m', 'absolute_error_m'] + list(stats)
    with (root/'results_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(dict(row, **stats))
    stats['by_mode'] = {mode: error_statistics([r['signed_error_m'] for r in rows if r['estimation_mode']==mode])
                        for mode in sorted({r['estimation_mode'] for r in rows})}
    measured = [r['measured_final_distance_m'] for r in rows]
    stats['stopping_accuracy'] = dict(
        mean_physical_distance_m=statistics.mean(measured) if measured else None,
        mean_deviation_from_target_m=statistics.mean(measured)-.26 if measured else None,
        MAE_to_target_m=statistics.mean([abs(d-.26) for d in measured]) if measured else None,
        std_physical_distance_m=statistics.stdev(measured) if len(measured)>1 else (0. if measured else None),
        target_m=.26)
    (root/'error_statistics.json').write_text(json.dumps(stats, indent=2)+'\n')
    if plot and errors:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        ax.bar(range(1, len(errors)+1), errors)
        ax.axhline(0, color='black', linewidth=0.8)
        ax.set(xlabel='Completed measured trial', ylabel='Estimated − measured distance (m)')
        fig.tight_layout()
        fig.savefig(root/'final_errors.png')
        plt.close(fig)
    return stats


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    enter = sub.add_parser('measure')
    enter.add_argument('trial_directory')
    enter.add_argument('distance_m', type=float)
    summary = sub.add_parser('summarize')
    summary.add_argument('results_directory')
    summary.add_argument('--plot', action='store_true')
    movement = sub.add_parser('plot', help='Plot commanded reference versus actual driver odometry')
    movement.add_argument('trial_directory')
    movement.add_argument('--show', action='store_true', help='Open the path plot on the desktop')
    args = parser.parse_args()
    if args.action == 'measure':
        enter_measurement(args.trial_directory, args.distance_m)
    elif args.action == 'plot':
        from .movement_plots import plot_movement, open_movement_plot
        from .estimator_plots import plot_estimator
        estimator_result = plot_estimator(args.trial_directory)
        try:
            result = plot_movement(args.trial_directory)
        except ValueError as exc:
            result = dict(images=[], movement_plot_status=str(exc))
        result['estimator_validation'] = estimator_result
        if args.show and result['images']:
            result['viewer_status'] = open_movement_plot(result['images'][0])
        print(json.dumps(result, indent=2))
    else:
        print(json.dumps(aggregate(args.results_directory, args.plot), indent=2))


if __name__ == '__main__':
    main()
