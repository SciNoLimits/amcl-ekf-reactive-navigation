#!/usr/bin/env python3
"""Time-align EKF and Gazebo streams and calculate relative trajectory errors."""

import argparse
import csv
import math
import statistics
from pathlib import Path


def read_rows(path):
    with open(path, newline='') as stream:
        return list(csv.DictReader(stream))


def val(row, key):
    return float(row[key])


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def interp_angle(a, b, ratio):
    return a + ratio * wrap(b - a)


def interpolate(rows, timestamp):
    times = [val(row, 'timestamp_sec') for row in rows]
    lo, hi = 0, len(times) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if times[mid] <= timestamp:
            lo = mid
        else:
            hi = mid
    if timestamp <= times[0]:
        a = b = rows[0]
        ratio = 0.0
    elif timestamp >= times[-1]:
        a = b = rows[-1]
        ratio = 0.0
    else:
        a, b = rows[lo], rows[hi]
        ta, tb = val(a, 'timestamp_sec'), val(b, 'timestamp_sec')
        ratio = (timestamp - ta) / (tb - ta) if tb != ta else 0.0

    result = {}
    for key in ('x_m', 'y_m'):
        result[key] = val(a, key) + ratio * (val(b, key) - val(a, key))
    result['yaw_rad'] = interp_angle(
        val(a, 'yaw_rad'), val(b, 'yaw_rad'), ratio)
    return result


def align_to_initial(rows):
    """Express each trajectory relative to its own initial pose."""
    first = rows[0]
    x0, y0, yaw0 = val(first, 'x_m'), val(first, 'y_m'), val(first, 'yaw_rad')
    c, s = math.cos(yaw0), math.sin(yaw0)
    output = []
    for row in rows:
        dx, dy = val(row, 'x_m') - x0, val(row, 'y_m') - y0
        output.append({
            'x': c * dx + s * dy,
            'y': -s * dx + c * dy,
            'yaw': wrap(val(row, 'yaw_rad') - yaw0)})
    return output


def rmse(values):
    return math.sqrt(statistics.mean(v * v for v in values))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input-dir', default='ekf_experiment')
    args = parser.parse_args()
    folder = Path(args.input_dir)

    ekf = sorted(read_rows(folder / 'ekf.csv'),
                 key=lambda row: val(row, 'timestamp_sec'))
    truth = sorted(read_rows(folder / 'ground_truth.csv'),
                   key=lambda row: val(row, 'timestamp_sec'))

    if len(ekf) < 2 or len(truth) < 2:
        raise SystemExit('Need at least two samples from each stream.')

    # Use EKF timestamps within the overlapping time interval.
    start = max(val(ekf[0], 'timestamp_sec'), val(truth[0], 'timestamp_sec'))
    end = min(val(ekf[-1], 'timestamp_sec'), val(truth[-1], 'timestamp_sec'))
    ekf_overlap = [
        row for row in ekf if start <= val(row, 'timestamp_sec') <= end]
    if len(ekf_overlap) < 2:
        raise SystemExit('Insufficient overlap between EKF and ground-truth streams.')

    truth_interp = [
        interpolate(truth, val(row, 'timestamp_sec')) for row in ekf_overlap]

    ekf_rel = align_to_initial(ekf_overlap)
    truth_rows_for_alignment = [
        {'x_m': str(row['x_m']), 'y_m': str(row['y_m']),
         'yaw_rad': str(row['yaw_rad'])} for row in truth_interp]
    truth_rel = align_to_initial(truth_rows_for_alignment)

    errors = []
    position_errors, yaw_errors = [], []
    for ekf_row, e, g in zip(ekf_overlap, ekf_rel, truth_rel):
        dx, dy = e['x'] - g['x'], e['y'] - g['y']
        dyaw = wrap(e['yaw'] - g['yaw'])
        position_error = math.hypot(dx, dy)
        position_errors.append(position_error)
        yaw_errors.append(dyaw)
        errors.append([
            val(ekf_row, 'timestamp_sec'), ekf_row['phase'],
            e['x'], e['y'], g['x'], g['y'], dx, dy,
            position_error, dyaw,
            val(ekf_row, 'variance_x'), val(ekf_row, 'variance_y'),
            val(ekf_row, 'variance_yaw')])

    with open(folder / 'aligned_errors.csv', 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow([
            'timestamp_sec', 'phase', 'ekf_relative_x_m', 'ekf_relative_y_m',
            'ground_truth_relative_x_m', 'ground_truth_relative_y_m',
            'error_x_m', 'error_y_m', 'position_error_m', 'yaw_error_rad',
            'variance_x', 'variance_y', 'variance_yaw'])
        writer.writerows(errors)

    metrics = {
        'overlap_samples': len(ekf_overlap),
        'overlap_duration_s': end - start,
        'position_rmse_m': rmse(position_errors),
        'position_mean_error_m': statistics.mean(position_errors),
        'position_max_error_m': max(position_errors),
        'yaw_rmse_rad': rmse(yaw_errors),
        'yaw_mean_absolute_error_rad':
            statistics.mean(abs(x) for x in yaw_errors),
        'yaw_max_absolute_error_rad':
            max(abs(x) for x in yaw_errors),
        'mean_variance_x': statistics.mean(
            val(row, 'variance_x') for row in ekf_overlap),
        'mean_variance_y': statistics.mean(
            val(row, 'variance_y') for row in ekf_overlap),
        'mean_variance_yaw': statistics.mean(
            val(row, 'variance_yaw') for row in ekf_overlap),
    }

    for phase in ('baseline', 'final_stationary'):
        phase_rows = [row for row in ekf if row['phase'] == phase]
        if len(phase_rows) >= 2:
            first, last = phase_rows[0], phase_rows[-1]
            metrics[phase + '_drift_m'] = math.hypot(
                val(last, 'x_m') - val(first, 'x_m'),
                val(last, 'y_m') - val(first, 'y_m'))

    with open(folder / 'metrics.csv', 'w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['metric', 'value'])
        writer.writerows(metrics.items())

    for name, value in metrics.items():
        print(f'{name}: {value}')
    print(f'Wrote results to {folder.resolve()}')

    try:
        import matplotlib.pyplot as plt
        times = [val(row, 'timestamp_sec') - val(ekf_overlap[0], 'timestamp_sec')
                 for row in ekf_overlap]

        fig, ax = plt.subplots()
        ax.plot([p['x'] for p in truth_rel], [p['y'] for p in truth_rel],
                label='Gazebo ground truth')
        ax.plot([p['x'] for p in ekf_rel], [p['y'] for p in ekf_rel],
                label='EKF')
        ax.set_xlabel('Relative x (m)')
        ax.set_ylabel('Relative y (m)')
        ax.set_title('Trajectory (initial poses aligned)')
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(folder / 'trajectory_comparison.png')
        plt.close(fig)

        fig, ax = plt.subplots()
        ax.plot(times, position_errors)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Planar position error (m)')
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(folder / 'position_error.png')
        plt.close(fig)

        fig, ax = plt.subplots()
        ax.plot(times, [math.degrees(x) for x in yaw_errors])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw error (degrees)')
        ax.grid(True)
        fig.tight_layout()
        fig.savefig(folder / 'yaw_error.png')
        plt.close(fig)

        fig, ax = plt.subplots()
        ax.plot(times, [val(row, 'variance_x') for row in ekf_overlap],
                label='x')
        ax.plot(times, [val(row, 'variance_y') for row in ekf_overlap],
                label='y')
        ax.plot(times, [val(row, 'variance_yaw') for row in ekf_overlap],
                label='yaw')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('EKF covariance')
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(folder / 'covariance.png')
        plt.close(fig)
    except ImportError:
        print('matplotlib unavailable: CSV metrics were generated, plots skipped.')


if __name__ == '__main__':
    main()
