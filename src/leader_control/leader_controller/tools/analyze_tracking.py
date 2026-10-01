#!/usr/bin/env python3
"""Plot and summarize leader-to-Mirabo tracking from a ROS 2 bag."""

import argparse
import csv
from difflib import get_close_matches
import json
import math
from pathlib import Path
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
import yaml
from leader_controller import mapping
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    '/leader/joint_states': 'leader',
    '/leader_controller/reference': 'reference',
    '/leader_controller/filtered_reference': 'filtered',
    '/leader_controller/command': 'command',
    '/mirabo/joint_states': 'actual',
    '/leader_controller/diagnostics': 'diagnostics',
    '/leader_controller/mirabo_status': 'status',
}
JOINTS = {'0x68': 'mirabo_68', '0x69': 'mirabo_69'}
LEADER_JOINTS = {'mirabo_68': 'joint_2', 'mirabo_69': 'joint_3'}
SIGNALS = {
    'leader': 'q_leader', 'reference': 'q_des',
    'filtered': 'q_filtered', 'command': 'q_cmd', 'actual': 'q_actual',
}
SAMPLE_FIELDS = (
    'ros_time_ns', 'bag_receive_time_ns', 'motor', 'leader_joint',
    'signal', 'position_rad', 'velocity_rad_s',
)
ALIGNED_FIELDS = (
    'ros_time_ns', 'motor', 'leader_joint', 'q_leader_rad',
    'q_des_rad', 'q_filtered_rad', 'q_cmd_rad', 'v_cmd_rad_s',
    'q_actual_rad', 'leader_age_ms', 'command_age_ms',
)


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_joint_sample(msg, joint_name):
    try:
        index = msg.name.index(joint_name)
    except ValueError:
        return None
    if index >= len(msg.position):
        return None
    return stamp_sec(msg), msg.position[index], (
        msg.velocity[index] if index < len(msg.velocity) else math.nan
    )


def read_bag(path):
    metadata = yaml.safe_load((path / 'metadata.yaml').read_text(
        encoding='utf-8'
    ))['rosbag2_bagfile_information']
    storage_id = metadata['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    storage = {topic: {key: [] for key in ('leader', 'reference', 'filtered',
                                           'command', 'actual')}
               for topic in JOINTS.values()}
    diagnostics = []
    statuses = []
    records = []
    cache = {}
    while reader.has_next():
        topic, raw, bag_time_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        message_type = cache.setdefault(topic, get_message(types[topic]))
        msg = deserialize_message(raw, message_type)
        key = TOPICS[topic]
        if key == 'diagnostics':
            try:
                diagnostics.append(json.loads(msg.data))
            except (ValueError, TypeError):
                continue
            continue
        if key == 'status':
            statuses.append(msg.data)
            continue
        ros_time_ns = (msg.header.stamp.sec * 1_000_000_000 +
                       msg.header.stamp.nanosec)
        for motor, joint_name in JOINTS.items():
            series = storage[joint_name]
            source_name = LEADER_JOINTS[joint_name] if key == 'leader' \
                else joint_name
            sample = read_joint_sample(msg, source_name)
            if sample is not None:
                series[key].append(sample)
                records.append({
                    'ros_time_ns': ros_time_ns,
                    'bag_receive_time_ns': bag_time_ns,
                    'motor': motor,
                    'leader_joint': LEADER_JOINTS[joint_name],
                    'signal': SIGNALS[key],
                    'position_rad': sample[1],
                    'velocity_rad_s': (sample[2]
                                       if math.isfinite(sample[2]) else ''),
                })
    return storage, diagnostics, statuses, records


def arrays(samples):
    if not samples:
        return np.array([]), np.array([]), np.array([])
    data = np.asarray(samples, dtype=float)
    order = np.argsort(data[:, 0])
    return data[order, 0], data[order, 1], data[order, 2]


def write_aligned_samples(series, records, output_path):
    prepared = {
        motor: {key: arrays(samples) for key, samples in series[joint].items()}
        for motor, joint in JOINTS.items()
    }

    def latest(data, at_sec):
        times, positions, velocities = data
        index = np.searchsorted(times, at_sec, side='right') - 1
        if index < 0:
            return None
        return times[index], positions[index], velocities[index]

    with output_path.open('w', newline='', encoding='utf-8') as output_file:
        writer = csv.DictWriter(output_file, fieldnames=ALIGNED_FIELDS)
        writer.writeheader()
        for record in sorted(records, key=lambda row: row['ros_time_ns']):
            if record['signal'] != 'q_actual':
                continue
            at_sec = record['ros_time_ns'] * 1e-9
            signals = prepared[record['motor']]
            leader = latest(signals['leader'], at_sec)
            reference = latest(signals['reference'], at_sec)
            filtered = latest(signals['filtered'], at_sec)
            command = latest(signals['command'], at_sec)
            writer.writerow({
                'ros_time_ns': record['ros_time_ns'],
                'motor': record['motor'],
                'leader_joint': record['leader_joint'],
                'q_leader_rad': leader[1] if leader else '',
                'q_des_rad': reference[1] if reference else '',
                'q_filtered_rad': filtered[1] if filtered else '',
                'q_cmd_rad': command[1] if command else '',
                'v_cmd_rad_s': command[2] if command else '',
                'q_actual_rad': record['position_rad'],
                'leader_age_ms': (at_sec - leader[0]) * 1000
                if leader else '',
                'command_age_ms': (at_sec - command[0]) * 1000
                if command else '',
            })


def rms(values):
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else None


def aligned_error(t_source, q_source, t_other, q_other):
    if not len(t_source) or not len(t_other):
        return np.array([])
    overlap = (t_source >= t_other[0]) & (t_source <= t_other[-1])
    if not np.any(overlap):
        return np.array([])
    return q_source[overlap] - np.interp(
        t_source[overlap], t_other, q_other
    )


def lag_estimate(t_cmd, q_cmd, t_actual, q_actual):
    if len(t_cmd) < 3 or len(t_actual) < 3:
        return None
    start = max(t_cmd[0], t_actual[0])
    end = min(t_cmd[-1], t_actual[-1])
    if end - start < 1.0:
        return None
    dt = min(np.median(np.diff(t_cmd)), np.median(np.diff(t_actual)))
    dt = max(float(dt), 0.002)
    grid = np.arange(start, end, dt)
    if len(grid) < 30:
        return None
    cmd = np.interp(grid, t_cmd, q_cmd)
    actual = np.interp(grid, t_actual, q_actual)
    if np.ptp(cmd) < math.radians(0.2) or np.ptp(actual) < math.radians(0.1):
        return None
    cmd_v = np.gradient(cmd, dt)
    actual_v = np.gradient(actual, dt)
    cmd_v -= np.mean(cmd_v)
    actual_v -= np.mean(actual_v)
    if np.linalg.norm(cmd_v) < 1e-8 or np.linalg.norm(actual_v) < 1e-8:
        return None
    size = 1 << (2 * len(cmd_v) - 1).bit_length()
    correlation = np.fft.irfft(
        np.fft.rfft(actual_v, size) *
        np.conj(np.fft.rfft(cmd_v, size)), size
    )
    max_lag_samples = min(len(cmd_v) - 1, int(10.0 / dt))
    return float(np.argmax(correlation[:max_lag_samples + 1]) * dt)


def rate_summary(diagnostics):
    keys = ('leader_rx', 'control', 'can_tx', 'feedback_68', 'feedback_69')
    result = {}
    for key in keys:
        values = [row.get('rates_hz', {}).get(key, 0.0)
                  for row in diagnostics]
        values = [value for value in values if math.isfinite(value) and value > 0]
        result[key] = float(np.median(values)) if values else None
    return result


def count_summary(diagnostics):
    keys = ('can_warning_frames', 'can_bus_off_frames',
            'can_malformed_frames', 'motor_fault_events',
            'can_receive_errors', 'command_errors')
    result = {}
    for key in keys:
        values = [row['event_counts'][key] for row in diagnostics
                  if key in row.get('event_counts', {})]
        if not values:
            result[key] = None
            continue
        result[key] = sum(
            current - previous if current >= previous else current
            for previous, current in zip(values, values[1:])
        )
    return result


def command_profile_summary(diagnostics):
    fields = ('command_speed_configured', 'command_acceleration_configured',
              'command_speed_raw', 'command_acceleration_raw')
    result = {}
    for field in fields:
        values = {row[field] for row in diagnostics if field in row}
        result[field] = next(iter(values)) if len(values) == 1 else None
    if not diagnostics or all(result[field] is None for field in fields):
        result['status'] = 'UNAVAILABLE'
    elif any(len({row[field] for row in diagnostics if field in row}) > 1
             for field in fields):
        result['status'] = 'MIXED'
    elif any(result[field] is None for field in fields):
        result['status'] = 'INCOMPLETE'
    else:
        result['status'] = 'RECORDED'
    return result


def hold_response_estimate(t_cmd, q_cmd, t_actual, q_actual):
    """Estimate response only for a clear move between stable command holds."""
    unavailable = {
        'overshoot_estimate_rad': None,
        'settling_estimate_sec': None,
        'response_status': 'INSUFFICIENT STEP/HOLD',
    }
    if len(t_cmd) < 20 or len(t_actual) < 20:
        return unavailable
    stable = np.abs(np.diff(q_cmd)) <= math.radians(0.02)
    holds = []
    start = None
    for index, is_stable in enumerate(np.r_[stable, False]):
        if is_stable and start is None:
            start = index
        elif not is_stable and start is not None:
            end = index
            if t_cmd[end] - t_cmd[start] >= 1.0:
                holds.append((start, end))
            start = None
    for (first_start, first_end), (last_start, last_end) in reversed(
            list(zip(holds, holds[1:]))):
        if t_cmd[last_end] - t_cmd[last_start] < 2.0:
            continue
        initial = float(np.median(q_cmd[first_start:first_end + 1]))
        target = float(np.median(q_cmd[last_start:last_end + 1]))
        amplitude = target - initial
        if abs(amplitude) < math.radians(1.0):
            continue
        window = ((t_actual >= t_cmd[first_end]) &
                  (t_actual <= t_cmd[last_end]))
        hold_window = ((t_actual >= t_cmd[last_start]) &
                       (t_actual <= t_cmd[last_end]))
        if np.count_nonzero(window) < 20 or np.count_nonzero(hold_window) < 20:
            continue
        actual = q_actual[window]
        overshoot = max(0.0, float(np.max(np.sign(amplitude) *
                                           (actual - target))))
        hold_times = t_actual[hold_window]
        hold_actual = q_actual[hold_window]
        tolerance = max(math.radians(0.1), 0.05 * abs(amplitude))
        outside = np.flatnonzero(np.abs(hold_actual - target) > tolerance)
        settle_index = int(outside[-1] + 1) if len(outside) else 0
        settled = (settle_index < len(hold_times) and
                   hold_times[-1] - hold_times[settle_index] >= 0.5)
        return {
            'overshoot_estimate_rad': overshoot,
            'settling_estimate_sec': (
                float(hold_times[settle_index] - t_cmd[last_start])
                if settled else None
            ),
            'response_status': 'ESTIMATED' if settled else 'NOT SETTLED',
        }
    return unavailable


def fault_transition_count(statuses):
    if not statuses:
        return None
    states = [status.split(';', 1)[0].removeprefix('state=')
              for status in statuses]
    return sum(state in ('FAULT', 'ESTOP') and
               (index == 0 or states[index - 1] != state)
               for index, state in enumerate(states))


def last_status_details(statuses):
    if not statuses:
        return None, None
    latest = statuses[-1]
    state = next((part.strip().split('=', 1)[1]
                  for part in latest.split(';')
                  if part.strip().startswith('can_state=')), None)
    faults = {
        f'0x{motor.lower()}': int(byte, 16)
        for motor, byte in re.findall(
            r'0x(68|69)=[^;]*/fault=0x([0-9A-Fa-f]+)', latest
        )
    }
    return state, faults or None


def timing_summary(diagnostics):
    configured = next(
        (row['configured_control_rate_hz'] for row in diagnostics
         if row.get('configured_control_rate_hz', 0) > 0), None
    )
    stamps = [row.get('timestamps_ns', {}).get('t_control_loop')
              for row in diagnostics]
    stamps = np.asarray(sorted(set(stamp for stamp in stamps if stamp)),
                        dtype=np.int64)
    if len(stamps) < 2:
        return {'configured_control_rate_hz': configured,
                'actual_control_rate_hz': None,
                'control_dt_mean_ms': None,
                'control_dt_median_ms': None,
                'control_dt_p95_ms': None,
                'control_dt_max_ms': None,
                'control_interval_std_ms': None,
                'control_jitter_p95_ms': None}
    intervals = np.diff(stamps).astype(float) * 1e-9
    expected = 1.0 / configured if configured else None
    return {
        'configured_control_rate_hz': configured,
        'actual_control_rate_hz': float(1.0 / np.mean(intervals)),
        'control_dt_mean_ms': float(np.mean(intervals) * 1000),
        'control_dt_median_ms': float(np.median(intervals) * 1000),
        'control_dt_p95_ms': float(np.percentile(intervals, 95) * 1000),
        'control_dt_max_ms': float(np.max(intervals) * 1000),
        'control_interval_std_ms': float(np.std(intervals) * 1000),
        'control_jitter_p95_ms': (
            float(np.percentile(np.abs(intervals - expected), 95) * 1000)
            if expected else None
        ),
    }


def analyze(path, outdir):
    series, diagnostics, statuses, records = read_bag(path)
    status_can_state, status_motor_faults = last_status_details(statuses)
    outdir.mkdir(parents=True, exist_ok=True)
    with (outdir / 'tracking_samples.csv').open(
            'w', newline='', encoding='utf-8') as sample_file:
        writer = csv.DictWriter(sample_file, fieldnames=SAMPLE_FIELDS)
        writer.writeheader()
        writer.writerows(sorted(
            records,
            key=lambda row: (row['ros_time_ns'], row['bag_receive_time_ns'],
                             row['motor'], row['signal']),
        ))
    write_aligned_samples(series, records, outdir / 'tracking_aligned.csv')
    summary = {'bag': str(path), 'rates_hz': rate_summary(diagnostics),
               'timing': timing_summary(diagnostics),
               'configured_limits': {
                   key: diagnostics[0].get(f'configured_{key}')
                   if diagnostics else None
                   for key in ('max_velocity_deg_s',
                               'max_acceleration_deg_s2', 'max_step_deg')
               },
               'event_counts': count_summary(diagnostics),
               'command_profile': command_profile_summary(diagnostics),
               'fault_count': fault_transition_count(statuses),
               'last_can_state': (diagnostics[-1].get('can_state')
                                  if diagnostics else None) or status_can_state,
               'last_motor_fault_bytes': (
                   diagnostics[-1].get('motor_fault_bytes')
                   if diagnostics else None) or status_motor_faults,
               'can_fault_observed': (
                   any('fault_code=CAN_' in status or
                       'CAN BUS-OFF' in status for status in statuses)
                   if statuses else None
               ),
               'lag_method': 'velocity cross-correlation of ROS-stamped samples',
               'joints': {}}
    for motor, joint_name in JOINTS.items():
        values = series[joint_name]
        data = {key: arrays(values[key]) for key in values}
        required = ('reference', 'filtered', 'command', 'actual')
        timeline = [data[key][0] for key in required if len(data[key][0])]
        fig, axis = plt.subplots(figsize=(11, 5.5))
        if timeline:
            left = min(item[0] for item in timeline)
            right = max(item[-1] for item in timeline)
            for key, label in (('reference', 'q_des'), ('filtered', 'q_filtered'),
                               ('command', 'q_cmd'), ('actual', 'q_actual')):
                times, positions, _ = data[key]
                if len(times):
                    axis.plot(times - left, np.degrees(positions), label=label,
                              linewidth=1.25)
            axis.set_xlim(0, max(right - left, 0.01))
            axis.legend()
        else:
            axis.text(0.5, 0.5, 'No tracking samples in bag',
                      ha='center', va='center', transform=axis.transAxes)
        axis.set(xlabel='Time (s)', ylabel='Angle (deg)',
                 title=f'{joint_name} ({motor}) tracking')
        axis.grid(True, alpha=0.25)
        fig.tight_layout()
        fig.savefig(outdir / f'tracking_{motor[2:]}.png', dpi=150)
        plt.close(fig)

        t_des, q_des, _ = data['reference']
        t_leader, q_leader, _ = data['leader']
        t_cmd, q_cmd, v_cmd = data['command']
        t_actual, q_actual, _ = data['actual']
        _, q_filtered, _ = data['filtered']
        velocity_fig, velocity_axis = plt.subplots(figsize=(11, 3.5))
        valid_velocity = np.isfinite(v_cmd)
        if np.any(valid_velocity):
            velocity_axis.plot(
                t_cmd[valid_velocity] - t_cmd[valid_velocity][0],
                np.degrees(v_cmd[valid_velocity]), linewidth=1.25,
            )
        else:
            velocity_axis.text(
                0.5, 0.5, 'No command velocity samples',
                ha='center', va='center', transform=velocity_axis.transAxes,
            )
        velocity_axis.set(xlabel='Time (s)', ylabel='v_cmd (deg/s)',
                          title=f'{joint_name} ({motor}) command velocity')
        velocity_axis.grid(True, alpha=0.25)
        velocity_fig.tight_layout()
        velocity_fig.savefig(outdir / f'velocity_{motor[2:]}.png', dpi=150)
        plt.close(velocity_fig)
        e_ref = aligned_error(t_des, q_des, t_actual, q_actual)
        e_cmd = aligned_error(t_cmd, q_cmd, t_actual, q_actual)
        limiter_gap = aligned_error(t_cmd, q_cmd, t_des, q_des) * -1
        reference_lag = lag_estimate(t_des, q_des, t_cmd, q_cmd)
        sign = next(item.sign for item in mapping.MIRABO_JOINTS
                    if item.motor_id == int(motor, 16))
        leader_lag = lag_estimate(
            t_leader, sign * q_leader, t_actual, q_actual,
        )
        lag = lag_estimate(t_cmd, q_cmd, t_actual, q_actual)
        excited = (len(t_cmd) >= 10 and len(t_actual) >= 10
                   and np.ptp(q_cmd) >= math.radians(0.2)
                   and np.ptp(q_actual) >= math.radians(0.1))
        summary['joints'][motor] = {
            'samples': {key: len(values[key]) for key in values},
            'rms_reference_error_rad': rms(e_ref),
            'rms_command_error_rad': rms(e_cmd),
            'rms_limiter_gap_rad': rms(limiter_gap),
            'max_abs_reference_error_rad': float(np.max(np.abs(e_ref))) if len(e_ref) else None,
            'max_abs_command_error_rad': float(np.max(np.abs(e_cmd))) if len(e_cmd) else None,
            'max_abs_limiter_gap_rad': float(np.max(np.abs(limiter_gap))) if len(limiter_gap) else None,
            'q_filtered_samples': len(q_filtered),
            'v_cmd_samples': len(v_cmd),
            'peak_command_velocity_rad_s': (
                float(np.max(np.abs(v_cmd[np.isfinite(v_cmd)])))
                if np.any(np.isfinite(v_cmd)) else None
            ),
            'q_des_to_q_cmd_lag_sec': reference_lag,
            'q_leader_to_q_actual_lag_sec': leader_lag,
            'q_cmd_to_q_actual_lag_sec': lag if excited else None,
            'lag_status': 'ESTIMATED' if excited and lag is not None
                          else 'INSUFFICIENT EXCITATION',
            **hold_response_estimate(t_cmd, q_cmd, t_actual, q_actual),
        }

    (outdir / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    lines = [f"Bag: {path}", 'Rates (Hz):']
    lines.extend(f'  {key}: {value if value is not None else "N/A"}'
                 for key, value in summary['rates_hz'].items())
    lines.append('Control timing:')
    lines.extend(f'  {key}: {value if value is not None else "N/A"}'
                 for key, value in summary['timing'].items())
    lines.append(f"CAN fault observed: {summary['can_fault_observed']}")
    lines.append(f"Fault transitions: {summary['fault_count']}")
    lines.append('Event counts:')
    lines.extend(f'  {key}: {value if value is not None else "N/A"}'
                 for key, value in summary['event_counts'].items())
    lines.append('Mirabo command profile:')
    lines.extend(f'  {key}: {value if value is not None else "N/A"}'
                 for key, value in summary['command_profile'].items())
    for motor, row in summary['joints'].items():
        lines.extend((f'\n{motor}:',
                      f"  RMS reference error (rad): {row['rms_reference_error_rad']}",
                      f"  RMS command error (rad): {row['rms_command_error_rad']}",
                      f"  RMS limiter gap (rad): {row['rms_limiter_gap_rad']}",
                      f"  Max abs reference error (rad): {row['max_abs_reference_error_rad']}",
                      f"  Max abs command error (rad): {row['max_abs_command_error_rad']}",
                      f"  Max abs limiter gap (rad): {row['max_abs_limiter_gap_rad']}",
                      f"  q_cmd -> q_actual lag (s): {row['q_cmd_to_q_actual_lag_sec']}",
                      f"  q_des -> q_cmd lag (s): {row['q_des_to_q_cmd_lag_sec']}",
                      f"  q_leader -> q_actual lag (s): {row['q_leader_to_q_actual_lag_sec']}",
                      f"  Peak command velocity (rad/s): {row['peak_command_velocity_rad_s']}",
                      f"  Overshoot estimate (rad): {row['overshoot_estimate_rad']}",
                      f"  Settling estimate from command hold (s): {row['settling_estimate_sec']}",
                      f"  Response status: {row['response_status']}",
                      f"  {row['lag_status']}"))
    (outdir / 'summary.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=Path, help='ROS 2 bag directory')
    parser.add_argument('-o', '--output-dir', type=Path)
    args = parser.parse_args()
    if not args.bag.is_dir():
        parent = args.bag.parent
        candidates = [item.name for item in parent.iterdir() if item.is_dir()] \
            if parent.is_dir() else []
        similar = get_close_matches(args.bag.name, candidates, n=1)
        hint = f' Did you mean {parent / similar[0]}?' if similar else ''
        parser.error(
            f'Bag directory {args.bag} does not exist.{hint} '
            'Record a bag first with scripts/record_tracking.sh.'
        )
    if not (args.bag / 'metadata.yaml').is_file():
        parser.error(
            f'{args.bag} has no metadata.yaml; use the rosbag directory '
            'after recording has stopped.'
        )
    analyze(args.bag, args.output_dir or args.bag / 'tracking_analysis')


if __name__ == '__main__':
    main()
