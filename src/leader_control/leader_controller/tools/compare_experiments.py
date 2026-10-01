#!/usr/bin/env python3
"""Combine measured tracking summaries into a Phase 2 CSV report."""

import argparse
import csv
import json
from pathlib import Path

import yaml


FIELDS = (
    'profile', 'bag', 'configured_control_hz', 'actual_control_hz',
    'configured_vmax', 'configured_amax', 'configured_max_step',
    'RMS_des_cmd_68', 'Peak_des_cmd_68', 'RMS_cmd_actual_68',
    'Peak_cmd_actual_68', 'RMS_des_actual_68', 'lag_68_ms',
    'RMS_des_cmd_69', 'Peak_des_cmd_69', 'RMS_cmd_actual_69',
    'Peak_cmd_actual_69', 'RMS_des_actual_69', 'lag_69_ms',
    'leader_callback_hz', 'CAN_TX_hz', 'CAN_feedback_68_hz',
    'CAN_feedback_69_hz', 'control_dt_mean_ms', 'control_dt_median_ms',
    'control_dt_p95_ms', 'control_dt_max_ms', 'control_jitter_p95_ms',
    'CAN_warning_count', 'fault_count', 'can_fault_observed', 'status',
)
PROFILE_DIR = Path(__file__).resolve().parents[1] / 'config' / 'experiments'


def measured_row(profile, summary):
    profile_path = PROFILE_DIR / f'{profile}.yaml'
    with profile_path.open(encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)[
            'leader_mirabo_teleop']['ros__parameters']
    timing = summary.get('timing', {})
    rates = summary.get('rates_hz', {})
    measured_config = {
        'control_rate_hz': timing.get('configured_control_rate_hz'),
        **summary.get('configured_limits', {}),
    }
    for key, value in measured_config.items():
        if value is not None and value != config[key]:
            raise ValueError(
                f'{profile} does not match the bag: {key}='
                f'{value}, profile expects {config[key]}'
            )
    row = {
        'profile': profile,
        'bag': summary.get('bag'),
        'configured_control_hz': config['control_rate_hz'],
        'actual_control_hz': timing.get('actual_control_rate_hz'),
        'configured_vmax': config['max_velocity_deg_s'],
        'configured_amax': config['max_acceleration_deg_s2'],
        'configured_max_step': config['max_step_deg'],
        'leader_callback_hz': rates.get('leader_rx'),
        'CAN_TX_hz': rates.get('can_tx'),
        'CAN_feedback_68_hz': rates.get('feedback_68'),
        'CAN_feedback_69_hz': rates.get('feedback_69'),
        'control_dt_mean_ms': timing.get('control_dt_mean_ms'),
        'control_dt_median_ms': timing.get('control_dt_median_ms'),
        'control_dt_p95_ms': timing.get('control_dt_p95_ms'),
        'control_dt_max_ms': timing.get('control_dt_max_ms'),
        'control_jitter_p95_ms': timing.get('control_jitter_p95_ms'),
        'CAN_warning_count': summary.get('event_counts', {}).get(
            'can_warning_frames'),
        'fault_count': summary.get('fault_count'),
        'can_fault_observed': summary.get('can_fault_observed'),
    }
    complete = True
    for motor, suffix in (('0x68', '68'), ('0x69', '69')):
        joint = summary.get('joints', {}).get(motor, {})
        samples = joint.get('samples', {})
        complete &= all(samples.get(name, 0) for name in
                        ('reference', 'filtered', 'command', 'actual'))
        row.update({
            f'RMS_des_cmd_{suffix}': joint.get('rms_limiter_gap_rad'),
            f'Peak_des_cmd_{suffix}': joint.get('max_abs_limiter_gap_rad'),
            f'RMS_cmd_actual_{suffix}': joint.get('rms_command_error_rad'),
            f'Peak_cmd_actual_{suffix}': joint.get(
                'max_abs_command_error_rad'),
            f'RMS_des_actual_{suffix}': joint.get('rms_reference_error_rad'),
            f'lag_{suffix}_ms': (
                joint['q_cmd_to_q_actual_lag_sec'] * 1000
                if joint.get('q_cmd_to_q_actual_lag_sec') is not None else None
            ),
        })
    row['status'] = ('INCOMPLETE' if not complete else
                     'WARNING_COUNT_UNAVAILABLE' if row['CAN_warning_count']
                     is None else 'MEASURED')
    return row


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('measurements', nargs='+', metavar='NAME=SUMMARY_JSON')
    parser.add_argument(
        '-o', '--output', type=Path,
        default=Path('reports/phase2/config_comparison.csv'),
    )
    args = parser.parse_args()
    rows = []
    for item in args.measurements:
        name, separator, filename = item.partition('=')
        if not separator or not name or not filename:
            parser.error(f'expected NAME=SUMMARY_JSON, got {item!r}')
        with Path(filename).open(encoding='utf-8') as report_file:
            rows.append(measured_row(name, json.load(report_file)))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='', encoding='utf-8') as output_file:
        writer = csv.DictWriter(output_file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


if __name__ == '__main__':
    main()
