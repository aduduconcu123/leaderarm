#!/usr/bin/env python3
"""Compare measured Mirabo command profiles without ranking them."""

import argparse
import csv
import json
from pathlib import Path


FIELDS = (
    'run', 'motor', 'command_speed', 'command_acceleration',
    'rms_command_error_rad', 'max_abs_command_error_rad',
    'q_cmd_to_q_actual_lag_sec', 'overshoot_estimate_rad',
    'settling_estimate_sec', 'response_status', 'profile_status',
)


def resolve_summary(path):
    if path.is_dir():
        analysis = path / 'tracking_analysis' / 'summary.json'
        path = analysis if analysis.is_file() else path / 'summary.json'
    return path


def measured_rows(label, summary):
    profile = summary.get('command_profile', {})
    for motor in ('0x68', '0x69'):
        joint = summary.get('joints', {}).get(motor, {})
        yield {
            'run': label,
            'motor': motor,
            'command_speed': profile.get('command_speed_configured'),
            'command_acceleration': profile.get(
                'command_acceleration_configured'),
            'rms_command_error_rad': joint.get('rms_command_error_rad'),
            'max_abs_command_error_rad': joint.get(
                'max_abs_command_error_rad'),
            'q_cmd_to_q_actual_lag_sec': joint.get(
                'q_cmd_to_q_actual_lag_sec'),
            'overshoot_estimate_rad': joint.get('overshoot_estimate_rad'),
            'settling_estimate_sec': joint.get('settling_estimate_sec'),
            'response_status': joint.get('response_status'),
            'profile_status': profile.get('status', 'UNAVAILABLE'),
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', metavar='[NAME=]PATH',
                        help='Analysis folder, bag folder, or summary JSON')
    parser.add_argument('-o', '--output', type=Path,
                        default=Path('reports/phase3_profile_comparison.csv'))
    args = parser.parse_args()
    rows = []
    for argument in args.runs:
        name, separator, filename = argument.partition('=')
        path = Path(filename if separator else argument)
        if not path.exists():
            parser.error(f'run not found: {path}')
        summary_path = resolve_summary(path)
        with summary_path.open(encoding='utf-8') as source:
            summary = json.load(source)
        label = name if separator else Path(summary.get('bag', path.stem)).name
        rows.extend(measured_rows(label, summary))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    for motor in ('0x68', '0x69'):
        print(f'{motor}:')
        print('run | speed | accel | RMS cmd error (rad) | lag (s) | '
              'max error (rad) | overshoot (rad) | settling (s)')
        for row in rows:
            if row['motor'] == motor:
                values = (row[key] if row[key] is not None else 'N/A'
                          for key in ('run', 'command_speed',
                                      'command_acceleration',
                                      'rms_command_error_rad',
                                      'q_cmd_to_q_actual_lag_sec',
                                      'max_abs_command_error_rad',
                                      'overshoot_estimate_rad',
                                      'settling_estimate_sec'))
                print(' | '.join(map(str, values)))
    print(f'CSV: {args.output}')


if __name__ == '__main__':
    main()
