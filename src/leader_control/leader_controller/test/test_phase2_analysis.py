"""Check Phase 2 timing, CAN counters and velocity report offline."""

import importlib.util
import json
from pathlib import Path

import pytest


def load_tool(name):
    path = Path(__file__).resolve().parents[1] / 'tools' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_timing_and_counter_summary():
    analysis = load_tool('analyze_tracking')
    diagnostics = [
        {'configured_control_rate_hz': 20.0,
         'timestamps_ns': {'t_control_loop': 1_000_000_000 + index * 50_000_000},
         'event_counts': {'can_warning_frames': count}}
        for index, count in enumerate((0, 1, 2, 0, 1))
    ]
    timing = analysis.timing_summary(diagnostics)
    assert timing['actual_control_rate_hz'] == pytest.approx(20.0)
    assert timing['control_dt_mean_ms'] == pytest.approx(50.0)
    assert timing['control_dt_median_ms'] == pytest.approx(50.0)
    assert timing['control_dt_p95_ms'] == pytest.approx(50.0)
    assert timing['control_dt_max_ms'] == pytest.approx(50.0)
    assert analysis.count_summary(diagnostics)['can_warning_frames'] == 3
    assert analysis.count_summary([
        {'event_counts': {'can_warning_frames': 4}},
        {'event_counts': {'can_warning_frames': 5}},
    ])['can_warning_frames'] == 1
    assert analysis.fault_transition_count([
        'state=READY', 'state=FAULT', 'state=FAULT',
        'state=READY', 'state=FAULT',
    ]) == 2


def test_velocity_plots_and_warning_report(tmp_path, monkeypatch):
    analysis = load_tool('analyze_tracking')
    times = [1000.0 + index * 0.05 for index in range(50)]
    storage = {}
    for joint in analysis.JOINTS.values():
        positions = [index * 0.01 for index in range(50)]
        storage[joint] = {
            'leader': list(zip(times, positions, [0.0] * 50)),
            'reference': list(zip(times, positions, [0.0] * 50)),
            'filtered': list(zip(times, positions, [0.0] * 50)),
            'command': list(zip(times, positions, [0.1] * 50)),
            'actual': list(zip(times, positions, [0.0] * 50)),
        }
    diagnostics = [
        {'configured_control_rate_hz': 20.0,
         'timestamps_ns': {'t_control_loop': 1_000_000_000 + index * 50_000_000},
         'rates_hz': {'leader_rx': 20.0, 'control': 20.0, 'can_tx': 40.0,
                      'feedback_68': 50.0, 'feedback_69': 50.0},
         'event_counts': {'can_warning_frames': int(index > 0)},
         'can_state': 'ACTIVE',
         'motor_fault_bytes': {'0x68': 0, '0x69': 0}}
        for index in range(50)
    ]
    monkeypatch.setattr(
        analysis, 'read_bag',
        lambda _: (storage, diagnostics, ['state=READY', 'state=ARMED'], []),
    )
    output = tmp_path / 'analysis'
    analysis.analyze(Path('fake_bag'), output)
    summary = json.loads((output / 'summary.json').read_text())
    assert summary['event_counts']['can_warning_frames'] == 1
    assert summary['fault_count'] == 0
    assert summary['timing']['control_dt_p95_ms'] == pytest.approx(50.0)
    assert (output / 'velocity_68.png').stat().st_size > 0
    assert (output / 'velocity_69.png').stat().st_size > 0
    assert (output / 'tracking_68.png').stat().st_size > 0
    assert (output / 'tracking_69.png').stat().st_size > 0


def test_comparison_uses_measured_values_and_rejects_profile_mismatch():
    comparison = load_tool('compare_experiments')
    joint = {
        'samples': {name: 10 for name in
                    ('reference', 'filtered', 'command', 'actual')},
        'rms_limiter_gap_rad': 0.1,
        'max_abs_limiter_gap_rad': 0.2,
        'rms_command_error_rad': 0.01,
        'max_abs_command_error_rad': 0.02,
        'rms_reference_error_rad': 0.11,
        'q_cmd_to_q_actual_lag_sec': 0.3,
    }
    summary = {
        'bag': 'trial',
        'timing': {'configured_control_rate_hz': 10.0,
                   'actual_control_rate_hz': 9.8},
        'configured_limits': {'max_velocity_deg_s': 2.5,
                              'max_acceleration_deg_s2': 12.5,
                              'max_step_deg': 0.5},
        'event_counts': {'can_warning_frames': 2},
        'fault_count': 0,
        'joints': {'0x68': joint, '0x69': joint},
    }
    row = comparison.measured_row('rate_10', summary)
    assert row['configured_control_hz'] == 10.0
    assert row['actual_control_hz'] == 9.8
    assert row['RMS_des_cmd_68'] == 0.1
    assert row['lag_69_ms'] == 300.0
    assert row['CAN_warning_count'] == 2
    assert row['status'] == 'MEASURED'
    summary['configured_limits']['max_velocity_deg_s'] = 5.0
    with pytest.raises(ValueError, match='does not match the bag'):
        comparison.measured_row('rate_10', summary)
