"""Hardware-free checks for configurable Mirabo command fields."""

import importlib.util
import math
from pathlib import Path
import struct
import subprocess
import sys

from leader_controller import mapping
from leader_controller.mirabo_can import (
    COMMAND_PAYLOAD_FORMAT, MiraboCanDriver, pack_position_command,
    validate_command_field,
)
import numpy as np
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]


def load_tool(name):
    path = ROOT / 'tools' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def config(name):
    path = ROOT / 'config' / 'experiments' / f'{name}.yaml'
    return yaml.safe_load(path.read_text())['leader_mirabo_teleop'][
        'ros__parameters']


class SendOnlyBus:
    def __init__(self):
        self.sent = []

    def send(self, message, timeout):
        assert timeout == 0.0
        self.sent.append(message)


def test_default_profile_is_byte_for_byte_identical():
    defaults = yaml.safe_load((ROOT / 'config' / 'teleop.yaml').read_text())[
        'leader_mirabo_teleop']['ros__parameters']
    speed = defaults['mirabo_command_speed']
    accel = defaults['mirabo_command_acceleration']
    assert (speed, accel) == (mapping.COMMAND_SPEED, mapping.COMMAND_ACCEL)
    assert COMMAND_PAYLOAD_FORMAT == '>ihh'
    assert pack_position_command(10.0, speed, accel).hex() == (
        '000186a000640064'
    )


@pytest.mark.parametrize('speed,accel', [
    (900, 1000), (1000, 1000), (1100, 1000),
    (1000, 900), (1000, 1100),
])
def test_configured_fields_use_existing_signed_big_endian_encoding(
        speed, accel):
    payload = pack_position_command(10.0, speed, accel)
    assert len(payload) == 8
    assert struct.unpack('>ihh', payload) == (100000, speed // 10,
                                                accel // 10)
    assert validate_command_field(speed, 'speed') == speed // 10
    assert validate_command_field(accel, 'accel') == accel // 10


def test_changing_one_field_changes_only_its_bytes_and_not_can_id():
    baseline = pack_position_command(10.0, 1000, 1000)
    lower_speed = pack_position_command(10.0, 900, 1000)
    higher_accel = pack_position_command(10.0, 1000, 1100)
    assert lower_speed[:4] == higher_accel[:4] == baseline[:4]
    assert lower_speed[4:6] != baseline[4:6]
    assert lower_speed[6:8] == baseline[6:8]
    assert higher_accel[4:6] == baseline[4:6]
    assert higher_accel[6:8] != baseline[6:8]
    bus = SendOnlyBus()
    driver = MiraboCanDriver({0x2968, 0x2969}, bus=bus)
    driver.send_position(0x668, 10.0, 900, 1000)
    driver.send_position(0x668, 10.0, 1000, 1100)
    assert [frame.arbitration_id for frame in bus.sent] == [0x668, 0x668]
    assert all(frame.is_extended_id for frame in bus.sent)
    assert [bytes(frame.data) for frame in bus.sent] == [
        lower_speed, higher_accel,
    ]


@pytest.mark.parametrize('value', [0, 9, -100, 32768, 327680, math.inf,
                                   math.nan, True, '1000', 1000.5])
def test_invalid_command_field_rejected(value):
    with pytest.raises(ValueError):
        validate_command_field(value, 'mirabo_command_speed')


def test_phase3_profiles_keep_phase2_limits_and_change_one_field():
    phase2 = config('phase2_20hz_5dps')
    expected = {
        'phase3_speed_low': (900, 1000),
        'phase3_speed_baseline': (1000, 1000),
        'phase3_speed_high': (1100, 1000),
        'phase3_accel_low': (1000, 900),
        'phase3_accel_baseline': (1000, 1000),
        'phase3_accel_high': (1000, 1100),
    }
    for name, (speed, accel) in expected.items():
        profile = config(name)
        assert {key: profile[key] for key in phase2} == phase2
        assert profile['mirabo_command_speed'] == speed
        assert profile['mirabo_command_acceleration'] == accel
        assert validate_command_field(speed, 'speed') == speed // 10
        assert validate_command_field(accel, 'accel') == accel // 10


def test_command_profile_summary_requires_consistent_diagnostics():
    analysis = load_tool('analyze_tracking')
    sample = {
        'command_speed_configured': 900,
        'command_acceleration_configured': 1000,
        'command_speed_raw': 90,
        'command_acceleration_raw': 100,
    }
    assert analysis.command_profile_summary([])['status'] == 'UNAVAILABLE'
    result = analysis.command_profile_summary([sample, sample])
    assert result['status'] == 'RECORDED'
    assert result['command_speed_raw'] == 90
    mixed = analysis.command_profile_summary([
        sample, {**sample, 'command_speed_configured': 1100},
    ])
    assert mixed['status'] == 'MIXED'
    assert mixed['command_speed_configured'] is None


def test_response_estimates_require_clear_step_and_hold():
    analysis = load_tool('analyze_tracking')
    t = np.arange(0.0, 8.0, 0.05)
    command = np.clip((t - 1.5) / 1.0, 0.0, 1.0) * 0.2
    actual = np.where(t < 3.0, 0.0,
                      np.where(t < 3.9, (t - 3.0) * 0.22 / 0.9,
                               np.where(t < 4.1, 0.22, 0.2)))
    result = analysis.hold_response_estimate(t, command, t, actual)
    assert result['response_status'] == 'ESTIMATED'
    assert result['overshoot_estimate_rad'] == pytest.approx(0.02, abs=0.002)
    assert 1.0 < result['settling_estimate_sec'] < 2.0
    unsuitable = analysis.hold_response_estimate(t, t * 0.01, t, t * 0.01)
    assert unsuitable['response_status'] == 'INSUFFICIENT STEP/HOLD'
    assert unsuitable['overshoot_estimate_rad'] is None
    assert unsuitable['settling_estimate_sec'] is None


def test_comparison_reads_summary_without_ranking():
    comparison = load_tool('compare_mirabo_profiles')
    summary = {
        'command_profile': {'command_speed_configured': 900,
                            'command_acceleration_configured': 1000,
                            'status': 'RECORDED'},
        'joints': {'0x68': {'rms_command_error_rad': 0.1,
                            'max_abs_command_error_rad': 0.2,
                            'q_cmd_to_q_actual_lag_sec': 0.3,
                            'overshoot_estimate_rad': None,
                            'settling_estimate_sec': None,
                            'response_status': 'INSUFFICIENT STEP/HOLD'},
                   '0x69': {'rms_command_error_rad': 0.05}},
    }
    rows = list(comparison.measured_rows('speed_low', summary))
    assert [row['motor'] for row in rows] == ['0x68', '0x69']
    assert rows[0]['command_speed'] == 900
    assert rows[0]['rms_command_error_rad'] == 0.1
    assert rows[1]['max_abs_command_error_rad'] is None


def test_offline_inspector_prints_payload_without_socket():
    tool = ROOT / 'tools' / 'inspect_mirabo_command_frame.py'
    result = subprocess.run(
        [sys.executable, str(tool), '--position-deg', '10', '--speed', '900',
         '--acceleration', '1000'],
        capture_output=True, text=True, check=True,
    )
    assert 'encoded speed raw: 90' in result.stdout
    assert 'encoded acceleration raw: 100' in result.stdout
    assert '8-byte payload hex: 000186a0005a0064' in result.stdout
