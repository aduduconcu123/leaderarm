"""Hardware-free checks of Phase 2 command limits and staged profiles."""

import math
from pathlib import Path

from leader_controller.command_limiter import (
    active_constraints, limit_position, trace_limit_stages,
)
import pytest
import yaml


def limited(target, previous=0.0, velocity=0.0, dt=0.05,
            step=0.5, vmax=5.0, amax=25.0):
    return limit_position(target, previous, velocity, dt, step, vmax, amax)


def test_zero_error_and_bounded_braking():
    assert limited(0.0).position_deg == 0.0
    assert limited(0.0).velocity_deg_s == 0.0
    braking = limited(0.0, velocity=5.0)
    assert braking.velocity_deg_s == pytest.approx(3.75)
    assert braking.position_deg == pytest.approx(0.1875)


def test_velocity_limit_scales_with_actual_dt():
    command = limited(100.0, velocity=5.0, amax=100.0)
    assert command.position_deg == pytest.approx(0.25)
    assert command.velocity_deg_s == pytest.approx(5.0)


def test_acceleration_limit_scales_with_actual_dt():
    command = limited(100.0)
    assert command.velocity_deg_s == pytest.approx(1.25)
    assert command.position_deg == pytest.approx(0.0625)


def test_five_dps_profile_ramps_through_acceleration_limit():
    position = velocity = 0.0
    velocities = []
    for _ in range(5):
        command = limited(100.0, previous=position, velocity=velocity)
        position, velocity = command.position_deg, command.velocity_deg_s
        velocities.append(velocity)
    assert velocities == pytest.approx([1.25, 2.5, 3.75, 5.0, 5.0])


@pytest.mark.parametrize('dt', [0.04, 0.05, 0.07])
def test_irregular_dt_preserves_all_bounds(dt):
    old_velocity = 2.0
    command = limited(-100.0, velocity=old_velocity, dt=dt)
    delta = command.position_deg
    assert abs(delta) <= 5.0 * dt + 1e-12
    assert abs(delta) <= 0.5 + 1e-12
    assert abs(command.velocity_deg_s - old_velocity) <= 25.0 * dt + 1e-12
    assert command.velocity_deg_s == pytest.approx(delta / dt)


def test_step_is_an_additional_safety_cap():
    command = limited(100.0, velocity=5.0, dt=1.0, vmax=20.0,
                      amax=100.0)
    assert command.position_deg == pytest.approx(0.5)


def test_five_hz_step_cap_limits_requested_five_deg_per_second():
    dt = 0.2
    command = limited(100.0, dt=dt)
    flags = active_constraints(100.0, 0.0, 0.0, dt, 0.5, 5.0, 25.0,
                               command)
    assert command.velocity_deg_s == pytest.approx(2.5)
    assert flags['step_limited']
    assert not flags['velocity_limited']


@pytest.mark.parametrize('dt,vmax,amax,old_velocity,expected_stage,expected', [
    (0.2, 5.0, 25.0, 0.0, 'position_increment_after_step_limit', 0.5),
    (0.05, 5.0, 25.0, 0.0, 'velocity_after_acceleration_limit', 1.25),
    (0.05, 5.0, 25.0, 5.0, 'velocity_after_velocity_limit', 5.0),
])
def test_trace_reports_bound_without_changing_command(
        dt, vmax, amax, old_velocity, expected_stage, expected):
    command = limited(100.0, velocity=old_velocity, dt=dt,
                      vmax=vmax, amax=amax)
    trace = trace_limit_stages(100.0, 0.0, old_velocity, dt, 0.5,
                               vmax, amax, command)
    assert trace[expected_stage] == pytest.approx(expected)
    assert trace['final_v_cmd'] == command.velocity_deg_s
    assert trace['position_increment_after_step_limit'] == pytest.approx(
        command.position_deg
    )


def test_twenty_hz_five_deg_per_second_is_not_step_limited():
    dt = 0.05
    command = limited(100.0, velocity=5.0, dt=dt)
    flags = active_constraints(100.0, 0.0, 5.0, dt, 0.5, 5.0, 25.0,
                               command)
    assert command.position_deg == pytest.approx(0.25)
    assert command.velocity_deg_s == pytest.approx(5.0)
    assert flags['velocity_limited']
    assert not flags['step_limited']


def test_acceleration_diagnostic_on_start_and_reversal():
    starting = limited(100.0)
    start_flags = active_constraints(100.0, 0.0, 0.0, 0.05, 0.5, 5.0, 25.0,
                                     starting)
    assert start_flags['acceleration_limited']
    assert not start_flags['step_limited']
    reversing = limited(-100.0, velocity=5.0)
    reverse_flags = active_constraints(-100.0, 0.0, 5.0, 0.05, 0.5, 5.0,
                                       25.0, reversing)
    assert reverse_flags['acceleration_limited']


def test_direction_reversal_requires_deceleration():
    command = limited(-100.0, velocity=5.0)
    assert command.velocity_deg_s == pytest.approx(3.75)
    assert command.position_deg > 0


def test_rate_change_does_not_multiply_physical_speed():
    endpoints = []
    for rate in (5, 10, 20):
        position = velocity = 0.0
        dt = 1.0 / rate
        for _ in range(rate * 2):
            command = limited(100.0, previous=position, velocity=velocity,
                              dt=dt, vmax=2.5, amax=12.5)
            assert abs(command.position_deg - position) <= 2.5 * dt + 1e-12
            position, velocity = command.position_deg, command.velocity_deg_s
        endpoints.append(position)
    assert max(endpoints) - min(endpoints) <= 0.35
    assert all(0.0 <= position <= 5.0 for position in endpoints)


def test_five_deg_per_second_is_similar_at_ten_twenty_forty_hz():
    endpoints = []
    for rate in (10, 20, 40):
        position = velocity = 0.0
        dt = 1.0 / rate
        for _ in range(rate * 2):
            command = limited(100.0, previous=position, velocity=velocity,
                              dt=dt)
            assert abs(command.position_deg - position) <= 5.0 * dt + 1e-12
            position, velocity = command.position_deg, command.velocity_deg_s
        endpoints.append(position)
    assert max(endpoints) - min(endpoints) <= 0.2
    assert all(9.5 <= position <= 10.0 for position in endpoints)


def test_five_dps_profile_closes_same_target_gap_faster_than_baseline():
    endpoints = []
    for rate, vmax, amax in ((5, 2.5, 12.5), (20, 5.0, 25.0)):
        position = velocity = 0.0
        for _ in range(2 * rate):
            command = limited(40.0, previous=position, velocity=velocity,
                              dt=1.0 / rate, vmax=vmax, amax=amax)
            position, velocity = command.position_deg, command.velocity_deg_s
        endpoints.append(position)
    assert endpoints == pytest.approx([5.0, 9.625])
    assert 40.0 - endpoints[1] < 40.0 - endpoints[0]


def test_infeasible_limits_raise_instead_of_exceeding_cap():
    with pytest.raises(ValueError, match='conflict'):
        limit_position(100.0, 0.0, 5.0, 0.2, 0.01, 5.0, 0.1)


def test_phase2_profiles_change_only_the_intended_parameters():
    root = Path(__file__).resolve().parents[1] / 'config' / 'experiments'

    def load(name):
        return yaml.safe_load((root / f'{name}.yaml').read_text())[
            'leader_mirabo_teleop']['ros__parameters']

    baseline = load('baseline')
    for name, rate in (('rate_10', 10.0), ('rate_20', 20.0)):
        config = load(name)
        assert config['control_rate_hz'] == rate
        assert {k: v for k, v in config.items() if k != 'control_rate_hz'} == \
            {k: v for k, v in baseline.items() if k != 'control_rate_hz'}
    candidate = load('profile_c_velocity_5')
    assert candidate['control_rate_hz'] == 20.0
    assert candidate['max_velocity_deg_s'] == 5.0
    assert candidate['max_acceleration_deg_s2'] == 25.0
    assert candidate['max_step_deg'] == 0.5
    assert candidate['filter_alpha'] == 1.0
    assert math.isclose(candidate['max_velocity_deg_s'] /
                        candidate['control_rate_hz'], 0.25)
    assert load('phase2_20hz_5dps') == candidate
