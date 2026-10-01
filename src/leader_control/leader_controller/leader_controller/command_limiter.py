"""Deterministic command limits in Mirabo CAN angle units."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LimitedCommand:
    position_deg: float
    velocity_deg_s: float


def limit_position(target_deg, previous_deg, previous_velocity_deg_s,
                   elapsed_sec, max_step_deg, max_velocity_deg_s,
                   max_acceleration_deg_s2):
    """Limit one command using elapsed steady time and prior sent state."""
    values = (target_deg, previous_deg, previous_velocity_deg_s,
              elapsed_sec, max_step_deg, max_velocity_deg_s,
              max_acceleration_deg_s2)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('command limiter values must be finite')
    if (max_step_deg <= 0 or max_velocity_deg_s <= 0
            or max_acceleration_deg_s2 <= 0):
        raise ValueError('command limits must be positive')
    if elapsed_sec <= 0:
        return LimitedCommand(previous_deg, 0.0)

    desired_velocity = (target_deg - previous_deg) / elapsed_sec
    step_velocity = max_step_deg / elapsed_sec
    lower = max(-max_velocity_deg_s,
                previous_velocity_deg_s - max_acceleration_deg_s2 * elapsed_sec,
                -step_velocity)
    upper = min(max_velocity_deg_s,
                previous_velocity_deg_s + max_acceleration_deg_s2 * elapsed_sec,
                step_velocity)
    if lower > upper:
        raise ValueError('velocity, acceleration and step limits conflict')
    velocity = max(lower, min(desired_velocity, upper))
    return LimitedCommand(previous_deg + velocity * elapsed_sec, velocity)


def active_constraints(target_deg, previous_deg, previous_velocity_deg_s,
                       elapsed_sec, max_step_deg, max_velocity_deg_s,
                       max_acceleration_deg_s2, command):
    """Report which bounds clipped the desired velocity, without changing it."""
    flags = {name: False for name in (
        'velocity_limited', 'acceleration_limited', 'step_limited',
    )}
    if elapsed_sec <= 0:
        return flags
    desired_velocity = (target_deg - previous_deg) / elapsed_sec
    if math.isclose(desired_velocity, command.velocity_deg_s,
                    rel_tol=1e-9, abs_tol=1e-9):
        return flags
    direction = 1 if desired_velocity > command.velocity_deg_s else -1
    bound = command.velocity_deg_s
    flags['velocity_limited'] = math.isclose(
        bound, direction * max_velocity_deg_s, rel_tol=1e-9, abs_tol=1e-9,
    )
    flags['acceleration_limited'] = math.isclose(
        bound, previous_velocity_deg_s + direction *
        max_acceleration_deg_s2 * elapsed_sec,
        rel_tol=1e-9, abs_tol=1e-9,
    )
    flags['step_limited'] = math.isclose(
        bound, direction * max_step_deg / elapsed_sec,
        rel_tol=1e-9, abs_tol=1e-9,
    )
    return flags


def trace_limit_stages(target_deg, previous_deg, previous_velocity_deg_s,
                       elapsed_sec, max_step_deg, max_velocity_deg_s,
                       max_acceleration_deg_s2, command):
    """Observe each bound in CAN degrees without changing the command."""
    if elapsed_sec <= 0:
        return {
            'requested_velocity': 0.0,
            'velocity_after_velocity_limit': 0.0,
            'velocity_after_acceleration_limit': 0.0,
            'position_increment_before_step_limit': 0.0,
            'position_increment_after_step_limit': 0.0,
            'final_v_cmd': command.velocity_deg_s,
        }

    requested = (target_deg - previous_deg) / elapsed_sec
    velocity_limited = max(-max_velocity_deg_s,
                           min(requested, max_velocity_deg_s))
    acceleration_limited = max(
        previous_velocity_deg_s - max_acceleration_deg_s2 * elapsed_sec,
        min(velocity_limited,
            previous_velocity_deg_s + max_acceleration_deg_s2 * elapsed_sec),
    )
    increment_before_step = acceleration_limited * elapsed_sec
    increment_after_step = max(-max_step_deg,
                               min(increment_before_step, max_step_deg))
    return {
        'requested_velocity': requested,
        'velocity_after_velocity_limit': velocity_limited,
        'velocity_after_acceleration_limit': acceleration_limited,
        'position_increment_before_step_limit': increment_before_step,
        'position_increment_after_step_limit': increment_after_step,
        'final_v_cmd': command.velocity_deg_s,
    }
