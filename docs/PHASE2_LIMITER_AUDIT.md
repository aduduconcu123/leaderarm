# Phase 2 limiter audit

This describes the implementation present before the Phase 2 profile/report
changes. All angles and limits here use Mirabo CAN degrees, not radians.

## Measured baseline

`tracking_run_02` is an 80.3 s recording at the 5 Hz baseline. The measured
control rate is 5.000 Hz; CAN TX is about 10.21 frames/s (two frames per
command pair); leader callbacks are about 20.00 Hz; feedback is about 50.00 Hz
per motor. Peak command velocity is 0.043633 rad/s = 2.5 deg/s for both motors.

| Motor | RMS desired-command gap | RMS command-actual error | Peak desired-command gap |
| --- | ---: | ---: | ---: |
| 0x68 | 0.094548 rad | 0.013478 rad | 0.248871 rad |
| 0x69 | 0.327639 rad | 0.016491 rad | 0.870308 rad |

The plots show `q_des` and `q_filtered` overlap at alpha=1. The commanded
position ramps near 2.5 deg/s while the measured follower stays near the
command. The gap dominates tracking error in this recording; this is a
baseline observation, not a safety rating or a prediction for another rate.
Cross-correlation lag depends on the motion waveform and ROS timestamping.

## Exact limiter equation

For `dt > 0`, the current `limit_position()` computes:

```text
v_des = (q_target - q_previous) / dt
v_step = max_step_deg / dt
v_lower = max(-vmax, v_previous - amax*dt, -v_step)
v_upper = min(+vmax, v_previous + amax*dt, +v_step)
v_new = clamp(v_des, v_lower, v_upper)
q_new = q_previous + v_new*dt
```

If `v_lower > v_upper`, the configured constraints conflict and the limiter
raises `ValueError`; the existing CAN command fault path disarms. For `dt <= 0`
it holds the previous position and reports zero velocity.

1. Position step: `|q_new-q_previous| <= max_step_deg` because the feasible
   velocity range includes `+-max_step_deg/dt`.
2. Velocity: `v_new` is clamped to `+-vmax`; therefore the position change is
   at most `vmax*dt`.
3. Acceleration: `v_new` is clamped to `v_previous +- amax*dt`.
4. `dt` is the actual elapsed monotonic time since the previous sent command,
   computed in `MiraboTeleop.command_for_mapping()`. No fixed timer period is
   passed to the limiter.
5. `max_step_deg` is applied simultaneously with velocity and acceleration
   through the feasible range, not as a later clamp that could violate the
   acceleration bound.
6. On READY -> ARMED, `arm_from_current_positions()` resets both command
   positions to the latest follower CAN angles, both command velocities to
   zero, timestamps to the arming time, and EMA state to follower angles.
   First target and command are therefore the current follower positions.
   On DISARM the node stops sending and clears `last_sent_stamps`; previous
   limiter state remains stored until the next ARM replaces it.
7. `MiraboTeleop.__init__()` creates the steady-clock control timer with
   `period=1.0/self.config['control_rate_hz']`. The CAN polling timer has a
   separate configurable rate.

The control loop sends a pair only when the leader and both feedback samples
are fresh and newer than the last sent pair. Raising the timer rate does not
guarantee the same increase in CAN TX pairs. The timer, limiter and CAN packet
fields are separate: `COMMAND_SPEED` and `COMMAND_ACCEL` remain 1000.

## Rate-independence implication

At 20 Hz, `vmax=2.5 deg/s` permits about 0.125 deg in a nominal 50 ms cycle;
at 5 Hz it permits 0.5 deg in 200 ms. Thus rate-only profiles increase
temporal resolution without multiplying the configured physical speed. The
actual `dt` and delivered TX rate still need measurement on hardware.

## Step-cap diagnostic added after audit

The limiter equation above was retained. At startup, the node now logs its
loaded control rate, velocity, acceleration, step and filter parameters, plus
the **nominal** step velocity cap `max_step_deg * control_rate_hz`. It warns
when this cap is below the configured velocity limit. Diagnostics also report
that nominal cap and per-motor `velocity_limited`,
`acceleration_limited` and `step_limited` flags for the command computed in
the current control tick. If no command was computed on that tick, the
per-motor flags are null and `limiter_flags_valid` is false.

At 5 Hz, a 0.5 deg step cap permits no more than 2.5 deg/s at the nominal
period, even if `vmax` were configured as 5 deg/s. At 20 Hz, the same step
cap corresponds to 10 deg/s nominally, so a 5 deg/s velocity limit can bind
first. This is a configuration calculation, not a measured motor velocity.
The actual command interval and fresh-input gating can reduce effective TX
rate; `v_cmd` and recorded CAN TX events remain the physical-test evidence.
