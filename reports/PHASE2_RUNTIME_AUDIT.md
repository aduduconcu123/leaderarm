# Phase 2 runtime profile audit (2026-10-01)

## Finding

The most recent available hardware launch log,
`~/.ros/log/2026-10-01-16-54-21-650933-anwirnd-35892/launch.log`,
shows the teleop node actually started with the baseline profile:

| Parameter | Old active value | Intended profile value |
| --- | ---: | ---: |
| `control_rate_hz` | 5 Hz | 20 Hz |
| control timer period | 0.2 s | 0.05 s |
| `max_velocity_deg_s` (0x68, 0x69) | 2.5 deg/s | 5 deg/s |
| `max_acceleration_deg_s2` (0x68, 0x69) | 12.5 deg/s² | 25 deg/s² |
| `max_step_deg` | 0.5 deg | 0.5 deg |
| `filter_alpha` | 1 | 1 |
| nominal `max_step_deg * control_rate_hz` | 2.5 deg/s | 10 deg/s |

The 16:32, 16:33 and 16:52 launch logs show the same baseline values.
The root cause of the observed 2.5 deg/s cap in these runs is **active
baseline parameters**, not a hidden downstream clamp. The launch command
used in those runs is not saved in the logs, so the precise operator-side
reason the experimental YAML was not selected cannot be established.
There is no currently running teleop process to query for active values.
No newer bag was found in this workspace; the older baseline bag reports
an actual 4.999965 Hz control rate, yielding an effective nominal step
velocity cap of about 2.499983 deg/s. Do not attribute that measured rate
to an unrecorded later run.

## Code path

`launch/teleop.launch.py:generate_launch_description()` defaults to
`config/teleop.yaml` and passes `teleop_config` directly to the teleop node
as its ROS parameter file. The launch now logs that resolved file path.
`leader_controller/teleop.py:MiraboTeleop.__init__()` first reads baseline
YAML as declaration defaults, then reads the effective ROS parameter values
into `self.config`, creates filters from those values, and finally creates
the control timer. It does not reload baseline settings afterward.

There are no per-motor `CommandLimiter` instances storing a separate
`vmax`: `MiraboTeleop.command_for_mapping()` calls the stateless
`command_limiter.limit_position()` with `self.config` on each tick for each
motor. The startup `ACTIVE TELEOP PROFILE` log now prints the effective
control rate, actual ROS timer period, both motors' vmax/amax, max step,
nominal step cap, and filter alpha. The diagnostics now contain a per-motor
`limiter_trace_can_units` with the requested velocity, velocity after the
velocity bound, velocity after the acceleration bound, position increment
before and after the step bound, and final `v_cmd`. Velocity fields are CAN
deg/s and increments are CAN degrees. This trace is observational; it does
not feed the CAN command. `mapping.COMMAND_SPEED` and `COMMAND_ACCEL` are
unchanged CAN payload fields and are not a 2.5 deg/s software clamp.
Diagnostics also report `actual_control_rate_hz` from callback timestamps
and `effective_step_velocity_cap_actual = max_step_deg *
actual_control_rate_hz`; unlike the nominal cap, these reflect the measured
callback rate.

Search results for `2.5`, `MAX_VELOCITY`, `max_velocity`, `max_step`, and
velocity/speed clamp across source, build, install, docs and reports found
baseline and rate-only YAML values of 2.5, but no other runtime 2.5 clamp.
The only software command bounds are `max_velocity_deg_s`,
`max_acceleration_deg_s2`, and `max_step_deg` in `limit_position()`.

## Offline verification

The exact `config/experiments/phase2_20hz_5dps.yaml` was loaded as ROS
parameter overrides in the hardware-free teleop test. It created a 50 ms
timer and, with a large target error and fake feedback, produced `v_cmd`
of 1.25, 2.5, 3.75, 5.0, 5.0 deg/s for **both** motors. The trace reports
the 5 deg/s velocity bound and a 0.25 deg final increment, below the
0.5 deg step cap. This confirms the code path can exceed 2.5 deg/s; it
does **not** prove the intended profile has run on physical hardware.

## One launch command

From `~/leader_ws` after sourcing ROS and `install/setup.bash`, with the
previous launch stopped:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=/home/phong/leader_ws/src/leader_control/leader_controller/config/experiments/phase2_20hz_5dps.yaml
```

Before manually arming, the launch must print `Teleop parameter file:`
with that exact path and `ACTIVE TELEOP PROFILE` with 20 Hz, 0.05 s,
5 deg/s vmax for 0x68 and 0x69, 25 deg/s² amax for each, and a 10 deg/s
nominal step cap. In a second sourced terminal:

```bash
ros2 param get /leader_mirabo_teleop control_rate_hz
ros2 param get /leader_mirabo_teleop max_velocity_deg_s
ros2 param get /leader_mirabo_teleop max_acceleration_deg_s2
ros2 param get /leader_mirabo_teleop max_step_deg
ros2 param get /leader_mirabo_teleop filter_alpha
ros2 topic echo --once /leader_controller/diagnostics
ros2 topic hz /leader_controller/diagnostics
ros2 topic hz /leader_controller/command
```

Check `rates_hz.control` and successive `timestamps_ns.t_control_loop`
in diagnostics for actual timer callbacks. `ros2 topic hz` can measure
less than 20 Hz when commands are not published (for example, while
disarmed). Do not arm automatically or increase `vmax` further. The new
active value on hardware remains **unverified** until the next launch is
observed.
