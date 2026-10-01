# Phase 2 runtime parameters before the step-cap diagnostic

On 2026-10-01, `ros2 node list` returned no nodes and
`ros2 param list /leader_mirabo_teleop` returned `Node not found`. A live
parameter query cannot be reported for this session. No robot launch or arm
was performed to obtain one.

The latest available bag, `phase2_baseline_20261001_161139`, contains
diagnostics emitted by the running teleop node. These values are stronger
evidence than a YAML file alone, but describe that recording, not an
unobserved later launch:

| Parameter | Runtime value in bag | Default YAML | Observed behavior |
| --- | ---: | ---: | --- |
| `control_rate_hz` | 5.0 | 5.0 | actual 5.000 Hz |
| `max_velocity_deg_s` | 2.5 | 2.5 | peak `v_cmd` 2.5 deg/s, both motors |
| `max_acceleration_deg_s2` | 12.5 | 12.5 | not independently identifiable from peak velocity |
| `max_step_deg` | 0.5 | 0.5 | nominal step cap 2.5 deg/s at 5 Hz |
| `filter_alpha` | not recorded | 1.0 | `q_des` and `q_filtered` nearly overlap |

This bag recorded 3,454 non-fatal CAN warning frames, with no bus-off or
motor fault events. It is not a clean safety baseline.

The user's attempted `max_velocity_deg_s=5.0` setting is not confirmed by
this bag: its recorded runtime value was **2.5**. At 5 Hz, a 0.5 deg/step
cap would also restrict a correctly loaded 5 deg/s velocity setting to a
nominal 2.5 deg/s. These are distinct possible bottlenecks and must not be
conflated.

## Runtime parameter path

`teleop.launch.py` passes the selected `teleop_config` YAML to the teleop
node. `MiraboTeleop.__init__()` loads defaults from the installed
`config/teleop.yaml`, declares each parameter with a read-only descriptor,
then reads the ROS-loaded values with `get_parameter()` into `self.config`.
Thus the selected launch YAML overrides the defaults before the timer,
filters or command calculations are initialized.

| Parameter | Declaration and load | Runtime consumer |
| --- | --- | --- |
| `control_rate_hz` | `teleop.py`, `__init__()`, `self.config` | `create_timer(1.0 / self.config['control_rate_hz'])` |
| `max_velocity_deg_s` | same | passed from `self.config` to `limit_position()` for each motor command |
| `max_acceleration_deg_s2` | same | passed from `self.config` to `limit_position()` for each motor command |
| `max_step_deg` | same | passed from `self.config` to `limit_position()` for each motor command |
| `filter_alpha` | same | passed to each `EmaFilter` constructor after parameter load |

There is **no limiter object** storing a module-level 2.5 deg/s default.
`limit_position()` is a pure function called with the loaded `self.config`
values for both 0x68 and 0x69. Its previous command position, velocity and
monotonic timestamp are reset from current feedback when manually arming.

After manually launching a profile, check the actual parameters with:

```bash
ros2 param list /leader_mirabo_teleop
ros2 param get /leader_mirabo_teleop control_rate_hz
ros2 param get /leader_mirabo_teleop max_velocity_deg_s
ros2 param get /leader_mirabo_teleop max_acceleration_deg_s2
ros2 param get /leader_mirabo_teleop max_step_deg
ros2 param get /leader_mirabo_teleop filter_alpha
```

Compare those values with `/leader_controller/diagnostics` and the measured
`/leader_controller/command` velocity; a configured rate alone does not prove
the effective command rate or velocity.
