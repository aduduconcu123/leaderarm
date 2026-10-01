# Mirabo command-field characterization: manual runbook

This document prepares future **manual** trials only. No command here was
run by Codex on hardware. The two CAN fields are named speed and
acceleration in source, but their physical meaning is unverified. The
working Phase 2 profile is unchanged. Every trial starts disarmed and
requires the operator to arm explicitly.

## Before every run

Use the same known-safe initial pose, with leader and Mirabo away from
mechanical limits. Stop the previous launch completely so there is only
one Feetech serial owner and one teleop node. Keep the known-good software
origin; recalibrate separately only when that origin is genuinely wrong.
Source both setup files in **each** terminal:

```bash
cd ~/leader_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Use terminal 1 for the selected launch below and leave it open. The
launch must log the selected `Teleop parameter file`, `ACTIVE TELEOP
PROFILE` (20 Hz, 5 deg/s, 25 deg/s², 0.5 deg step, alpha 1), and `MIRABO
COMMAND PROFILE` with the field values shown for that run. It starts
disarmed. Before starting motion, run these in terminal 2:

```bash
ros2 param get /leader_mirabo_teleop control_rate_hz
ros2 param get /leader_mirabo_teleop max_velocity_deg_s
ros2 param get /leader_mirabo_teleop max_acceleration_deg_s2
ros2 param get /leader_mirabo_teleop max_step_deg
ros2 param get /leader_mirabo_teleop filter_alpha
ros2 param get /leader_mirabo_teleop mirabo_command_speed
ros2 param get /leader_mirabo_teleop mirabo_command_acceleration
ros2 topic echo --once /leader_controller/mirabo_status
ros2 topic echo --once /leader_controller/diagnostics
```

Require READY, no motor fault bytes, the expected field values, and
normal diagnostics before manually arming. For every run start the
recorder in terminal 2, then in a separately sourced terminal 3 arm with
`ros2 param set /leader_mirabo_teleop armed true`. Move **one joint at a
time**, through the same small, known-safe range at approximately the
same leader speed; hold at the target for about 3 seconds, return slowly
to the start and hold for about 3 seconds. Repeat for the other joint.
Do not approach a mechanical limit. Human repeatability is imperfect,
so compare like-for-like trials and do not overinterpret small differences.

After each run, disarm from terminal 3:

```bash
ros2 param set /leader_mirabo_teleop armed false
```

Then stop rosbag in terminal 2 with Ctrl-C, dump runtime parameters and
analyze there using that run's commands below. Stop the launch in terminal
1 with Ctrl-C before selecting the next profile. The recorder refuses an
existing bag directory; use a new suffix if a name is already taken.

**Stop/disarm immediately** for an unexpected jump, rapid oscillation,
violent motion, unusual mechanical noise, large sustained
`q_cmd-q_actual` error, motor fault byte nonzero, CAN bus-off, loss of
feedback, or unexpected command discontinuity. Disarming stops commands
but does **not** remove motor torque. Review CAN warning counts too; the
earlier Phase 2 bag recorded 1412 warnings even without bus-off or fault,
so do not assume a new warning count was caused by a profile field alone.

## A. Baseline command profile

Expected speed=1000 (raw 100), acceleration=1000 (raw 100). Terminal 1:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_speed_baseline.yaml
```

Terminal 2, after the common parameter/status checks:

```bash
BAG=phase3_speed_baseline_01
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
```

After the common one-joint-at-a-time motion, disarm and Ctrl-C the bag:

```bash
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

## B. Speed lower

Expected speed=900 (raw 90), acceleration=1000 (raw 100); only the
speed field changes. Repeat the common checks and motion:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_speed_low.yaml
```

In terminal 2, start this bag before manual arm, then disarm/Ctrl-C and
analyze afterward:

```bash
BAG=phase3_speed_low_01
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

The last two lines are run **after** Ctrl-C stops `record_tracking.sh`.

## C. Speed higher

Expected speed=1100 (raw 110), acceleration=1000 (raw 100); only the
speed field changes. Proceed only if A and B had acceptable motion and
fault status:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_speed_high.yaml
```

Terminal 2:

```bash
BAG=phase3_speed_high_01
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

Again, execute the last two lines only after disarming and stopping the bag.

## D. Return to baseline

Repeat the baseline before touching acceleration so drift/repeatability
can be judged. Expected fields are both 1000 (raw 100):

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_accel_baseline.yaml
```

Terminal 2:

```bash
BAG=phase3_accel_baseline_02
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

## E. Acceleration lower

Expected speed=1000 (raw 100), acceleration=900 (raw 90); only the
acceleration field changes:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_accel_low.yaml
```

Terminal 2:

```bash
BAG=phase3_accel_low_01
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

## F. Acceleration higher

Expected speed=1000 (raw 100), acceleration=1100 (raw 110); only the
acceleration field changes. Proceed only if earlier stages were acceptable:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB0 teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/phase3_accel_high.yaml
```

Terminal 2:

```bash
BAG=phase3_accel_high_01
src/leader_control/leader_controller/scripts/record_tracking.sh "$BAG"
ros2 param dump /leader_mirabo_teleop > "$BAG/runtime_params.yaml"
python3 src/leader_control/leader_controller/tools/analyze_tracking.py "$BAG"
```

For sections D-F, as above, the dump/analyze commands are run only after
manual disarm and Ctrl-C has finished recording. For each bag inspect
`tracking_analysis/summary.json`, `summary.txt`, `tracking_68.png`,
`tracking_69.png`, `velocity_68.png`, `velocity_69.png`, and the
`runtime_params.yaml` snapshot. Confirm `command_profile.status=RECORDED`
and that its configured/raw values match the stage; otherwise do not
compare that bag as a valid profile trial. Overshoot/settling may be null
with `INSUFFICIENT STEP/HOLD`, which is preferable to a fabricated value.

After all analyzed bags exist, compare without ranking:

```bash
python3 src/leader_control/leader_controller/tools/compare_mirabo_profiles.py \
  first_baseline=phase3_speed_baseline_01 \
  speed_low=phase3_speed_low_01 \
  speed_high=phase3_speed_high_01 \
  return_baseline=phase3_accel_baseline_02 \
  accel_low=phase3_accel_low_01 \
  accel_high=phase3_accel_high_01
```

The table prints separately for 0x68 and 0x69 and is saved to
`reports/phase3_profile_comparison.csv`. Compare RMS/max command error,
lag, overshoot/settling status, CAN warnings, fault status, actual loop
rate, and whether the motion patterns were truly similar. No script
selects a preferred field value automatically.
