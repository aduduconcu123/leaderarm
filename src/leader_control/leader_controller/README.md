# Leader to Mirabo teleop

Run the full leader-to-Mirabo chain with one command:

```bash
ros2 launch leader_controller teleop.launch.py
```

The launch file starts three separate nodes. Only `feetech_node` opens the
serial port; `leader_state` and `teleop` subscribe to ROS topics. Startup is
always disarmed. `ros2 run leader_controller teleop` starts only the CAN
controller when the upstream nodes are already running. The old `--arm` option
is rejected.

```text
feetech_driver/node.py -> /feetech/joint_states (absolute encoder radians)
leader_state/state.py  -> /leader/joint_states (calibrated joint radians)
leader_controller/teleop.py + mapping.py -> SocketCAN can0 -> Mirabo
```

`leader_state` uses `feetech_driver/calibration.py` and
`feetech_driver/calibration/calibration.json`. The Feetech node owns the serial
port exclusively; the Mirabo node receives calibrated joint angles by topic.
A persistent software origin at
`~/.config/leader_controller/origin.json` zeroes the published leader and
Mirabo feedback angles. The Feetech calibration landmarks, motor firmware
origin and absolute CAN position commands are unchanged.
The Mirabo controller requires a valid origin file before arming by default;
capture it explicitly with `calibrate_origin` before teleoperation.
`controller.py` provides separate FK, COM and gravity calculations; it is not
imported by the Mirabo position loop. Its gravity torque does not control Mirabo
torque.

## Configuration

Verified motor IDs and direction signs are in `leader_controller/mapping.py`.
Runtime rate, timeout, CAN interface, limiter and filter defaults are in
`config/teleop.yaml`; the launch file loads this configuration. Each parameter
is read only after startup. Absolute Mirabo mechanical limits are unknown and
are not configured.

The launch argument `teleop_config` accepts a full ROS parameter YAML file.
Direct `ros2 run leader_controller teleop` also reads the installed
`config/teleop.yaml` for defaults; ROS parameter overrides take precedence.

| Leader joint | Feetech ID | Mirabo ID | Command ID | Feedback ID | Sign |
| --- | --- | --- | --- | --- | --- |
| joint_2 | 3 | 0x68 | 0x668 | 0x2968 | -1, user tested |
| joint_3 | 2 | 0x69 | 0x669 | 0x2969 | -1, user tested |

`joint_1` (Feetech ID 1) does not control a follower.

`GEAR_RATIO=8` describes the 1:8 gearbox. By default,
`CAN_ANGLE_IS_MOTOR_SHAFT=False`: CAN angles are assumed to be output joint
angles, and command deltas use scale 1. Set it true only after confirming that
the firmware reports and accepts the motor shaft angle before the gearbox;
then command deltas use scale 8 and feedback JointState angles are divided by 8.
The 0.5-degree command step is in CAN-angle units. The limiter also caps speed
at 2.5 CAN degrees/s and acceleration at 12.5 CAN degrees/s^2. These are
software defaults, not verified mechanical ratings. The EMA filter defaults
to alpha=1 (no filtering). There is temporarily no
absolute software angle limit or homing move; confirm the real mechanical
travel before arming.
The angle convention, direction, speed/acceleration and mechanical travel still
need checking on the real machine.

### Limiter Equations

For each motor, the limiter uses actual elapsed steady time `dt` since the
previous sent command, in CAN degrees. For `dt > 0`:

```text
v_des = (q_filtered - q_previous) / dt
v_low = max(-vmax, v_previous - amax*dt, -max_step/dt)
v_high = min(vmax, v_previous + amax*dt, max_step/dt)
v_new = clamp(v_des, v_low, v_high)
q_new = q_previous + v_new*dt
```

Thus `|q_new-q_previous| <= vmax*dt`, `|v_new-v_previous| <= amax*dt`,
and `|q_new-q_previous| <= max_step`. A change in control rate changes the
per-cycle distance, not the configured physical velocity ceiling. If the
three intervals do not overlap, the limiter raises an error and the existing
command fault path disarms. If the target jumps to the current position while
the command is moving, a bounded braking step may temporarily cross the target
to honor acceleration. For `dt <= 0`, it holds position with zero velocity.

### Phase 2 Rate Experiments

The real `tracking_run_02` baseline is summarized in
`reports/phase2/PHASE2_REPORT.md`. Files in `config/experiments/` provide
rate-only 5, 10 and 20 Hz candidates with the same physical limits, then
`profile_c_velocity_5.yaml` as a first 20 Hz velocity experiment. The 40 Hz
rate-only candidate is optional and outside the staged baseline -> A -> B -> C
sequence. Stop the old launch before changing configuration. Each launch
starts disarmed; use a small movement of one joint at a time and arm
explicitly after status is READY.

From `~/leader_ws`, select one experiment:

```bash
ros2 launch leader_controller teleop.launch.py \
  teleop_config:=$PWD/src/leader_control/leader_controller/config/experiments/baseline.yaml \
  port:=/dev/ttyUSB0
```

Start `scripts/record_tracking.sh` in another sourced terminal for each run.
After stopping the bag, analyze it and append its measured summary to the
comparison CSV:

```bash
python3 src/leader_control/leader_controller/tools/analyze_tracking.py BAG_DIR
python3 src/leader_control/leader_controller/tools/compare_experiments.py \
  baseline=BAG_DIR/tracking_analysis/summary.json \
  rate_10=OTHER_BAG/tracking_analysis/summary.json
```

The comparison CSV now contains one measured baseline row. Additional rows
come only from recorded data. Compare both joints' limiter gap, command error,
estimated lag, peak command velocity, actual loop rate, jitter, CAN
TX/feedback rates and CAN warning/fault counts. If movement is insufficient,
lag remains `INSUFFICIENT EXCITATION`. Run the velocity candidate only after
the rate-only profiles are stable on hardware.

Protocol packing was checked against
`~/Downloads/Version1 (copy).zip`, under
`26-5-Control/src/can_interface/src/caninterface.cpp` and
`26-5-Control/src/motor_driver/src/ControlMotor.cpp`.
Absolute software angle limits are currently disabled. A feedback angle such
as -173 degrees can therefore be used as the initial motor reference, but CAN
and motor faults, stale data and invalid feedback still block arming.
Feedback IDs 0x2968/0x2969 follow the supplied protocol requirement; the C++
receiver itself masks the received ID down to the motor ID.
Commands are extended frames with big-endian `int32(angle_deg*10000)`,
`int16(speed/10)`, `int16(accel/10)`; the angle first uses C++ float precision,
then integer conversion truncates toward zero as in C++.
The feedback angle is signed big-endian int16 / 10, and byte 7 is the fault.

## Build

```bash
cd ~/leader_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables leader_controller
```

## Run

In every terminal first run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/leader_ws/install/setup.bash
```

`can0` must already be UP at 1,000,000 bit/s. Inspect it with
`ip -details link show can0`. The node does not reconfigure the interface;
passing `bitrate` to python-can SocketCAN does not change the kernel bitrate.

### Capture Current Pose As Zero

With leader and both Mirabo motors powered, put them in the desired reference
pose. Stop any running `teleop` or separate Feetech node so the serial port has
one owner. Then run the one-shot, read-only calibration command:

```bash
ros2 run leader_controller calibrate_origin --ros-args -p port:=/dev/ttyUSB0
```

Change `/dev/ttyUSB0` if the Feetech adapter uses another port. The command
starts Feetech and leader state readers, waits up to 10 seconds for fresh
leader angles and fault-free feedback from both Mirabo IDs on `can0`, saves
`~/.config/leader_controller/origin.json` atomically, then exits. It sends no
motor position, torque, set-origin or homing commands. If data is missing or a
CAN/motor fault occurs, it exits without replacing the saved origin.

The saved origin is a software offset: `/leader/joint_states` and
`/leader_controller/mirabo_feedback` report zero at the captured pose after
nodes restart. Feetech mechanical calibration and limits stay in
`feetech_driver/calibration/calibration.json`; Mirabo commands still use its
absolute CAN angle. Running calibration again replaces the logical zero. To
use a different path, pass `-p origin_file:=/path/to/origin.json` to both the
calibration and teleop commands.

Check the new origin without moving the follower:

```bash
ros2 launch leader_controller teleop.launch.py
```

In another terminal, inspect `/leader/joint_states`,
`/leader_controller/mirabo_feedback` and
`/leader_controller/mirabo_status`. Both angle topics should be near zero in
the captured pose, and status should be `READY` before arming. Keep this one
launch session running; arm from another terminal after checking status.

The Feetech port defaults to `/dev/ttyUSB0`. For another port:

```bash
ros2 launch leader_controller teleop.launch.py port:=/dev/ttyUSB1
```

Do not launch another Feetech reader on the same serial port. The driver opens
it exclusively, and a second leader publisher could provide conflicting data.

From another terminal, inspect status including CAN angles and sample ages:

```bash
ros2 topic echo /leader_controller/mirabo_status
```

The feedback topic reports follower angles in radians:

```bash
ros2 topic echo /leader_controller/mirabo_feedback
```

## Tracking instrumentation

The telemetry topics are observational and do not feed back into control:

- `/leader_controller/reference`: mapped desired position, in radians.
- `/leader_controller/filtered_reference`: EMA output, in radians.
- `/leader_controller/command`: limited position and velocity, in rad and rad/s.
- `/mirabo/joint_states`: one stamped sample per valid CAN feedback frame,
  after the existing origin and scale, in radians. Messages can contain just
  one joint; always match by `name`.
- `/leader_controller/diagnostics`: JSON with per-joint errors, monotonic stage
  timestamps (ns), and measured event rates. CAN feedback rates are counted
  from decoded CAN receive events rather than the feedback ROS publisher.

Record without launching or arming the robot:

```bash
src/leader_control/leader_controller/scripts/record_tracking.sh [bag_directory]
```

The script prints the bag directory it is creating. Keep it running while
moving one joint at a time, then stop it with Ctrl-C before analysis. If no
directory argument is given, it uses a timestamped `tracking_YYYYMMDD_HHMMSS`
name; pass that actual directory name to the analyzer.
Existing bag directories are never overwritten; give each run a new name.

Stop recording with Ctrl-C. Analyze a bag after sourcing the ROS environment:

```bash
python3 src/leader_control/leader_controller/tools/analyze_tracking.py BAG_DIRECTORY
```

The analysis directory contains `tracking_samples.csv` with every recorded
leader, reference, command and follower angle sample and its ROS and bag
receive timestamps. `tracking_aligned.csv` has one row per follower feedback
sample with the most recently observed values from the other signals; its
`leader_age_ms` and `command_age_ms` columns show how old those held values
are. The raw samples, not the held values, are used for lag estimates. The
same directory also contains `tracking_68.png`, `tracking_69.png`,
`summary.json`, and `summary.txt`. Summary lag estimates include leader to
follower, desired to command, and command to follower. They are based on
cross-correlation of motion using ROS sample timestamps and are reported only
when the data has enough movement. CAN receive stamps mark processing of the
received frame, so polling and message scheduling still bound timing accuracy.

After checking status is `READY`, arm or stop commands explicitly:

```bash
ros2 param set /leader_mirabo_teleop armed true
ros2 param set /leader_mirabo_teleop armed false
```

The node starts disarmed, even with an `armed:=true` ROS parameter override.
Arming is rejected until the leader and both feedback streams are fresh and
fault-free. At the first armed timer tick, the current leader and motor angles
become the reference, so the first target is each motor's current angle. This
reference is captured when arming, not when starting disarmed. Subsequent
targets follow signed leader deltas without an absolute travel limit.
One 5 Hz timer sends the pair of commands with step, velocity and acceleration
limits in CAN degrees.
Each pair requires a new leader sample and new feedback from both motors.
The two frames are sequential CAN writes, not an atomic hardware transaction.

Missing data for more than 0.5 s, malformed leader input, CAN BUS-OFF,
receive/send errors or motor faults stop commands and reset the actual `armed`
parameter to false. Transient CAN error frames and malformed Mirabo feedback
are logged and skipped. Only valid frames refresh each motor's feedback age;
persistent errors therefore stop commands through the feedback timeout.
A full receive batch is processed on the next timer tick. Timers and timeouts
use a steady/monotonic clock, independent of ROS simulation time. Status retains
FAULT, its code, reason and time until `clear_fault` succeeds with all inputs
healthy. Clearing never arms. `estop` latches a software command stop and needs
the separate `clear_estop` service. Neither service changes motor torque.
Status is WAIT_LEADER, WAIT_CAN, READY, ARMED, FAULT or ESTOP. The status topic
also reports CAN state, last command and feedback ages. Stale follower feedback
is not republished as a fresh JointState.

```bash
ros2 service call /leader_controller/clear_fault std_srvs/srv/Trigger '{}'
ros2 service call /leader_controller/estop std_srvs/srv/Trigger '{}'
ros2 service call /leader_controller/clear_estop std_srvs/srv/Trigger '{}'
```

Stopping commands, closing the node or losing communication does not disable
torque: a motor can continue toward or hold its last target. No set-origin,
MIT-mode, torque-disable or automatic homing commands are sent.
