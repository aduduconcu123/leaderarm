# Leader to Mirabo teleop

Run the full leader-to-Mirabo chain with one command:

```bash
ros2 run leader_controller teleop --arm
```

This starts the three ROS nodes in one process. The `--arm` flag expresses
intent to move; startup sends no position command until the calibrated leader
and both Mirabo feedback streams are fresh and fault-free. A startup fault
cancels the pending arm request. Without `--arm`, the process starts all three
nodes but stays disarmed until `armed` is set manually.

```text
feetech_driver/node.py -> /feetech/joint_states (absolute encoder radians)
leader_state/state.py  -> /leader/joint_states (calibrated joint radians)
leader_controller/teleop.py + mapping.py -> SocketCAN can0 -> Mirabo
```

`leader_state` uses `feetech_driver/calibration.py` and
`feetech_driver/calibration/calibration.json`. The Feetech node in this process
owns the serial port; the Mirabo node receives already calibrated joint angles.
`controller.py` provides separate FK, COM and gravity calculations; it is not
imported by the Mirabo position loop. Its gravity torque does not control Mirabo
torque.

## Configuration

All motor mappings and motion settings are in `leader_controller/mapping.py`.

| Leader joint | Feetech ID | Mirabo ID | Command ID | Feedback ID | Sign | CAN software limits |
| --- | --- | --- | --- | --- | --- | --- |
| joint_2 | 3 | 0x68 | 0x668 | 0x2968 | +1, existing config | -15 to 90 degrees |
| joint_3 | 2 | 0x69 | 0x669 | 0x2969 | -1, user tested | 0 to 90 degrees |

`joint_1` (Feetech ID 1) does not control a follower.
J2 direction still needs physical confirmation; the existing +1 is retained.

`GEAR_RATIO=8` describes the 1:8 gearbox. By default,
`CAN_ANGLE_IS_MOTOR_SHAFT=False`: CAN angles are assumed to be output joint
angles, and command deltas use scale 1. Set it true only after confirming that
the firmware reports and accepts the motor shaft angle before the gearbox;
then command deltas use scale 8 and feedback JointState angles are divided by 8.
Limits and the 2-degree command step are always in CAN-angle units.
The angle convention, direction, speed/acceleration and mechanical travel still
need checking on the real machine.

Protocol packing and software limits were checked against
`~/Downloads/Version1 (copy).zip`, under
`26-5-Control/src/can_interface/src/caninterface.cpp` and
`26-5-Control/src/motor_driver/src/ControlMotor.cpp`.
These are software limits from that code, not confirmed mechanical limits.
Feedback IDs 0x2968/0x2969 follow the supplied protocol requirement; the C++
receiver itself masks the received ID down to the motor ID.
Commands are extended frames with big-endian `int32(angle_deg*10000)`,
`int16(speed/10)`, `int16(accel/10)`; the angle first uses C++ float precision,
then integer conversion truncates toward zero as in C++.
The feedback angle is signed big-endian int16 / 10, and byte 7 is the fault.

## Build And Offline Test

```bash
cd ~/leader_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg executables leader_controller
colcon test --packages-select leader_controller --pytest-args test/test_teleop.py
```

The offline tests replace CAN with an in-memory bus and use ROS domain 231.
No physical motor is armed. The explicit test path runs the teleop regression
tests separately from the package's generic lint checks.

## Run

In every terminal first run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/leader_ws/install/setup.bash
```

`can0` must already be UP at 1,000,000 bit/s. Inspect it with
`ip -details link show can0`. The node does not reconfigure the interface;
passing `bitrate` to python-can SocketCAN does not change the kernel bitrate.

One terminal starts the complete chain:

```bash
ros2 run leader_controller teleop --arm
```

To inspect feedback first without motion, omit `--arm`:

```bash
ros2 run leader_controller teleop
```

The Feetech port defaults to `/dev/ttyUSB0`. For another port:

```bash
ros2 run leader_controller teleop --arm --ros-args -p port:=/dev/ttyUSB1
```

Do not also run a separate `feetech_node` or `leader_state` process alongside
the one-command process. The Feetech serial port is already open and a second
publisher could provide conflicting leader samples.

From another terminal, inspect status including CAN angles and sample ages:

```bash
ros2 topic echo /leader_controller/mirabo_status
```

The feedback topic reports follower angles in radians:

```bash
ros2 topic echo /leader_controller/mirabo_feedback
```

To stop commands or arm after starting without `--arm`:

```bash
ros2 param set /leader_mirabo_teleop armed true
ros2 param set /leader_mirabo_teleop armed false
```

The node starts disarmed, even with an `armed:=true` ROS parameter override.
`--arm` is handled at runtime once all streams pass the readiness checks.
Arming is rejected until the leader and both feedback streams are fresh,
fault-free and within the CAN software limits. At the first armed timer tick,
the current leader and motor angles become the reference, so the first target
is each motor's current angle. Subsequent targets follow signed leader deltas.
One 20 Hz timer sends the pair of commands, limited to 2 CAN degrees per cycle.
Each pair requires a new leader sample and new feedback from both motors.
The two frames are sequential CAN writes, not an atomic hardware transaction.

Missing data for more than 0.5 s, malformed input, CAN error frames, receive/send
errors or motor faults stop commands and reset the actual `armed` parameter to
false. Timers and timeouts use a steady/monotonic clock, independent of ROS
simulation time. Status retains FAULT and its reason until explicitly
acknowledged with `armed false` or a successful new `armed true` request.
Recovery does not automatically restart motion; rearming latches new references.
Status is READY when disarmed with valid inputs, DISARMED when waiting for inputs,
ARMED while enabled, and FAULT after an error. Feedback angles may be stale;
check the sample ages in the status topic.

Stopping commands, closing the node or losing communication does not disable
torque: a motor can continue toward or hold its last target. No set-origin,
MIT-mode, torque-disable or automatic homing commands are sent.
