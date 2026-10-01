# Leader workspace

ROS 2 Jazzy workspace for a Feetech leader arm and two Mirabo follower joints.

```text
src/
  leader_hardware/feetech_driver/  Serial driver, calibration and diagnostics
  leader_control/leader_state/    Calibrated leader joint states
  leader_control/leader_controller/  Mirabo teleop and leader calculations
```

The running nodes form this chain:

```text
feetech_node -> /feetech/joint_states
leader_state -> /leader/joint_states
leader_controller teleop -> can0 -> Mirabo 0x68 / 0x69
```

One launch command starts the three nodes disarmed. Arm explicitly only after
checking fresh leader and Mirabo feedback:

```bash
ros2 launch leader_controller teleop.launch.py
ros2 param set /leader_mirabo_teleop armed true
```

To capture the current leader and both Mirabo poses as a persistent software
zero, stop teleop first and run the read-only calibration command:

```bash
ros2 run leader_controller calibrate_origin --ros-args -p port:=/dev/ttyUSB0
```

It saves `~/.config/leader_controller/origin.json` without moving motors or
changing their firmware zero. See the teleop guide for the safety checks and
how to verify the new origin before arming.

`leader_controller/leader_controller/controller.py` is retained for leader-arm
forward kinematics, center-of-mass and gravity calculations. It is independent
of the Mirabo position teleop loop.

From a new terminal, build and load only this workspace over ROS Jazzy:

```bash
cd ~/leader_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

See [the teleop guide](src/leader_control/leader_controller/README.md) for
configuration, node commands, status topics and explicit arming.

Keep `src/leader_hardware/feetech_driver/calibration/calibration.json`: it is
machine-specific data. Its editor is `tools/calibrate_keyboard.py` in the same
package.

`build/`, `install/`, `log/` and Python caches are generated output. Rebuilding
recreates the three current packages; they are not source or calibration data.
