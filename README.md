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
package. `feetech_driver/test_motor.py` provides read-only diagnostics for all
three leader motors; the package's `test/` also retains low-level Feetech checks.

Hardware-free regression tests:

```bash
colcon test --packages-select leader_controller --pytest-args test/test_teleop.py
colcon test --packages-select feetech_driver --pytest-args test/test_calibration_tool.py
```

`build/`, `install/`, `log/` and Python caches are generated output. Rebuilding
recreates the three current packages; they are not source or calibration data.
