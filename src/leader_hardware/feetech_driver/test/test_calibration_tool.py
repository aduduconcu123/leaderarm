"""Hardware-free regression tests for calibration and session behavior."""
import importlib.util
import io
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from feetech_driver.calibration import CalibrationManager, MotorCalibration

spec = importlib.util.spec_from_file_location(
    "calibrate_keyboard",
    Path(__file__).resolve().parents[1] / "tools" / "calibrate_keyboard.py",
)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def test_wrap_and_zero():
    for direction in (-1, 1):
        calibration = MotorCalibration("J1", zero_raw=4090, direction=direction)
        assert calibration.raw_to_angle(6) == pytest.approx(direction * 12 * 360 / 4096)
        assert calibration.angle_to_raw(calibration.raw_to_angle(6)) == 6
        assert calibration.raw_to_angle(4080) == pytest.approx(-direction * 10 * 360 / 4096)
    assert MotorCalibration("J1", zero_raw=0).raw_to_angle(1) > 0


def test_limits_across_seam_and_incomplete():
    data = dict(zero_raw=4090, limit_min=4000, limit_max=50, direction=1)
    with pytest.raises(ValueError, match="Capture first"):
        tool.validate(data, set())
    assert tool.validate(data, set(tool.LANDMARKS.values())) == (-90, 56)
    data["limit_max"] = 3900
    with pytest.raises(ValueError):
        tool.validate(data, set(tool.LANDMARKS.values()))


def test_fresh_capture_and_save(tmp_path):
    manager = CalibrationManager(tmp_path / "calibration.json")
    motors = {i: Mock() for i in (1, 3, 2)}
    for motor in motors.values():
        motor.read_raw_position.return_value = 100
        motor.is_torque_enabled.return_value = True
    actions = iter(["s", "z", "l", "h", "n"] * 3 + ["s", "q"])

    def key():
        action = next(actions)
        # Simulate movement while waiting for input, AFTER screen sampling.
        if action in "zlh":
            for motor in motors.values():
                motor.read_raw_position.return_value = {"z": 4090, "l": 4000, "h": 50}[action]
        return action

    with patch.object(tool, "get_key", side_effect=key), patch("sys.stdout", new_callable=io.StringIO):
        tool.run_session(motors, manager)
    loaded = CalibrationManager(manager.calibration_file)
    assert loaded.load()
    assert {mid: c.joint for mid, c in loaded.motors.items()} == {1: "J1", 3: "J2", 2: "J3"}
    for calibration in loaded.motors.values():
        assert (calibration.zero_raw, calibration.limit_min, calibration.limit_max) == (4090, 4000, 50)
    for motor in motors.values():
        motor.enable_torque.assert_not_called()
        motor.disable_torque.assert_not_called()
        motor.set_position.assert_not_called()


def test_no_save_defaults_or_torque_on_exit(tmp_path):
    manager = CalibrationManager(tmp_path / "calibration.json")
    motors = {i: Mock() for i in (1, 3, 2)}
    for motor in motors.values():
        motor.read_raw_position.return_value = 0
    with patch.object(tool, "get_key", side_effect=["s", "n", "q"]), patch("sys.stdout", new_callable=io.StringIO):
        tool.run_session(motors, manager)
    assert not manager.calibration_file.exists()
    for motor in motors.values():
        motor.disable_torque.assert_not_called()


def test_one_connection_closed_on_startup_failure():
    protocol = Mock()
    first, second = Mock(), Mock()
    second.ping.side_effect = RuntimeError("No response")
    with patch.object(tool, "FeetechProtocol", return_value=protocol) as factory, patch.object(tool, "STS3250", side_effect=[first, second]) as motors, patch.object(tool, "CalibrationManager"), patch("sys.stdout", new_callable=io.StringIO):
        tool.main()
    factory.assert_called_once()
    assert [call.kwargs for call in motors.call_args_list] == [
        {"motor_id": 1, "protocol": protocol},
        {"motor_id": 3, "protocol": protocol},
    ]
    protocol.close.assert_called_once()
    first.disable_torque.assert_not_called()
    second.disable_torque.assert_not_called()
