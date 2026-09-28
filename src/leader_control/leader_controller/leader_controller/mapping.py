"""Mirabo mapping used by leader_controller.teleop."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MiraboJointMapping:
    """Mapping from calibrated leader joint to one Mirabo CAN motor."""

    leader_joint: str
    motor_id: int
    command_id: int
    feedback_id: int
    sign: float
    min_deg: float
    max_deg: float


CAN_INTERFACE = 'can0'
CAN_BITRATE = 1000000

CONTROL_RATE_HZ = 20.0
LEADER_TIMEOUT_SEC = 0.5
FEEDBACK_TIMEOUT_SEC = 0.5
MAX_STEP_DEG_PER_CYCLE = 2.0
MAX_CAN_FRAMES_PER_READ = 256

COMMAND_SPEED = 1000
COMMAND_ACCEL = 1000

GEAR_RATIO = 8.0

# False means the Mirabo firmware CAN angle is treated as the output joint
# angle. Set true only if testing confirms CAN angle is motor-shaft angle.
CAN_ANGLE_IS_MOTOR_SHAFT = False

MIRABO_JOINTS = (
    MiraboJointMapping(
        leader_joint='joint_2',
        motor_id=0x68,
        command_id=0x668,
        feedback_id=0x2968,
        sign=1.0,  # Existing configuration; J2 mechanics not yet confirmed.
        # CAN-angle software limits from ControlMotor.cpp, not mechanical limits.
        min_deg=-15.0,
        max_deg=90.0,
    ),
    MiraboJointMapping(
        leader_joint='joint_3',
        motor_id=0x69,
        command_id=0x669,
        feedback_id=0x2969,
        sign=-1.0,  # Direction confirmed by the user's J3 test.
        min_deg=0.0,
        max_deg=90.0,
    ),
)
