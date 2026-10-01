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


CAN_INTERFACE = 'can0'
CAN_BITRATE = 1000000
# Mirabo's C++ CANInterface::transmit sets CAN_EFF_FLAG even for IDs < 0x800.
COMMAND_IS_EXTENDED_ID = True

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
        sign=-1.0,  # Direction confirmed by the user's J2 test.
    ),
    MiraboJointMapping(
        leader_joint='joint_3',
        motor_id=0x69,
        command_id=0x669,
        feedback_id=0x2969,
        sign=-1.0,  # Direction confirmed by the user's J3 test.
    ),
)


def target_can_degrees(item, leader_rad, baseline_leader_deg,
                       baseline_motor_deg):
    """Map a leader joint delta onto a motor's captured CAN angle."""
    from math import degrees

    scale = GEAR_RATIO if CAN_ANGLE_IS_MOTOR_SHAFT else 1.0
    return (baseline_motor_deg
            + item.sign * scale * (degrees(leader_rad) - baseline_leader_deg))
