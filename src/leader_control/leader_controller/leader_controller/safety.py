"""Structured states and fault records for the Mirabo teleop controller."""

from dataclasses import dataclass
from enum import Enum


class TeleopState(str, Enum):
    INIT = 'INIT'
    WAIT_LEADER = 'WAIT_LEADER'
    WAIT_CAN = 'WAIT_CAN'
    READY = 'READY'
    ARMED = 'ARMED'
    FAULT = 'FAULT'
    ESTOP = 'ESTOP'


@dataclass(frozen=True)
class FaultRecord:
    code: str
    reason: str
    stamp_sec: float
