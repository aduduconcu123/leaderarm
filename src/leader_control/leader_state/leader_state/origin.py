"""Persistent software zero for the leader and Mirabo feedback."""

import json
import math
import os
from pathlib import Path
import tempfile


LEADER_JOINTS = ('joint_1', 'joint_2', 'joint_3')
MIRABO_IDS = ('0x68', '0x69')


def default_origin_file():
    """Return the machine-local path for the software origin."""
    return Path.home() / '.config' / 'leader_controller' / 'origin.json'


def validate_origin(data):
    """Reject incomplete or non-finite origin data."""
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('origin file must have version 1')
    for field, expected in (
        ('leader_radians', LEADER_JOINTS),
        ('mirabo_can_degrees', MIRABO_IDS),
    ):
        values = data.get(field)
        if not isinstance(values, dict) or set(values) != set(expected):
            raise ValueError(f'{field} must contain {list(expected)}')
        for name, value in values.items():
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value)):
                raise ValueError(f'{field}[{name}] must be finite')
    return data


def load_origin(path):
    """Load an origin or return None when calibration has not run."""
    path = Path(path).expanduser()
    if not path.exists():
        return None
    with path.open(encoding='utf-8') as stream:
        return validate_origin(json.load(stream))


def save_origin(path, data):
    """Atomically replace the saved software origin."""
    path = Path(path).expanduser()
    validate_origin(data)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=path.parent,
            prefix='.origin-', suffix='.json', delete=False,
        ) as stream:
            temp_path = Path(stream.name)
            json.dump(data, stream, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_path, path)
    finally:
        if temp_path is not None and temp_path.exists():
            temp_path.unlink()
