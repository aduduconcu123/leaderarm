# Mirabo command field audit (Phase 3 preparation)

No hardware experiment was performed for this audit.

## Source and wire format

The live command path is `MiraboTeleop.send_position_command()` in
`leader_controller/teleop.py` -> `MiraboCanDriver.send_position()` ->
`pack_position_command()` in `leader_controller/mirabo_can.py`.
`mapping.py` fixes command IDs 0x668 (motor 0x68) and 0x669 (motor 0x69),
both transmitted with `is_extended_id=True`; feedback IDs remain 0x2968
and 0x2969. The archived C++ implementation at
`~/Downloads/Version1 (copy).zip`,
`26-5-Control/src/can_interface/src/caninterface.cpp:setPositionSpeed()`,
also writes an int32 position followed by two int16 fields and sets
`CAN_EFF_FLAG` in `transmit()`.

The Python payload is exactly `struct.pack('>ihh', position_raw,
speed_raw, acceleration_raw)` (8 bytes). `>` means big-endian; `i` is
signed 32-bit and each `h` is signed 16-bit.

| Bytes | Field | Input Python type | Encoding | Current default |
| --- | --- | --- | --- | ---: |
| 0-3 | position | finite numeric CAN degrees | `int(float32(position) * 10000)` | dynamic |
| 4-5 | field named speed | C++ int16, ROS config positive integer | `int(speed / 10.0)` | 1000 -> raw 100 |
| 6-7 | field named acceleration | C++ int16, ROS config positive integer | `int(acceleration / 10.0)` | 1000 -> raw 100 |

`int()` truncates toward zero. There is **no clamping** in the packer:
`struct.pack` raises if a raw value is outside the signed field range.
The former runtime constants `COMMAND_SPEED=1000` and
`COMMAND_ACCEL=1000` remain in `mapping.py` for reference and inspection
tool defaults; runtime teleop now reads `mirabo_command_speed` and
`mirabo_command_acceleration` from ROS parameters. Both default to 1000
in `config/teleop.yaml`, preserving the exact old bytes. At startup the
node rejects non-integer, non-positive, out-of-positive-int16-source-range,
or out-of-raw-range profile fields before opening CAN. This validation
does not alter the position format, speed/acceleration scaling, or normal
default payload.

For example, position 10.0 CAN degrees and both defaults encode as
`00 01 86 A0 00 64 00 64` (`000186a000640064`). The fields named speed
and acceleration are *not* the software limiter's `v_cmd` and `amax`.
Phase 2 limiter settings, CAN IDs, feedback decoder, arming and fault
behavior remain unchanged.

## Known and unknown

Known: packet order, widths, signedness, endianness, current defaults,
raw scaling, and C++ transmitter equivalence. The archived source names
the fields `speed` and `RPA`; that is not a verified physical-unit
specification for this motor/firmware.

**UNKNOWN:** whether the speed field is a motor velocity limit, target
velocity, an RPM-like quantity, or another firmware parameter. The
acceleration field's unit, exact meaning, zero/negative behavior,
internal clamping, and whether either field explains the measured
`q_cmd -> q_actual` lag are also **UNKNOWN**. Do not substitute these
fields for a validated physical safety limit.

## Controlled candidates

The existing default is S0=A0=1000 (raw 100). Candidate low/high values
900 and 1100 encode as 90 and 110, a 10% step around the only observed
working value. This is a modest experimental perturbation, not a claim of
safety. Six explicit YAML profiles are in `config/experiments/`:

| Profile | Speed field | Acceleration field | Difference from baseline |
| --- | ---: | ---: | --- |
| `phase3_speed_baseline.yaml` | 1000 | 1000 | none |
| `phase3_speed_low.yaml` | 900 | 1000 | speed only |
| `phase3_speed_high.yaml` | 1100 | 1000 | speed only |
| `phase3_accel_baseline.yaml` | 1000 | 1000 | none |
| `phase3_accel_low.yaml` | 1000 | 900 | acceleration only |
| `phase3_accel_high.yaml` | 1000 | 1100 | acceleration only |

All profiles retain Phase 2 control rate 20 Hz, max software velocity
5 deg/s, max software acceleration 25 deg/s², max step 0.5 deg and filter
alpha 1. The existing `phase2_20hz_5dps.yaml` was not changed.

Startup logs print `MIRABO COMMAND PROFILE` with configured and raw values.
`/leader_controller/diagnostics` records four corresponding fields for
future bags. `tools/inspect_mirabo_command_frame.py` prints payload bytes
offline without opening CAN. `tools/analyze_tracking.py` carries recorded
profile values into `summary.json` and reports conservative overshoot and
settling estimates only when a clear command move between stable holds
exists; otherwise those metrics are null. `tools/compare_mirabo_profiles.py`
prints per-motor tables and CSV without choosing a winner.

The Phase 2 bag `phase2_20hz_5dps_20261001_172601` shows command peak
5 deg/s and actual control rate about 20 Hz, but was recorded before
these profile diagnostics were added. Its command field values cannot
be certified from that bag alone; `command_profile.status=UNAVAILABLE`
is the correct analyzer result for older recordings.

Offline frame inspection example (from `~/leader_ws` after sourcing ROS
and the workspace):

```bash
python3 src/leader_control/leader_controller/tools/inspect_mirabo_command_frame.py --position-deg 10 --speed 900 --acceleration 1000
```
