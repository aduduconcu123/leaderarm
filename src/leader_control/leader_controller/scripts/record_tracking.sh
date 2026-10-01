#!/usr/bin/env bash
set -euo pipefail

topics=(
  /leader/joint_states
  /leader_controller/reference
  /leader_controller/filtered_reference
  /leader_controller/command
  /mirabo/joint_states
  /leader_controller/diagnostics
  /leader_controller/mirabo_status
)

stamp=$(date +%Y%m%d_%H%M%S)
out=${1:-tracking_${stamp}}
if [[ -e "$out" ]]; then
  printf 'Bag path already exists: %s\nUse a new name or omit it for a timestamped name.\n' "$out" >&2
  exit 1
fi
printf 'Recording rosbag to %s (stop with Ctrl-C)\n' "$out"
exec ros2 bag record -o "$out" --topics "${topics[@]}"
