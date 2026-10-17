#!/usr/bin/env bash
# Check installed runtime packages without installing or changing system files.
set -eo pipefail
PART1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$PART1_ROOT/part1_rosorin/ws/install/local_setup.bash" ]]; then
  source "$PART1_ROOT/part1_rosorin/ws/install/local_setup.bash"
fi
missing=0
for package in robot_gazebo rosorin_description ros_ign_gazebo ros_gz_sim \
  ros_gz_bridge ign_ros2_control controller_manager joint_state_publisher \
  robot_state_publisher rviz2 xacro; do
  if prefix="$(ros2 pkg prefix "$package" 2>/dev/null)"; then
    printf 'FOUND %s: %s\n' "$package" "$prefix"
  else
    printf 'MISSING %s\n' "$package"
    missing=$((missing + 1))
  fi
done
if ((missing > 0)); then
  printf 'Runtime prerequisites missing: %d\n' "$missing"
  exit 2
fi
printf 'Package lookup passed; simulator runtime still needs verification.\n'
