#!/usr/bin/env bash
# Start only the selected Part 1 component in a child shell.
set -eo pipefail
PART1_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$PART1_ROOT"
source /opt/ros/humble/setup.bash
source part1_rosorin/ws/install/local_setup.bash
export ROS_LOG_DIR="$PART1_ROOT/part1_rosorin/results/runtime/ros_logs"
mkdir -p "$ROS_LOG_DIR"
case "${1:-}" in
  simulation)
    exec ros2 launch part1_rosorin/launch/room.launch.py
    ;;
  sensors)
    exec rviz2 -d part1_rosorin/config/sensors.rviz --ros-args -p use_sim_time:=true
    ;;
  teleop)
    exec ros2 run robot_gazebo teleop_key_control
    ;;
  verify)
    exec /usr/bin/python3 part1_rosorin/scripts/verify_control_sensors.py
    ;;
  slam)
    exec ros2 launch part1_rosorin/launch/slam.launch.py
    ;;
  map-view)
    exec rviz2 -d part1_rosorin/config/navigation.rviz --ros-args -p use_sim_time:=true
    ;;
  sweep)
    exec /usr/bin/python3 part1_rosorin/scripts/mapping_sweep.py
    ;;
  save-map)
    mkdir -p part1_rosorin/maps
    exec ros2 run nav2_map_server map_saver_cli -f part1_rosorin/maps/room_slam \
      --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
    ;;
  capture-pose)
    exec /usr/bin/python3 part1_rosorin/scripts/capture_pose.py
    ;;
  navigation)
    if [[ ! -f part1_rosorin/config/nav2.yaml ]]; then
      /usr/bin/python3 part1_rosorin/scripts/prepare_nav_config.py
    fi
    exec ros2 launch part1_rosorin/launch/navigation.launch.py
    ;;
  trials)
    shift
    exec /usr/bin/python3 part1_rosorin/scripts/navigation_trials.py "$@"
    ;;
  localize)
    shift
    exec /usr/bin/python3 part1_rosorin/scripts/navigation_trials.py --localize-only "$@"
    ;;
  *)
    printf 'Usage: bash part1_rosorin/scripts/run.sh {simulation|sensors|teleop|verify|slam|map-view|sweep|save-map|capture-pose|navigation|trials|localize} [trials/localize options]\n' >&2
    exit 2
    ;;
esac
