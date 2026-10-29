#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
DISTRO="$(<"$WS/.morph_ros_distro")"
source "/opt/ros/$DISTRO/setup.bash"
source "$WS/install/setup.bash"
set -u
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export ROS_HOME="${ROS_HOME:-$WS/log/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/log/runtime}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
exec ros2 run rviz2 rviz2 -d "$WS/src/morph_sim/config/morph.rviz" --ros-args -p use_sim_time:=true
