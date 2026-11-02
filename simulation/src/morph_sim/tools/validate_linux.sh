#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$WS"
DISTRO="${ROS_DISTRO:-}"
if [[ -z "$DISTRO" && -f .morph_ros_distro ]]; then DISTRO="$(<.morph_ros_distro)"; fi
case "$DISTRO" in
 humble|jazzy|lyrical) ;;
 *) echo 'Source ROS first, or run setup_ubuntu.sh to record the workspace distro.' >&2; exit 1;;
esac
if [[ ! -f "/opt/ros/$DISTRO/setup.bash" ]]; then echo "ROS $DISTRO is not installed." >&2; exit 1; fi
set +u
source "/opt/ros/$DISTRO/setup.bash"
set -u
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to morph_sim --event-handlers console_direct+
set +u
source install/setup.bash
set -u
colcon test --packages-select morph_sim --event-handlers console_direct+
colcon test-result --verbose
python3 src/morph_sim/tools/preflight.py
printf '\nNext: bash run_gazebo.sh headless:=true rviz:=false teleop:=false\n'
printf 'Then, in another sourced terminal: ros2 run morph_sim smoke_test\n'
