#!/usr/bin/env bash
set -euo pipefail
# Run from the workspace root (the directory holding src/ and run_gazebo.sh).
# Distro: .morph_ros_distro written by setup_ubuntu.sh, else $ROS_DISTRO, else lyrical.
DISTRO="$( [[ -f .morph_ros_distro ]] && cat .morph_ros_distro || echo "${ROS_DISTRO:-lyrical}")"
set +u; source "/opt/ros/$DISTRO/setup.bash"; set -u
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select morph_sim --event-handlers console_direct+
set +u; source install/setup.bash; set -u
colcon test --packages-select morph_sim --event-handlers console_direct+
colcon test-result --verbose
python3 src/morph_sim/tools/preflight.py
printf '\nNext: bash run_gazebo.sh headless:=true rviz:=false controls:=none teleop:=false\n'
printf 'Then, in another sourced terminal: ros2 run morph_sim smoke_test\n'
