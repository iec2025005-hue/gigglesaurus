#!/usr/bin/env bash
set -euo pipefail
# Run from the root of the extracted morph_sim_ws workspace.
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
colcon test --packages-select morph_sim --event-handlers console_direct+
colcon test-result --verbose
python3 src/morph_sim/tools/preflight.py
printf '\nNext: ros2 launch morph_sim simulation.launch.py headless:=true teleop:=false\n'
printf 'Then, in another sourced terminal: ros2 run morph_sim smoke_test\n'
