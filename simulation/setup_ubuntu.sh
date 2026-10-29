#!/usr/bin/env bash
set -Eeuo pipefail
trap 'printf "\nSetup stopped at line %s. See setup.log in the workspace.\n" "$LINENO" >&2' ERR
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"
exec > >(tee -a "$WS/setup.log") 2>&1
if [[ $EUID == 0 ]]; then echo 'Run as your normal Ubuntu user; the script requests sudo when needed.'; exit 1; fi
source /etc/os-release
case "$ID:$VERSION_ID" in
 ubuntu:22.04) DISTRO=humble;;
 ubuntu:24.04) DISTRO=jazzy;;
 ubuntu:26.04) DISTRO=lyrical;;
 *) echo 'Supported systems: Ubuntu 22.04, 24.04, or 26.04.'; exit 1;;
esac
chmod +x src/morph_sim/nodes/*
if [[ "$DISTRO" == lyrical ]]; then
 set +u
 source /opt/ros/lyrical/setup.bash
 set -u
 colcon build --symlink-install --packages-select morph_sim --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
 set +u
 source install/setup.bash
 set -u
 python3 src/morph_sim/tools/preflight.py
 colcon test --packages-select morph_sim --event-handlers console_direct+
 colcon test-result --verbose
 printf '%s\n' "$DISTRO" > .morph_ros_distro
 printf '\nSetup complete. Start with: bash run_gazebo.sh\n'
 exit 0
fi
if [[ "$WS" == /mnt/* || "$WS" == /media/* ]]; then
 echo 'Extract this folder into your Ubuntu home folder before setup.'; exit 1
fi
if dpkg-query -W -f='${Status}' gazebo11 2>/dev/null | grep -q 'install ok installed'; then
 echo 'Gazebo Classic 11 is installed and conflicts with Harmonic. Resolve that conflict before setup; nothing was removed.'; exit 1
fi
sudo apt-get update
sudo apt-get install -y curl ca-certificates gnupg software-properties-common locales git build-essential cmake
sudo add-apt-repository -y universe
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo tee /usr/share/keyrings/ros-archive-keyring.gpg >/dev/null
printf 'deb [arch=%s signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.ros.org/ros2/ubuntu %s main\n' "$(dpkg --print-architecture)" "$VERSION_CODENAME" | sudo tee /etc/apt/sources.list.d/morph-ros2.list >/dev/null
curl -fsSL https://packages.osrfoundation.org/gazebo.gpg | sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable %s main\n' "$(dpkg --print-architecture)" "$VERSION_CODENAME" | sudo tee /etc/apt/sources.list.d/morph-gazebo.list >/dev/null
sudo apt-get update
sudo apt-get install -y "ros-$DISTRO-ros-base" "ros-$DISTRO-ros2-control" "ros-$DISTRO-ros2-controllers" "ros-$DISTRO-robot-state-publisher" "ros-$DISTRO-xacro" "ros-$DISTRO-rviz2" "ros-$DISTRO-teleop-twist-keyboard" "ros-$DISTRO-ament-cmake-python" "ros-$DISTRO-ament-cmake-pytest" python3-colcon-common-extensions python3-pytest python3-pygame python3-tk python3-yaml python3-numpy python3-pil xterm gz-harmonic libgz-sim8-dev libgz-transport13-dev libgz-msgs10-dev libgz-plugin2-dev
if [[ $DISTRO == humble ]]; then
 sudo apt-get install -y ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-sim ros-humble-yaml-cpp-vendor
 if [[ ! -d src/gz_ros2_control ]]; then git clone --depth 1 --branch humble https://github.com/ros-controls/gz_ros2_control.git src/gz_ros2_control; fi
 git -C src/gz_ros2_control rev-parse HEAD > control_plugin_revision.txt
else
 sudo apt-get install -y ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-gz-sim-vendor ros-jazzy-gz-transport-vendor ros-jazzy-gz-msgs-vendor ros-jazzy-gz-plugin-vendor
fi
# ROS setup files may reference unset variables.
set +u
source "/opt/ros/$DISTRO/setup.bash"
set -u
export GZ_VERSION=harmonic
colcon build --symlink-install --packages-up-to morph_sim --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
set +u
source install/setup.bash
set -u
python3 src/morph_sim/tools/preflight.py
colcon test --packages-select morph_sim --event-handlers console_direct+
colcon test-result --verbose
printf '%s\n' "$DISTRO" > .morph_ros_distro
printf '\nSetup complete. Start the demonstration with: bash run_gazebo.sh\n'
