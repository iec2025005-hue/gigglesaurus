#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"
source "$WS/transport_env.sh"
export MORPH_SUPERVISED_GZ=1
if [[ ! -f .morph_ros_distro || ! -f install/setup.bash ]]; then echo 'Run bash setup_ubuntu.sh first.'; exit 1; fi
DISTRO="$(<.morph_ros_distro)"
set +u
source "/opt/ros/$DISTRO/setup.bash"
source "$WS/install/setup.bash"
set -u
if [[ "$DISTRO" != lyrical ]]; then export GZ_VERSION=harmonic; fi
# Local desktop transport avoids stale shared-memory discovery across sessions.
export GZ_IP="${GZ_IP:-127.0.0.1}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export GZ_LOG_PATH="${GZ_LOG_PATH:-$WS/log/gazebo}"
mkdir -p "$GZ_LOG_PATH"
echo "MORPH: connected arena is the default. Workspace: $WS"
export ROS_HOME="${ROS_HOME:-$WS/log/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$WS/log/runtime}"
mkdir -p "$ROS_HOME" "$ROS_LOG_DIR"
# The supervisor owns the lock; child GUIs cannot inherit it. It also forwards
# terminal interruptions to the full launch group and waits for bounded cleanup.
exec python3 "$WS/src/morph_sim/tools/launch_supervisor.py" "$WS/log/simulation-launch.lock" ros2 launch morph_sim simulation.launch.py "$@"
