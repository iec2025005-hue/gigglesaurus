#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
set -u
source "$(dirname -- "${BASH_SOURCE[0]}")/transport_env.sh"
export GZ_IP="${GZ_IP:-127.0.0.1}"
python3 "$WS/src/morph_sim/tools/course_view.py" "${1:-overview}"
