#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Keep physics, live sensors, RViz and the combined console; omit duplicate 3D rendering.
exec bash "$WS/run_smart.sh" gazebo_gui:=false "$@"
