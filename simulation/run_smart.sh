#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$WS/run_realtime.sh" rviz:=true range_sensor:=true perception_auto:=true "$@"
