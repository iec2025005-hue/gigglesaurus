#!/usr/bin/env bash
set -Eeuo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Bound software-rendering workers; hardware drivers ignore this Mesa setting.
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
exec bash "$WS/run_arena.sh" camera_rate:=5 camera_width:=320 camera_effects:=false render_shadows:=false "$@"
