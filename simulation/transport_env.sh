#!/usr/bin/env bash
# Dedicated discovery partition: shutdown cannot target another project's server.
MORPH_TRANSPORT_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export GZ_PARTITION="${MORPH_GZ_PARTITION:-morph_$(python3 -c 'import hashlib,sys;print(hashlib.sha256(sys.argv[1].encode()).hexdigest()[:12])' "$MORPH_TRANSPORT_WS")}"
# ROS 2 discovery stays on this machine by default. Another team's ROS graph on the same network
# (default domain) injected its /clock into a run and could receive this simulation's /cmd_vel.
# Override with ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET to reach the simulation from another device.
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
