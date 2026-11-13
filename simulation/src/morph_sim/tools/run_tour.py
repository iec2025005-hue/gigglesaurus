#!/usr/bin/env python3
"""Protocol script for run_benchmark_case.py: one autonomous tour, policy from $MORPH_TOUR_POLICY.

    MORPH_PROTOCOL_SCRIPT=src/morph_sim/tools/run_tour.py MORPH_TOUR_POLICY=proposed \\
      python3 src/morph_sim/tools/run_benchmark_case.py NAME headless:=true rviz:=false arena:=tetra camera_enabled:=true
"""
import os
from rclpy.parameter import Parameter
import rclpy
from morph_sim import autonomous_tour


def main():
    autonomous_tour.lifecycle.initialize()
    node = autonomous_tour.AutonomousTour(); node.set_parameters([Parameter('use_sim_time', value=True), Parameter('exit_on_finish', value=True)])
    limit = float(os.environ.get('MORPH_TOUR_WALL_LIMIT_S', '1500')); import time; start = time.monotonic()
    try:
        while rclpy.ok() and not node.done_written and time.monotonic() - start < limit: rclpy.spin_once(node, timeout_sec=.05)
    except SystemExit: pass
    finally:
        if not node.done_written and node.start_time is not None: 
            try: node.finish(node.now())
            except SystemExit: pass
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
