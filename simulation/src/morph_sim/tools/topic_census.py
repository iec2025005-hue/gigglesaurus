#!/usr/bin/env python3
"""Live topic census for the V2 audit: existence, type, QoS, rate, frame, stamp age, TF tree.

Run as a run_benchmark_case.py protocol (MORPH_PROTOCOL_SCRIPT) or against a running sim.
Rates are measured against wall time over MORPH_CENSUS_SECONDS (default 8 s).
"""
import json, os, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

EXPECTED = ['/morph/vision/telemetry', '/morph/perception/decision', '/morph/status', '/morph/imu', '/morph/wheel_speed_mismatch',
            '/morph/decision/explainable', '/morph/terrain/sinkage_predicted_mm', '/morph/motion/jerk_trace', '/morph/shape/history',
            '/morph/arena/seed', '/morph/slip/derate_active', '/cmd_vel', '/joint_states', '/tf', '/tf_static', '/clock',
            '/morph/ground_truth/odom', '/morph/perception/points', '/morph/camera/image', '/morph/physics/status', '/morph/encoder/odom']


def main():
    rclpy.init(); n = Node('topic_census', parameter_overrides=[Parameter('use_sim_time', value=True)])
    duration = float(os.environ.get('MORPH_CENSUS_SECONDS', '8'))
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline and not any(name == '/morph/status' for name, _ in n.get_topic_names_and_types()):
        rclpy.spin_once(n, timeout_sec=.1)
    for _ in range(30): rclpy.spin_once(n, timeout_sec=.1)   # let discovery settle
    topics = dict(n.get_topic_names_and_types()); stats = {}; frames = {}
    def make(name):
        def cb(msg):
            s = stats[name]; s['count'] += 1
            header = getattr(msg, 'header', None)
            if header is not None:
                s['frame_id'] = header.frame_id
                stamp = header.stamp.sec + header.stamp.nanosec * 1e-9
                s['stamp_age_sim_s'] = n.get_clock().now().nanoseconds * 1e-9 - stamp
            if name in ('/tf', '/tf_static'):
                for tr in msg.transforms: frames[tr.child_frame_id] = (tr.header.frame_id, name)
        return cb
    for name, types in sorted(topics.items()):
        pubs = n.get_publishers_info_by_topic(name); subs = n.get_subscriptions_info_by_topic(name)
        stats[name] = {'type': types[0], 'publishers': sorted({p.node_name for p in pubs}), 'subscribers': sorted({s.node_name for s in subs if s.node_name != 'topic_census'}),
                       'publisher_qos': sorted({f'{p.qos_profile.reliability.name}/{p.qos_profile.durability.name}/depth{p.qos_profile.depth}' for p in pubs}), 'count': 0}
        if not pubs: continue
        transient = any(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
        if transient: qos.reliability = ReliabilityPolicy.RELIABLE
        try: n.create_subscription(get_message(types[0]), name, make(name), qos)
        except Exception as exc: stats[name]['error'] = repr(exc)
    start = time.monotonic(); sim0 = n.get_clock().now().nanoseconds * 1e-9
    while time.monotonic() - start < duration: rclpy.spin_once(n, timeout_sec=.01)
    wall = time.monotonic() - start; sim = n.get_clock().now().nanoseconds * 1e-9 - sim0
    for s in stats.values(): s['rate_hz_wall'] = round(s.pop('count') / wall, 2)
    # Discovery can lag the first snapshot (seen for /tf): report topics that appeared during the window.
    late = {name: types[0] for name, types in n.get_topic_names_and_types() if name not in topics}
    topics.update(late)
    result = {'passed': True, 'wall_s': wall, 'sim_s': sim, 'rtf_during_census': sim / wall,
              'expected_topics': {name: (name in topics) for name in EXPECTED}, 'topics': stats, 'discovered_late_not_measured': late,
              'tf_tree': {child: {'parent': parent, 'source': source} for child, (parent, source) in sorted(frames.items())},
              'nodes': sorted(f'{ns.rstrip("/")}/{name}' for name, ns in n.get_node_names_and_namespaces()),
              'use_sim_time': {}}
    Path(os.environ.get('MORPH_BENCH_OUTPUT', 'validation/performance/topic_census.json')).write_text(json.dumps(result, indent=2))
    n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
