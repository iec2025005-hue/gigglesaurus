#!/usr/bin/env python3
"""Short post-activation measurement; native warning/cleanup checks are external."""
import json,os,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from morph_sim.timing import TimedVelocityPublisher
rclpy.init();node=Node('activation_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={};samples=[]
node.create_subscription(String,'/morph/status',lambda m:state.update(json.loads(m.data)),1)
pub=TimedVelocityPublisher(node,'activation');deadline=time.monotonic()+60
try:
 while not state.get('feedback_ready') and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
 if not state.get('feedback_ready'):raise RuntimeError('No complete fresh feedback')
 start=node.get_clock().now().nanoseconds;wall=time.monotonic();next_pub=0.
 while (node.get_clock().now().nanoseconds-start)*1e-9<3:
  if time.monotonic()-wall>30:raise RuntimeError('Clock did not advance')
  if time.monotonic()>=next_pub:pub.publish(Twist());next_pub=time.monotonic()+.02;samples.append({'wall_ns':time.monotonic_ns(),'sim_ns':node.get_clock().now().nanoseconds,'feedback_ready':state.get('feedback_ready'),'backend':state.get('position_backend')})
  rclpy.spin_once(node,timeout_sec=.003)
 report={'scope':'Short startup/activation feedback check; native limits and clean exit assessed in runner cleanup','samples':samples,'passed':bool(samples) and all(x['feedback_ready'] for x in samples),'position_backend':state.get('position_backend')}
 Path(os.environ['MORPH_BENCH_OUTPUT']).write_text(json.dumps(report,indent=2))
 if not report['passed']:raise SystemExit(1)
finally:
 node.destroy_node()
 if rclpy.ok():rclpy.shutdown()
