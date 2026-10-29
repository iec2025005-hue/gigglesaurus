"""Single joint inspection command; SI units, limits checked by manager."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description='Command one controlled simulation joint')
    parser.add_argument('joint',help='ROS joint name, or shape_offset for coordinated morphing')
    parser.add_argument('value',type=float,help='radians, metres, or rad/s for a wheel')
    parser.add_argument('--duration',type=float,default=10.,help='manual override seconds (maximum 30)')
    args=parser.parse_args()
    rclpy.init();node=Node('morph_joint_control_cli')
    publisher=node.create_publisher(String,'/morph/manual_joint',10)
    msg=String(data=json.dumps(dict(joint=args.joint,value=args.value,duration=args.duration)))
    try:
        end=time.monotonic()+.5
        while time.monotonic()<end:
            publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=.05)
        print('Sent',msg.data)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
