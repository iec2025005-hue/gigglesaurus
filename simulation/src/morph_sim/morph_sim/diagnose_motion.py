"""Read-only runtime report for keyboard, controller feedback and vehicle motion."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, String


class Diagnose(Node):
    def __init__(self):
        super().__init__('morph_diagnose_motion')
        self.keys = 0
        self.states = {}
        self.status = {}
        self.speed = None
        self.create_subscription(Int32, '/keyboard/keypress', self.key, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_subscription(String, '/morph/status', self.report, 10)
        self.create_subscription(Odometry, '/morph/ground_truth/odom', self.odom, qos_profile_sensor_data)

    def key(self, _):
        self.keys += 1

    def joints(self, msg):
        self.states = {name: (msg.position[i] if i < len(msg.position) else None,
                              msg.velocity[i] if i < len(msg.velocity) else None)
                       for i, name in enumerate(msg.name)}

    def report(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def odom(self, msg):
        v = msg.twist.twist.linear
        self.speed = (v.x*v.x + v.y*v.y + v.z*v.z)**.5


def main():
    rclpy.init()
    node = Diagnose()
    try:
        end = time.monotonic() + 6.
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
        wheels = {name: data for name, data in node.states.items() if name.endswith('_wheel_joint')}
        print(json.dumps({
            'keyboard_events_seen': node.keys,
            'joint_count': len(node.states),
            'wheel_states': wheels,
            'controller_feedback_ready': node.status.get('feedback_ready'),
            'emergency_stop': node.status.get('estop'),
            'command_speed_m_per_s': node.status.get('command_speed'),
            'ground_truth_speed_m_per_s': node.speed,
            'stage_offset_m': node.status.get('stage_offset'),
            'terrain_zone': node.status.get('terrain_zone'),
            'automatic_terrain': node.status.get('automatic_terrain'),
            'transition_pending': node.status.get('transition_pending'),
            'hint': ('Click the Gazebo 3D scene and press W repeatedly while this diagnosis runs. '
                     'Zero keyboard events means Gazebo Key Publisher/bridge is absent. '
                     'Missing wheel states or feedback means controllers or the Gazebo plugin failed. '
                     'A latched emergency stop needs cause inspection and U reset.'),
        }, indent=2))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
