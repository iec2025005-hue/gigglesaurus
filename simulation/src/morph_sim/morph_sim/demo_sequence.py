"""Bounded, operator-supervised presentation sequence on the starting road."""
import json
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool


class Demo(Node):
    def __init__(self):
        super().__init__('morph_demo_sequence')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.events = self.create_publisher(String, '/morph/event', 10)
        self.camera = self.create_publisher(Bool, '/morph/camera/show', 10)
        self.status = {}
        self.create_subscription(String, '/morph/status', self.receive, 10)

    def receive(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (ValueError, TypeError):
            pass

    def send(self, vx=0., vy=0., yaw=0.):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, yaw
        self.cmd.publish(msg)

    def wait(self, seconds, vx=0., vy=0., yaw=0.):
        end = self.get_clock().now().nanoseconds*1e-9 + seconds
        while rclpy.ok() and self.get_clock().now().nanoseconds*1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.04)
            if self.status.get('estop'):
                raise RuntimeError('Emergency stop latched; demo stopped for inspection')
            self.send(vx, vy, yaw)
        self.send()

    def mode(self, value, target=None, timeout=70.):
        self.events.publish(String(data=value))
        if target is None:
            return
        deadline = self.get_clock().now().nanoseconds*1e-9 + timeout
        while rclpy.ok() and self.get_clock().now().nanoseconds*1e-9 < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.send()
            if self.status.get('estop'):
                raise RuntimeError('Emergency stop latched during shape change')
            measured = self.status.get('stage_offset')
            if self.status.get('feedback_ready') and isinstance(measured, (float, int)) and abs(measured-target) < .0005:
                return
        raise RuntimeError('Shape target did not converge; check /morph/status and controllers')


def main():
    rclpy.init()
    node = Demo()
    try:
        node.get_logger().info('Waiting for feedback before presentation sequence')
        deadline = time.monotonic() + 60.
        while rclpy.ok() and not node.status.get('feedback_ready') and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.send()
        if not node.status.get('feedback_ready'):
            raise RuntimeError('No controlled joint feedback; run diagnostic before demo')
        node.camera.publish(Bool(data=True))
        node.mode('expand', .016511111111)
        node.wait(2.)
        node.mode('normal', 0.)
        node.mode('compact', -.0379)
        node.wait(1.)
        node.mode('normal', 0.)
        node.mode('track')
        node.wait(3., vx=.12)
        node.wait(1., yaw=.2)
        node.camera.publish(Bool(data=False))
        node.wait(1.)
        node.camera.publish(Bool(data=True))
        node.wait(1., vy=.08)
        node.get_logger().info('Presentation sequence complete; drive remains stopped')
    except (RuntimeError, KeyboardInterrupt) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.send()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
