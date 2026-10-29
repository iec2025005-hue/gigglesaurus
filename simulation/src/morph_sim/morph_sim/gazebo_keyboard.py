"""Drive from Gazebo's Key Publisher using the laptop keyboard."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool


KEY_EVENTS = {
    ord('1'): 'track', ord('2'): 'rocky', ord('3'): 'climb', ord('4'): 'sand',
    ord('X'): 'compact', ord('N'): 'normal', ord('E'): 'expand',
    ord('U'): 'reset', ord('K'): 'estop', ord('F'): 'flip',
    ord('H'): 'raise', ord('L'): 'lower', ord('V'): 'perception_toggle',
}
KEY_VELOCITIES = {
    ord('W'): (0.30, 0.0, 0.0), 16777235: (0.30, 0.0, 0.0),
    ord('S'): (-0.30, 0.0, 0.0), 16777237: (-0.30, 0.0, 0.0),
    ord('A'): (0.0, 0.0, 0.50), 16777234: (0.0, 0.0, 0.50),
    ord('D'): (0.0, 0.0, -0.50), 16777236: (0.0, 0.0, -0.50),
    ord('Q'): (0.0, 0.24, 0.0), ord('C'): (0.0, -0.24, 0.0),
    ord('B'): (0.0, 0.0, 0.0),
}


class GazeboKeyboard(Node):
    def __init__(self):
        super().__init__('morph_gazebo_keyboard')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.event_pub = self.create_publisher(String, '/morph/event', 10)
        self.camera_pub = self.create_publisher(Bool, '/morph/camera/show', 10)
        self.camera_enabled = True
        self.create_subscription(Int32, '/keyboard/keypress', self.on_key, 10)
        self.last_key = 0.0
        self.velocity = (0.0, 0.0, 0.0)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Gazebo keyboard active: click Gazebo, W/S/A/D or arrows, Q/C, X/N/E, 1-4, B brake, K stop, U reset.')

    def on_key(self, message):
        key = message.data
        if 97 <= key <= 122:
            key -= 32
        event = KEY_EVENTS.get(key)
        if key == ord('P'):
            self.camera_enabled = not self.camera_enabled
            self.camera_pub.publish(Bool(data=self.camera_enabled))
        if event:
            self.event_pub.publish(String(data=event))
        if key in KEY_VELOCITIES:
            self.velocity = KEY_VELOCITIES[key]
            self.last_key = time.monotonic()
        elif key == ord('K'):
            self.velocity = (0.0, 0.0, 0.0)
            self.last_key = 0.0

    def tick(self):
        active = time.monotonic() - self.last_key < 0.35
        vx, vy, wz = self.velocity if active else (0.0, 0.0, 0.0)
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = GazeboKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
