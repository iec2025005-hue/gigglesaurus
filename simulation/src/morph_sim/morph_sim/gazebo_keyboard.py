"""Drive from Gazebo's Key Publisher using the laptop keyboard."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String, Bool


KEY_EVENTS = {
    ord('1'): 'track', ord('2'): 'rocky', ord('3'): 'climb', ord('4'): 'sand',
    ord('X'): 'compact', ord('N'): 'normal', ord('Z'): 'expand',
    ord('U'): 'reset', ord('K'): 'estop', ord('F'): 'flip',
    ord('H'): 'raise', ord('L'): 'lower', ord('V'): 'perception_toggle',
}
KEY_VELOCITIES = {
    ord('W'): (0.30, 0.0, 0.0), 16777235: (0.30, 0.0, 0.0),
    ord('S'): (-0.30, 0.0, 0.0), 16777237: (-0.30, 0.0, 0.0),
    # Holonomic swerve, not a car: A/D strafe sideways with no heading change, Q/E (and Left/Right) rotate in place.
    ord('A'): (0.0, 0.24, 0.0), ord('D'): (0.0, -0.24, 0.0),
    ord('Q'): (0.0, 0.0, 0.50), 16777234: (0.0, 0.0, 0.50),
    ord('E'): (0.0, 0.0, -0.50), 16777236: (0.0, 0.0, -0.50),
    ord('B'): (0.0, 0.0, 0.0),
}
HOLD_S = 0.35


class GazeboKeyboard(Node):
    def __init__(self):
        super().__init__('morph_gazebo_keyboard')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.event_pub = self.create_publisher(String, '/morph/event', 10)
        self.camera_pub = self.create_publisher(Bool, '/morph/camera/show', 10)
        self.camera_enabled = True
        self.create_subscription(Int32, '/keyboard/keypress', self.on_key, 10)
        # One value and one time stamp per axis (vx, vy, wz), so keys of different axes combine. Gazebo only
        # reports key presses (no releases, and the OS repeats only the last key held), so a combination lasts
        # HOLD_S after the earlier key's last event. The control console (controls:=combined) sees real key state.
        self.axis_value = [0.0, 0.0, 0.0]
        self.axis_time = [0.0, 0.0, 0.0]
        self.create_timer(0.05, self.tick)
        self.get_logger().info('Gazebo keyboard active: click Gazebo. W/S drive, A/D strafe, Q/E or Left/Right rotate, Z/X/N shape, 1-4 mode, B brake, K stop, U reset.')

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
        if key in (ord('B'), ord('K')):
            self.axis_time = [0.0, 0.0, 0.0]
        elif key in KEY_VELOCITIES:
            now = time.monotonic()
            for axis, value in enumerate(KEY_VELOCITIES[key]):
                if value:
                    self.axis_value[axis] = value
                    self.axis_time[axis] = now

    def tick(self):
        now = time.monotonic()
        vx, vy, wz = (value if now - stamp < HOLD_S else 0.0 for value, stamp in zip(self.axis_value, self.axis_time))
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
