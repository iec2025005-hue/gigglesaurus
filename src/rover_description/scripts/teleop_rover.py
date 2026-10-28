#!/usr/bin/env python3
"""
Full-Featured Teleoperation Controller for Hexagonal Telescopic Rover
Uses native ROS 2 / rclpy for smooth, continuous 20 Hz control:
  - W / S : Drive Forward / Backward
  - A / D : Steer Left / Right
  - SPACE : Full Stop / Brake
  - T / G : Extend / Retract Telescopic chassis
  - I / K : Tilt Camera Up / Down
  - J / L : Pan Camera Left / Right
"""

import sys
import tty
import termios
import select
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64
except ImportError:
    print("Error: ROS 2 rclpy environment not found.")
    print("Please run: source /opt/ros/lyrical/setup.bash")
    sys.exit(1)

HELP_MSG = """
=============================================================
  HEXAGONAL TELESCOPIC CHASSIS ROVER - TELEOPERATION
=============================================================
Driving:
    W : Drive Forward          S : Drive Backward
    A : Steer Left             D : Steer Right
    SPACE : Emergency Brake / Stop

Telescopic Chassis:
    T : Extend Chassis Outward
    G : Retract Chassis Inward

Camera Pan / Tilt:
    I : Tilt Up                K : Tilt Down
    J : Pan Left               L : Pan Right

Speed Adjust:
    + : Increase Max Speed     - : Decrease Max Speed
    Q / Ctrl+C : Quit
=============================================================
"""

class RoverTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_teleop')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.max_linear = 0.8
        self.max_angular = 1.0
        self.step_linear = 0.15
        self.step_angular = 0.25
        
        self.tele_pos = 0.0
        self.cam_pan = 0.0
        self.cam_tilt = 0.0
        
        # Continuous publishing timer at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_loop)
        
    def publish_loop(self):
        msg = Twist()
        msg.linear.x = float(self.linear_speed)
        msg.angular.z = float(self.angular_speed)
        self.cmd_vel_pub.publish(msg)
        
    def update_key(self, key):
        if key == 'w':
            self.linear_speed = min(self.max_linear, self.linear_speed + self.step_linear)
        elif key == 's':
            self.linear_speed = max(-self.max_linear, self.linear_speed - self.step_linear)
        elif key == 'a':
            self.angular_speed = min(self.max_angular, self.angular_speed + self.step_angular)
        elif key == 'd':
            self.angular_speed = max(-self.max_angular, self.angular_speed - self.step_angular)
        elif key == ' ':
            self.linear_speed = 0.0
            self.angular_speed = 0.0
        elif key in ('+', '='):
            self.max_linear = min(3.0, self.max_linear + 0.2)
            print(f"\n[Max Speed: {self.max_linear:.1f} m/s]")
        elif key in ('-', '_'):
            self.max_linear = max(0.2, self.max_linear - 0.2)
            print(f"\n[Max Speed: {self.max_linear:.1f} m/s]")
        elif key == 't':
            self.tele_pos = min(0.05, self.tele_pos + 0.01)
            print(f"\n[Chassis Extension: {self.tele_pos*1000:.0f} mm/stage]")
        elif key == 'g':
            self.tele_pos = max(0.0, self.tele_pos - 0.01)
            print(f"\n[Chassis Extension: {self.tele_pos*1000:.0f} mm/stage]")
        elif key == 'i':
            self.cam_tilt = min(0.7, self.cam_tilt + 0.1)
        elif key == 'k':
            self.cam_tilt = max(-1.4, self.cam_tilt - 0.1)
        elif key == 'j':
            self.cam_pan = min(3.14, self.cam_pan + 0.15)
        elif key == 'l':
            self.cam_pan = max(-3.14, self.cam_pan - 0.15)

def get_key(settings, timeout=0.05):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = RoverTeleopNode()
    
    # Run rclpy spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    settings = termios.tcgetattr(sys.stdin)
    print(HELP_MSG)
    
    try:
        last_display = time.time()
        while rclpy.ok():
            key = get_key(settings, 0.04)
            if key in ('q', 'Q', '\x03'):
                break
            elif key:
                node.update_key(key.lower())
            
            # Status line
            if time.time() - last_display > 0.2:
                sys.stdout.write(f"\r  Velocity: {node.linear_speed:+.2f} m/s | Turn: {node.angular_speed:+.2f} rad/s | Status: {'MOVING' if abs(node.linear_speed)>0.01 or abs(node.angular_speed)>0.01 else 'STOPPED'}   ")
                sys.stdout.flush()
                last_display = time.time()
                
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.linear_speed = 0.0
        node.angular_speed = 0.0
        node.publish_loop()
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop node terminated.")

if __name__ == '__main__':
    main()
