#!/usr/bin/env python3
"""
Advanced Multi-Mode 4WIS/4WID Swerve Teleoperation Controller
============================================================
Supports 4 specialized swerve modes with native ROS 2 topics:
  - Mode 1 (STANDARD / ACKERMANN): W/S drive forward/back, A/D steer
  - Mode 2 (CRAB WALK / STRAFE):  W/S drive forward/back, A/D strafe sideways (no rotation)
  - Mode 3 (ZERO-TURN SPIN):     A/D spin in place at center of mass
  - Mode 4 (X-STANCE BRAKE):     Mechanical 45-deg wheel locking on steep slopes
  - T / G : Extend / Retract telescopic chassis (0-300mm per side)
  - I / K & J / L : 2-DOF Camera Gimbal Tilt & Pan
  - SPACE : Instant Brake / Stop
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
    from std_msgs.msg import Float64, String
except ImportError:
    print("Error: ROS 2 rclpy environment not found.")
    print("Please run: source /opt/ros/lyrical/setup.bash")
    sys.exit(1)

HELP_MSG = """
=============================================================================
     4WIS/4WID HEXAGONAL ROVER - MULTI-MODE SWERVE CONTROLLER
=============================================================================
Modes:
    1 : Standard Drive (W/S Forward/Back, A/D Steer)
    2 : Crab Walk Mode (W/S Forward/Back, A/D Strafe Left/Right Sideways)
    3 : Zero-Turn Mode (A/D Spin In Place at Center of Mass)
    4 : X-Stance Brake (Lock Wheels in 45° X Pattern for Steep Slopes)

Driving Controls:
    W : Forward             S : Backward
    A : Left / Strafe       D : Right / Strafe
    SPACE : Emergency Stop / Brake

Telescopic Chassis Morphing:
    T : Extend Chassis Outward (+10mm/stage)
    G : Retract Chassis Inward (-10mm/stage)

Camera Pan / Tilt Gimbal:
    I : Tilt Up             K : Tilt Down
    J : Pan Left            L : Pan Right

Speed Adjust:
    + : Increase Max Speed  - : Decrease Max Speed
    Q / Ctrl+C : Quit
=============================================================================
"""


class SwerveTeleopNode(Node):
    def __init__(self):
        super().__init__('rover_swerve_teleop')

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_mode = self.create_publisher(String, '/rover/swerve_mode', 10)
        self.pub_cam_pan = self.create_publisher(Float64, '/rover/camera/pan/cmd_pos', 10)
        self.pub_cam_tilt = self.create_publisher(Float64, '/rover/camera/tilt/cmd_pos', 10)

        # 12 telescopic stage publishers
        self.pub_tele_l = [self.create_publisher(Float64, f'/rover/tele_l{i}/cmd_pos', 10) for i in range(1, 7)]
        self.pub_tele_r = [self.create_publisher(Float64, f'/rover/tele_r{i}/cmd_pos', 10) for i in range(1, 7)]

        # Velocities
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.max_lin = 0.8
        self.max_ang = 1.5
        self.step_lin = 0.15
        self.step_ang = 0.30

        self.active_mode = "STANDARD"  # "STANDARD", "CRAB", "ZERO_TURN", "X_BRAKE"

        # Chassis & Camera State
        self.tele_stage = 0.010
        self.cam_pan = 0.0
        self.cam_tilt = 0.0

        # Continuous publisher at 20 Hz
        self.timer = self.create_timer(0.05, self.publish_loop)

    def set_mode(self, mode_name: str):
        self.active_mode = mode_name
        msg = String()
        msg.data = "X_BRAKE" if mode_name == "X_BRAKE" else "AUTO"
        self.pub_mode.publish(msg)

    def publish_loop(self):
        msg = Twist()
        if self.active_mode != "X_BRAKE":
            msg.linear.x = float(self.vx)
            msg.linear.y = float(self.vy)
            msg.angular.z = float(self.wz)
        self.pub_cmd_vel.publish(msg)

        # Publish Camera Pan/Tilt
        fp = Float64()
        fp.data = float(self.cam_pan)
        self.pub_cam_pan.publish(fp)

        ft = Float64()
        ft.data = float(self.cam_tilt)
        self.pub_cam_tilt.publish(ft)

    def update_key(self, key):
        # Mode Switching
        if key == '1':
            self.set_mode("STANDARD")
            self.vy = 0.0
            print("\n[MODE: Standard Forward/Turn]")
        elif key == '2':
            self.set_mode("CRAB")
            self.wz = 0.0
            print("\n[MODE: Crab Walk / Omnidirectional Strafe]")
        elif key == '3':
            self.set_mode("ZERO_TURN")
            self.vx = 0.0
            self.vy = 0.0
            print("\n[MODE: Zero-Turn Spin-In-Place]")
        elif key == '4':
            self.set_mode("X_BRAKE")
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            print("\n[MODE: X-Stance Mechanical Hill Brake Engaged]")

        # Driving inputs by mode
        elif key == 'w':
            if self.active_mode != "X_BRAKE":
                self.vx = min(self.max_lin, self.vx + self.step_lin)
        elif key == 's':
            if self.active_mode != "X_BRAKE":
                self.vx = max(-self.max_lin, self.vx - self.step_lin)

        elif key == 'a':
            if self.active_mode == "CRAB":
                # Strafe left (+vy)
                self.vy = min(self.max_lin, self.vy + self.step_lin)
            elif self.active_mode in ("STANDARD", "ZERO_TURN"):
                # Steer or spin CCW (+wz)
                self.wz = min(self.max_ang, self.wz + self.step_ang)

        elif key == 'd':
            if self.active_mode == "CRAB":
                # Strafe right (-vy)
                self.vy = max(-self.max_lin, self.vy - self.step_lin)
            elif self.active_mode in ("STANDARD", "ZERO_TURN"):
                # Steer or spin CW (-wz)
                self.wz = max(-self.max_ang, self.wz - self.step_ang)

        elif key == ' ':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        # Speed adjustment
        elif key in ('+', '='):
            self.max_lin = min(2.5, self.max_lin + 0.2)
            print(f"\n[Max Speed: {self.max_lin:.1f} m/s]")
        elif key in ('-', '_'):
            self.max_lin = max(0.2, self.max_lin - 0.2)
            print(f"\n[Max Speed: {self.max_lin:.1f} m/s]")

        # Telescopic Stance
        elif key == 't':
            self.tele_stage = min(0.048, self.tele_stage + 0.005)
            self._send_chassis()
            print(f"\n[Chassis Extension: {self.tele_stage*1000:.1f} mm/stage | Total Track: {(0.476 + 12*self.tele_stage)*1000:.0f} mm]")
        elif key == 'g':
            self.tele_stage = max(0.0, self.tele_stage - 0.005)
            self._send_chassis()
            print(f"\n[Chassis Extension: {self.tele_stage*1000:.1f} mm/stage | Total Track: {(0.476 + 12*self.tele_stage)*1000:.0f} mm]")

        # Camera Gimbal
        elif key == 'i':
            self.cam_tilt = min(0.7, self.cam_tilt + 0.1)
        elif key == 'k':
            self.cam_tilt = max(-1.4, self.cam_tilt - 0.1)
        elif key == 'j':
            self.cam_pan = min(3.14, self.cam_pan + 0.15)
        elif key == 'l':
            self.cam_pan = max(-3.14, self.cam_pan - 0.15)

    def _send_chassis(self):
        f = Float64()
        f.data = float(self.tele_stage)
        for pub in self.pub_tele_l + self.pub_tele_r:
            pub.publish(f)


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
    node = SwerveTeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP_MSG)

    try:
        last_disp = time.time()
        while rclpy.ok():
            key = get_key(settings, 0.04)
            if key in ('q', 'Q', '\x03'):
                break
            elif key:
                node.update_key(key.lower())

            if time.time() - last_disp > 0.2:
                sys.stdout.write(
                    f"\r  Mode: {node.active_mode:<10} | "
                    f"Vx: {node.vx:+.2f} m/s | Vy: {node.vy:+.2f} m/s | Wz: {node.wz:+.2f} rad/s | "
                    f"Track: {(0.476 + 12*node.tele_stage)*1000:4.0f}mm   "
                )
                sys.stdout.flush()
                last_disp = time.time()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.vx = 0.0
        node.vy = 0.0
        node.wz = 0.0
        node.publish_loop()
        node.destroy_node()
        rclpy.shutdown()
        print("\nSwerve teleop terminated.")


if __name__ == '__main__':
    main()
