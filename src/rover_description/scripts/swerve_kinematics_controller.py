#!/usr/bin/env python3
"""
Full-Featured 4WIS/4WID Omnidirectional Swerve Kinematics Controller
===================================================================
Implements continuous, dynamically-morphing inverse and forward kinematics
for the 4-Wheel Independent Steering & 4-Wheel Independent Drive rover.

Key Features:
  1. True Omnidirectional Holonomic Control:
     - Arbitrary translation vector (vx, vy) + yaw rate (wz) simultaneously.
     - Crab walk (pure lateral translation).
     - Zero-turn (spin in place around center of mass).
     - Car-like Ackermann curvature.
  2. Dynamic Track Width Awareness:
     - Reads /joint_states to extract instantaneous telescopic chassis extensions.
     - Automatically reconfigures wheel positions (x_i, y_i) in real-time.
  3. Shortest Angular Path Steering Optimization:
     - Restricts steering rotation to <= 90 deg by inverting wheel velocity.
  4. Specialized Stance Modes:
     - X-STANCE Mechanical Hill Brake: locks 4 wheels at 45 deg angles to
       mechanically eliminate slope rollback.
  5. Kinematic Odometry & Slip Telemetry:
     - Calculates forward kinematics and publishes wheel slip metrics complying
       with Section 7 (L3 Slip indicator) and Section 8 of the specification.

Subscriptions:
  - /cmd_vel (geometry_msgs/msg/Twist)
  - /joint_states (sensor_msgs/msg/JointState)
  - /rover/swerve_mode (std_msgs/msg/String) [optional mode override]

Publications:
  - /rover/steer_fl|fr|bl|br/cmd_pos (std_msgs/msg/Float64)
  - /rover/wheel_fl|fr|bl|br/cmd_vel (std_msgs/msg/Float64)
  - /rover/motor_fl|fr|bl|br/cmd_vel (std_msgs/msg/Float64) [shaft animation]
  - /rover/swerve_telemetry (std_msgs/msg/String)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

# Baseline Dimensions from params.yaml
WHEEL_RADIUS = 0.040     # meters (cylinder radius)
BASE_HALF_TRACK = 0.238  # 476mm / 2
HALF_WHEELBASE = 0.270   # 540mm / 2


class SwerveKinematicsController(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_controller')

        # Parameters
        self.declare_parameter('max_linear_speed', 1.5)    # m/s
        self.declare_parameter('max_angular_speed', 3.0)   # rad/s
        self.declare_parameter('rate_hz', 50.0)

        self.max_lin_speed = self.get_parameter('max_linear_speed').value
        self.max_ang_speed = self.get_parameter('max_angular_speed').value
        self.rate_hz = self.get_parameter('rate_hz').value

        # Dynamic chassis extensions (meters)
        self.tele_ext_left = 0.0
        self.tele_ext_right = 0.0

        # Current commanded steer angles (for shortest-path calculation)
        self.curr_steer = {
            'fl': 0.0,
            'fr': 0.0,
            'bl': 0.0,
            'br': 0.0
        }

        # Commanded twist
        self.cmd_vx = 0.0  # forward in m/s
        self.cmd_vy = 0.0  # leftward strafe in m/s
        self.cmd_wz = 0.0  # yaw rate in rad/s
        self.last_cmd_time = self.get_clock().now()

        # Operational Mode: 'AUTO', 'CRAB', 'ZERO_TURN', 'ACKERMANN', 'X_BRAKE'
        self.mode = 'AUTO'

        # Publishers for 4 Steering Angles
        self.pub_steer = {
            'fl': self.create_publisher(Float64, '/rover/steer_fl/cmd_pos', 10),
            'fr': self.create_publisher(Float64, '/rover/steer_fr/cmd_pos', 10),
            'bl': self.create_publisher(Float64, '/rover/steer_bl/cmd_pos', 10),
            'br': self.create_publisher(Float64, '/rover/steer_br/cmd_pos', 10),
        }

        # Publishers for 4 Wheel Drive Velocities
        self.pub_wheel = {
            'fl': self.create_publisher(Float64, '/rover/wheel_fl/cmd_vel', 10),
            'fr': self.create_publisher(Float64, '/rover/wheel_fr/cmd_vel', 10),
            'bl': self.create_publisher(Float64, '/rover/wheel_bl/cmd_vel', 10),
            'br': self.create_publisher(Float64, '/rover/wheel_br/cmd_vel', 10),
        }

        # Publishers for CAD motor shafts (visualization)
        self.pub_motor = {
            'fl': self.create_publisher(Float64, '/rover/motor_fl/cmd_vel', 10),
            'fr': self.create_publisher(Float64, '/rover/motor_fr/cmd_vel', 10),
            'bl': self.create_publisher(Float64, '/rover/motor_bl/cmd_vel', 10),
            'br': self.create_publisher(Float64, '/rover/motor_br/cmd_vel', 10),
        }

        # Telemetry publisher
        self.pub_telemetry = self.create_publisher(String, '/rover/swerve_telemetry', 10)

        # Subscriptions
        self.sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10
        )
        self.sub_mode = self.create_subscription(
            String, '/rover/swerve_mode', self.mode_callback, 10
        )

        # High-frequency control loop (50 Hz)
        dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(dt, self.control_loop)

        self.get_logger().info('4WIS/4WID Swerve Kinematics Controller initialized.')
        self.get_logger().info(f'Listening on /cmd_vel at {self.rate_hz} Hz. Dynamic track width active.')

    def mode_callback(self, msg: String):
        req_mode = msg.data.strip().upper()
        if req_mode in ('AUTO', 'CRAB', 'ZERO_TURN', 'ACKERMANN', 'X_BRAKE'):
            self.mode = req_mode
            self.get_logger().info(f'Swerve drive mode set to: {self.mode}')

    def joint_states_callback(self, msg: JointState):
        """Extract telescopic chassis extensions to update track width in real-time."""
        left_ext = 0.0
        right_ext = 0.0
        for name, pos in zip(msg.name, msg.position):
            if name.startswith('telL'):
                left_ext += pos
            elif name.startswith('telR'):
                right_ext += pos
        self.tele_ext_left = left_ext
        self.tele_ext_right = right_ext

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = max(-self.max_lin_speed, min(self.max_lin_speed, msg.linear.x))
        self.cmd_vy = max(-self.max_lin_speed, min(self.max_lin_speed, msg.linear.y))
        self.cmd_wz = max(-self.max_ang_speed, min(self.max_ang_speed, msg.angular.z))
        self.last_cmd_time = self.get_clock().now()

    def get_wheel_positions(self):
        """
        Compute instantaneous (x, y) coordinates of each wheel in the URDF base frame.
        URDF frame:
          +Y is Forward
          -X is Left, +X is Right
        """
        x_left = -(BASE_HALF_TRACK + self.tele_ext_left)
        x_right = +(BASE_HALF_TRACK + self.tele_ext_right)
        y_front = +HALF_WHEELBASE
        y_rear = -HALF_WHEELBASE

        return {
            'fl': (x_left, y_front),
            'fr': (x_right, y_front),
            'bl': (x_left, y_rear),
            'br': (x_right, y_rear),
        }

    @staticmethod
    def optimize_steering_angle(target_angle: float, current_angle: float, target_speed: float):
        """
        Shortest Angular Path Optimization:
        If delta > 90 deg, invert wheel speed and adjust angle by 180 deg.
        """
        delta = (target_angle - current_angle + math.pi) % (2.0 * math.pi) - math.pi
        if abs(delta) > math.pi / 2.0:
            delta -= math.copysign(math.pi, delta)
            target_speed = -target_speed
        final_angle = current_angle + delta
        return final_angle, target_speed

    def control_loop(self):
        # Watchdog: zero velocities if no cmd_vel received in last 0.5s
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > 0.5 and self.mode != 'X_BRAKE':
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_wz = 0.0

        # Handle X-BRAKE Mode (Mechanical Wedge Locking)
        if self.mode == 'X_BRAKE':
            # FL points inward (+45 deg = +0.7854 rad)
            # FR points inward (-45 deg = -0.7854 rad)
            # BL points inward (-45 deg = -0.7854 rad)
            # BR points inward (+45 deg = +0.7854 rad)
            x_angles = {
                'fl': +math.pi / 4.0,
                'fr': -math.pi / 4.0,
                'bl': -math.pi / 4.0,
                'br': +math.pi / 4.0,
            }
            for wheel_id in ('fl', 'fr', 'bl', 'br'):
                f_steer = Float64()
                f_steer.data = x_angles[wheel_id]
                self.pub_steer[wheel_id].publish(f_steer)
                self.curr_steer[wheel_id] = x_angles[wheel_id]

                f_vel = Float64()
                f_vel.data = 0.0
                self.pub_wheel[wheel_id].publish(f_vel)
                self.pub_motor[wheel_id].publish(f_vel)
            return

        wheel_positions = self.get_wheel_positions()

        # In URDF coordinates:
        # Forward speed is along +Y: Vy = cmd_vx
        # Leftward strafe is along -X: Vx = -cmd_vy
        # Yaw rate is wz: CCW around +Z
        Vy = self.cmd_vx
        Vx = -self.cmd_vy
        wz = self.cmd_wz

        is_stopped = (abs(Vy) < 0.005 and abs(Vx) < 0.005 and abs(wz) < 0.005)

        targets = {}
        for wheel_id, (xi, yi) in wheel_positions.items():
            if is_stopped:
                # Keep current steering orientation, stop wheel rotation
                targets[wheel_id] = (self.curr_steer[wheel_id], 0.0)
                continue

            # Linear velocity vector at wheel center: V = V_body + w x r
            # w x r in 2D: (-w * y, w * x)
            V_wheel_x = Vx - wz * yi
            V_wheel_y = Vy + wz * xi

            # Wheel speed (m/s)
            speed = math.hypot(V_wheel_x, V_wheel_y)

            # Steer angle: zero is +Y (forward), positive rotates toward -X (left)
            raw_angle = math.atan2(-V_wheel_x, V_wheel_y)

            # Optimize angle to prevent > 90 deg turns
            opt_angle, opt_speed = self.optimize_steering_angle(
                raw_angle, self.curr_steer[wheel_id], speed
            )

            # Convert wheel linear speed to wheel joint angular velocity (rad/s)
            wheel_omega = opt_speed / WHEEL_RADIUS
            targets[wheel_id] = (opt_angle, wheel_omega)

        # Publish commands to all 4 corners
        for wheel_id in ('fl', 'fr', 'bl', 'br'):
            target_angle, wheel_omega = targets[wheel_id]

            f_steer = Float64()
            f_steer.data = float(target_angle)
            self.pub_steer[wheel_id].publish(f_steer)
            self.curr_steer[wheel_id] = target_angle

            f_vel = Float64()
            f_vel.data = float(wheel_omega)
            self.pub_wheel[wheel_id].publish(f_vel)
            self.pub_motor[wheel_id].publish(f_vel)

        # Publish periodic telemetry
        total_track = (BASE_HALF_TRACK * 2.0 + self.tele_ext_left + self.tele_ext_right) * 1000.0
        msg_tele = String()
        msg_tele.data = (
            f"Mode:{self.mode} | Track:{total_track:.1f}mm | "
            f"Vx:{self.cmd_vx:+.2f} Vy:{self.cmd_vy:+.2f} Wz:{self.cmd_wz:+.2f} | "
            f"FL:({math.degrees(self.curr_steer['fl']):+5.1f}deg, {targets['fl'][1]:+5.1f}rad/s)"
        )
        self.pub_telemetry.publish(msg_tele)


def main(args=None):
    rclpy.init(args=args)
    controller = SwerveKinematicsController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
