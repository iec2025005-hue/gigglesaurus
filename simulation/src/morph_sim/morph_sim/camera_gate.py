"""Presentation camera relay; the simulated camera itself keeps running."""
import rclpy
from . import lifecycle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool


class CameraGate(Node):
    def __init__(self):
        super().__init__('morph_camera_gate')
        self.enabled = True
        self.image_pub = self.create_publisher(Image, '/morph/camera/demo_image', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/morph/camera/demo_camera_info', qos_profile_sensor_data)
        self.create_subscription(Image, '/morph/camera/image', self.image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/morph/camera/camera_info', self.info, qos_profile_sensor_data)
        self.create_subscription(Bool, '/morph/camera/show', self.show, 10)

    def show(self, message):
        self.enabled = bool(message.data)
        self.get_logger().info('Presentation camera feed %s' % ('ON' if self.enabled else 'OFF'))

    def image(self, message):
        if self.enabled:
            self.image_pub.publish(message)

    def info(self, message):
        if self.enabled:
            self.info_pub.publish(message)


def main():
    rclpy.init()
    node = CameraGate()
    try:
        lifecycle.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
