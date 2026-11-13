"""Camera node: colour-class terrain telemetry on /morph/vision/telemetry (see vision_core)."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from . import lifecycle
from .vision_core import analyse


class TerrainVision(Node):
    def __init__(self):
        super().__init__('morph_terrain_vision'); self.declare_parameter('filter_frames', 5)
        self.history = []; self.pub = self.create_publisher(String, '/morph/vision/telemetry', 5)
        self.create_subscription(Image, '/morph/camera/image', self.on_image, qos_profile_sensor_data)

    def on_image(self, m):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            if m.encoding not in ('rgb8', 'bgr8'): raise ValueError('unsupported encoding ' + m.encoding)
            image = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step // 3 if m.step else m.width, 3)[:, :m.width]
            if m.encoding == 'bgr8': image = image[:, :, ::-1]
            result = analyse(image)
        except (ValueError, TypeError) as exc:
            result = {'valid': False, 'reason': str(exc)}
        if result.get('valid'):
            # Majority vote over the last frames so one odd frame cannot flip the class.
            self.history = (self.history + [result['class_here']])[-int(self.get_parameter('filter_frames').value):]
            result['class_filtered'] = max(set(self.history), key=self.history.count); result['frames_agreeing'] = self.history.count(result['class_filtered'])
        result['sim_time_s'] = now; result['image_stamp_s'] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pub.publish(String(data=json.dumps(result)))


def main():
    lifecycle.initialize(); node = TerrainVision()
    try: lifecycle.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
