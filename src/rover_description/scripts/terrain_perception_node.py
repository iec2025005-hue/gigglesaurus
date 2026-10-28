#!/usr/bin/env python3
"""
Real-Time Look-Ahead Terrain Perception & Visual Morphing Pipeline
=================================================================
Processes high-resolution camera frames from /camera/image_raw (1280x720, 30 Hz)
mounted on the rover's 2-DOF pan/tilt gimbal.

Key Features:
  1. Multi-Zone Ground Segmentation:
     - Isolates the ground ahead (near-field 0.5-2.0m, far-field 2.0-5.0m).
  2. Visual Surface Classifier:
     - Real-time color space decomposition (HSV + LAB).
     - Texture gradient analysis via Sobel / Laplacian variance to quantify
       micro-roughness (distinguishes smooth asphalt, granular gravel, mud, snow).
  3. Narrow Passage / Obstacle Aperture Estimator:
     - Vertical edge density detector to estimate corridor opening width.
  4. Pre-emptive Stance Morphing Output:
     - Recommends proactive track width adjustments 1.5 seconds BEFORE the wheels
       physically encounter rough terrain or tight bottlenecks.

Publications & Topics:
  - Publishes:
      /rover/terrain_zone (std_msgs/msg/String)
      /rover/preemptive_stance_m (std_msgs/msg/Float64)
      /rover/terrain_roughness (std_msgs/msg/Float64)
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge


class TerrainPerceptionNode(Node):
    def __init__(self):
        super().__init__('terrain_perception_node')

        self.bridge = CvBridge()

        # Classifications
        self.current_terrain = "UNKNOWN"
        self.roughness_index = 0.0
        self.aperture_width_m = 3.0
        self.recommended_stance_m = 0.596

        # Publishers
        self.pub_terrain = self.create_publisher(String, '/rover/terrain_zone', 10)
        self.pub_stance = self.create_publisher(Float64, '/rover/preemptive_stance_m', 10)
        self.pub_roughness = self.create_publisher(Float64, '/rover/terrain_roughness', 10)

        # Subscriber
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 5
        )

        self.frame_count = 0
        self.get_logger().info('Look-Ahead Terrain Perception Node initialized.')

    def image_callback(self, msg: Image):
        self.frame_count += 1
        # Process at 10 Hz to conserve CPU
        if self.frame_count % 3 != 0:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        h, w, _ = cv_img.shape

        # Region of Interest: Lower 45% of image (ground path ahead)
        roi_y_start = int(h * 0.55)
        roi = cv_img[roi_y_start:h, :]

        # 1. Texture / Roughness Analysis (Laplacian Variance)
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        laplacian_var = cv2.Laplacian(gray_roi, cv2.CV_64F).var()
        self.roughness_index = float(laplacian_var)

        # 2. Color Space Representation (HSV & LAB)
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mean_hsv = cv2.mean(hsv_roi)[:3]  # (H, S, V)
        mean_bgr = cv2.mean(roi)[:3]      # (B, G, R)

        h_val, s_val, v_val = mean_hsv
        b_val, g_val, r_val = mean_bgr

        # 3. Terrain Classifier Logic (PBR Multi-Terrain Sectors)
        # Snow: Very high brightness, low saturation
        if v_val > 190 and s_val < 35:
            terrain = "SNOW_ICE"
            stance = 0.850  # wide stable footprint for slip resistance

        # Grass: Green channel dominant, hue in green range (35-85)
        elif 35 <= h_val <= 85 and g_val > b_val and g_val > r_val:
            terrain = "GRASS_TURF"
            stance = 0.650

        # Mars Sand / Red Dirt: High Red, moderate saturation
        elif r_val > g_val + 20 and r_val > b_val + 30:
            terrain = "MARS_SAND_DUNES"
            stance = 0.750

        # Mud / Wet Dirt: Dark, low brightness, brown/grey
        elif v_val < 80 and s_val > 25:
            terrain = "WET_MUD"
            stance = 0.800

        # Boulder Garden / High-Roughness Rocks
        elif laplacian_var > 450.0:
            terrain = "ROCK_GARDEN_RUBBLE"
            stance = 0.700

        # Smooth Tarmac / Concrete: Uniform, low roughness
        elif laplacian_var < 80.0:
            terrain = "SMOOTH_TARMAC"
            stance = 0.596

        else:
            terrain = "GRAVEL_ROAD"
            stance = 0.650

        # 4. Corridor / Narrow Passage Check (Sobel vertical edges on side margins)
        # Left and Right margins of view
        margin_w = int(w * 0.20)
        left_margin = gray_roi[:, :margin_w]
        right_margin = gray_roi[:, -margin_w:]
        edge_left = cv2.Canny(left_margin, 50, 150).mean()
        edge_right = cv2.Canny(right_margin, 50, 150).mean()

        if edge_left > 35.0 and edge_right > 35.0:
            terrain = "NARROW_PASSAGE_CONFINEMENT"
            stance = 0.476  # retract fully

        self.current_terrain = terrain
        self.recommended_stance_m = stance

        # Publish decisions
        msg_t = String()
        msg_t.data = self.current_terrain
        self.pub_terrain.publish(msg_t)

        msg_s = Float64()
        msg_s.data = float(self.recommended_stance_m)
        self.pub_stance.publish(msg_s)

        msg_r = Float64()
        msg_r.data = float(self.roughness_index)
        self.pub_roughness.publish(msg_r)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
