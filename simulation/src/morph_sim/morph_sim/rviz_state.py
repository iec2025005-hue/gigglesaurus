"""Explicit ground-truth TF/path for visualization, separate from encoder odometry."""
import rclpy
from . import lifecycle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import TransformStamped,PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from pathlib import Path as FilePath
import xml.etree.ElementTree as ET

class VisualizationState(Node):
 def __init__(self):
  super().__init__('morph_rviz_state');self.tf=TransformBroadcaster(self);self.path=Path();self.path.header.frame_id='world';self.last=-1e9
  self.pub=self.create_publisher(Path,'/morph/ground_truth/path',1)
  self.create_subscription(Odometry,'/morph/ground_truth/odom',self.odom,qos_profile_sensor_data)
  qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.map=self.create_publisher(MarkerArray,'/morph/arena/markers',qos)
  self.declare_parameter('arena','multisector');self.publish_map()
 def publish_map(self):
  if self.get_parameter('arena').value!='multisector':return
  p=FilePath(get_package_share_directory('morph_sim'));r=ET.parse(p/'worlds/multisector.sdf');markers=MarkerArray()
  for i,v in enumerate(r.findall("./world/model[@name='MORPH_MULTITERRAIN_COURSE']/link/visual")):
   uri=v.findtext('geometry/mesh/uri')
   if not uri or 'label' in uri:continue
   m=Marker();m.header.frame_id='world';m.header.stamp=self.get_clock().now().to_msg();m.ns='terrain';m.id=i;m.type=Marker.MESH_RESOURCE;m.action=Marker.ADD;m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=1.;m.color.r=.48;m.color.g=.55;m.color.b=.52;m.color.a=.65;m.mesh_resource='package://morph_sim/models/'+uri.removeprefix('model://');m.mesh_use_embedded_materials=False;markers.markers.append(m)
  self.map.publish(markers)
 def odom(self,m):
  tr=TransformStamped();tr.header.stamp=m.header.stamp;tr.header.frame_id='world';tr.child_frame_id='base_link';tr.transform.translation.x=m.pose.pose.position.x;tr.transform.translation.y=m.pose.pose.position.y;tr.transform.translation.z=m.pose.pose.position.z;tr.transform.rotation=m.pose.pose.orientation;self.tf.sendTransform(tr)
  now=self.get_clock().now().nanoseconds*1e-9
  if now-self.last<.1:return
  self.last=now;pose=PoseStamped();pose.header=tr.header;pose.pose=m.pose.pose;self.path.poses.append(pose);self.path.poses=self.path.poses[-1000:];self.path.header.stamp=m.header.stamp;self.pub.publish(self.path)

def main():
 lifecycle.initialize();n=VisualizationState()
 try:lifecycle.spin(n)
 except KeyboardInterrupt:pass
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
