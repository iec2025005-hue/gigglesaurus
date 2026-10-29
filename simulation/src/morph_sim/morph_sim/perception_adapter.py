"""Small local geometry observer for supervised chassis adaptation."""
import json,math
import numpy as np
import rclpy
from . import lifecycle
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu,JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from .perception_core import assess,DecisionFilter

def xyz_from_cloud(m):
 fields={f.name:f for f in m.fields}
 if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):raise ValueError('Expected float32 x/y/z fields')
 endian='>' if m.is_bigendian else '<'
 dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[k].offset for k in ('x','y','z')],'itemsize':m.point_step})
 a=np.ndarray((m.height,m.width),dtype=dtype,buffer=m.data,strides=(m.row_step,m.point_step))
 return np.column_stack([a[k].ravel() for k in ('x','y','z')])

class PerceptionAdapter(Node):
 def __init__(self):
  super().__init__('morph_local_perception');self.declare_parameter('wheel_radius',.075);self.declare_parameter('processing_enabled',True);self.filter=DecisionFilter();self.roll=0.;self.pitch=0.;self.imu_time=-1e9;self.states={}
  self.pub=self.create_publisher(String,'/morph/perception/decision',1);self.marker=self.create_publisher(Marker,'/morph/perception/marker',1)
  self.create_subscription(PointCloud2,'/morph/perception/points',self.cloud,qos_profile_sensor_data)
  self.create_subscription(Imu,'/morph/imu',self.imu,qos_profile_sensor_data);self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
 def imu(self,m):
  q=m.orientation;norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
  if norm<.5 or not math.isfinite(norm):return
  x,y,z,w=q.x/norm,q.y/norm,q.z/norm,q.w/norm
  self.roll=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y));self.pitch=math.asin(max(-1.,min(1.,2*(w*y-z*x))));self.imu_time=self.get_clock().now().nanoseconds*1e-9
 def joints(self,m):self.states=dict(zip(m.name,m.position))
 def cloud(self,m):
  if not self.get_parameter('processing_enabled').value:return
  now=self.get_clock().now().nanoseconds*1e-9
  try:
   if now-self.imu_time>.3:result={'valid':False,'reason':'IMU stale'}
   else:
    names=('fl','fr','rl','rr');height=.26+float(self.get_parameter('wheel_radius').value)+.12+sum(self.states.get(n+'_height_joint',0)-self.states.get(n+'_spring_joint',0) for n in names)/4
    result=self.filter.update(assess(xyz_from_cloud(m),height,self.roll,self.pitch))
   result['sim_time_s']=now;self.pub.publish(String(data=json.dumps(result,allow_nan=False)))
   marker=Marker();marker.header.frame_id='base_link';marker.header.stamp=self.get_clock().now().to_msg();marker.ns='perception';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING;marker.action=Marker.ADD;marker.pose.position.z=.55;marker.pose.orientation.w=1.;marker.lifetime.sec=0;marker.lifetime.nanosec=400000000;marker.scale.z=.065;marker.color.r=.2;marker.color.g=1.;marker.color.b=.85;marker.color.a=1.;marker.text=f"SENSED: {result.get('shape','unknown')} | {result.get('reason')}";self.marker.publish(marker)
  except (ValueError,TypeError,KeyError) as exc:self.pub.publish(String(data=json.dumps({'valid':False,'reason':str(exc),'sim_time_s':now})))

def main():
 lifecycle.initialize();node=PerceptionAdapter()
 try:lifecycle.spin(node)
 except KeyboardInterrupt:pass
 finally:
  node.destroy_node()
  if rclpy.ok():rclpy.shutdown()
