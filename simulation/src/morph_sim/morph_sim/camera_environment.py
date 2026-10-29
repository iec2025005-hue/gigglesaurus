"""Optional image-only scenario effects. Raw camera topic is never modified.

No perception accuracy claim: this node has no detector or ground-truth labels.
Exposure, blur, occlusion and delay are deliberately simple assumed models.
"""
from collections import deque
import copy,json,math,random
import numpy as np
from PIL import Image as PILImage, ImageFilter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from .design_physics import load_parameters,clamp


class CameraEnvironment(Node):
    def __init__(self):
        super().__init__('morph_camera_environment')
        self.declare_parameter('params_file','');self.p=load_parameters(self.get_parameter('params_file').value)
        self.settings=self.p['sensors'];self.env=self.p['environment']['scenarios']['indoor'];self.rng=random.Random(self.p['provenance']['seed']+1)
        self.queue=deque(maxlen=8);self.omega=0.;self.brightness=1.;self.previous=None
        self.pub=self.create_publisher(Image,'/morph/camera/degraded_image',qos_profile_sensor_data)
        self.create_subscription(Image,'/morph/camera/image',self.image,qos_profile_sensor_data)
        self.create_subscription(Imu,'/morph/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(String,'/morph/physics/status',self.status,10)
        self.create_timer(.01,self.release)
    def imu(self,m):self.omega=math.sqrt(m.angular_velocity.x**2+m.angular_velocity.y**2+m.angular_velocity.z**2)
    def status(self,m):
        try:self.env=json.loads(m.data)['environment']
        except (ValueError,KeyError):pass
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def image(self,m):
        if m.encoding not in ('rgb8','bgr8') or self.rng.random()<self.settings['camera_drop_probability']:return
        now=self.now();dt=0. if self.previous is None else max(0.,now-self.previous);self.previous=now
        target=clamp(math.sqrt(self.env['lux']/400),.08,1.5)
        self.brightness+=(target-self.brightness)*(1-math.exp(-dt/.5))
        a=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3).copy()
        if m.encoding=='bgr8':a=a[:,:,::-1]
        a=np.clip(a.astype(float)*self.brightness,0,255).astype(np.uint8)
        image=PILImage.fromarray(a)
        blur=clamp(self.omega*self.settings['camera_exposure_s']*self.settings['camera_focal_px'],0.,12.)
        if blur>.25:image=image.filter(ImageFilter.GaussianBlur(blur*.35))
        a=np.asarray(image).copy();fraction=clamp(self.settings['camera_lens_dirt_fraction'],0.,.5)
        if fraction>0:a[:int(m.height*math.sqrt(fraction)),:int(m.width*math.sqrt(fraction))]//=4
        out=copy.deepcopy(m);out.encoding='rgb8';out.step=m.width*3;out.data=a.tobytes()
        self.queue.append((now+self.settings['camera_latency_s'],out))
    def release(self):
        now=self.now()
        while self.queue and self.queue[0][0]<=now:self.pub.publish(self.queue.popleft()[1])


def main():
    rclpy.init();node=CameraEnvironment()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
