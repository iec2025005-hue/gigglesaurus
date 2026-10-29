"""Bound command increments and servo lead without changing actuator limits."""
import math
import xml.etree.ElementTree as ET
import yaml


class PositionTrajectoryLimiter:
    def __init__(self,urdf_file,controllers_file,height_force_servo=False):
        self.height_force_servo=height_force_servo
        robot=ET.parse(urdf_file).getroot()
        controlled={j.get('name') for j in robot.findall('./ros2_control/joint') if j.find("command_interface[@name='position']") is not None}
        self.limits={}
        for joint in robot.findall('joint'):
            if joint.get('name') in controlled:
                limit=joint.find('limit');self.limits[joint.get('name')]=tuple(float(limit.get(key)) for key in ('lower','upper','velocity'))
        config=yaml.safe_load(open(controllers_file).read());self.period=1/config['controller_manager']['ros__parameters']['update_rate']
        gain=float(robot.findtext("./gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/position_proportional_gain"))
        self.servo_rate=gain/self.period
        if self.servo_rate<=0 or self.period<=0 or len(self.limits)!=24:raise ValueError('Incomplete position-controller limits or servo timing')
        self.previous={j.get('name'):float(j.findtext("state_interface[@name='position']/param[@name='initial_value']",default='0')) for j in robot.findall('./ros2_control/joint') if j.get('name') in controlled}
    def apply(self,requested,measured,dt):
        output={};trace=[];period=min(max(dt,0.),self.period)
        for name,target in requested.items():
            low,high,velocity=self.limits[name];actual=measured[name]
            if not all(math.isfinite(v) for v in (target,actual)):raise ValueError('Nonfinite position trajectory')
            previous=self.previous.get(name,min(high,max(low,actual)))
            force_backend=self.height_force_servo and name.endswith('_height_joint')
            lead=high-low if force_backend else .9*velocity/self.servo_rate
            feasible_low=max(low,actual-lead);feasible_high=min(high,actual+lead)
            bounded=min(feasible_high,max(feasible_low,target))
            step=.95*velocity*period
            published=min(high,max(low,min(previous+step,max(previous-step,bounded))))
            self.previous[name]=published;output[name]=published
            trace.append({'joint':name,'servo_backend':'force' if force_backend else 'velocity','requested_position':target,'published_position':published,'previous_position':previous,'measured_position':actual,'tracking_error':published-actual,'command_velocity':(published-previous)/period if period else 0.,'velocity_limit':velocity,'lead_limit':lead,'clamped':published!=target,'lead_conflict':published<feasible_low-1e-12 or published>feasible_high+1e-12})
        return output,trace
