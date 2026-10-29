import math, pathlib, sys, xml.etree.ElementTree as ET
import numpy as np
import pytest, yaml, xacro
P=pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0,str(P))
from morph_sim.control import AdaptationCore, STAGES, ARMS, LEGS, D_MIN, D_MAX, optimized_wheel, spring_force
from morph_sim.assets import enrich_sdf
from morph_sim.terrain_zones import ZONES, zone_at

def states():return {n:0. for n in STAGES+ARMS+tuple(x+y for x in LEGS for y in ('_steer_joint','_height_joint','_wheel_joint'))}
@pytest.fixture(scope='module')
def robot():return ET.fromstring(xacro.process_file(str(P/'urdf/morph.urdf.xacro'),mappings={'controllers_file':str(P/'config/controllers.yaml')}).toxml())

def test_tree_and_mass(robot):
 links={l.attrib['name']:l for l in robot.findall('link')};parents={j.find('child').attrib['link']:j.find('parent').attrib['link'] for j in robot.findall('joint')}
 assert len(parents)==len(robot.findall('joint'));assert set(links)-set(parents)=={'base_link'}
 for name in links:
  seen=set()
  while name in parents:
   assert name not in seen;seen.add(name);name=parents[name]
  assert name=='base_link'
 assert sum(float(l.find('inertial/mass').attrib['value']) for l in links.values())==pytest.approx(10.)

def test_inertias_positive_and_physical(robot):
 for link in robot.findall('link'):
  v=link.find('inertial/inertia').attrib
  a=np.array([[float(v['ixx']),float(v['ixy']),float(v['ixz'])],[float(v['ixy']),float(v['iyy']),float(v['iyz'])],[float(v['ixz']),float(v['iyz']),float(v['izz'])]])
  eig=np.linalg.eigvalsh(a);assert min(eig)>0;assert max(eig)<=sum(eig)-max(eig)+1e-10

def test_controller_interfaces_match(robot):
 cfg=yaml.safe_load((P/'config/controllers.yaml').read_text());hw=robot.find('ros2_control');interfaces={j.attrib['name']:j.find('command_interface').attrib['name'] for j in hw.findall('joint') if j.find('command_interface') is not None}
 configured=[]
 for key in ['drive','steering','height','shape']:
  ns=cfg[key+'_controller']['ros__parameters']['joints'];configured.extend(ns)
  assert all(interfaces[n]==('velocity' if key=='drive' else 'position') for n in ns)
 assert len(set(configured))==len(configured)==28;assert set(configured)==set(interfaces)
 assert all(hw.find("joint[@name='"+n+"_spring_joint']/command_interface") is None for n in LEGS)

def test_resources_and_joint_limits(robot):
 for mesh in robot.findall('.//mesh'):
  f=P/mesh.attrib['filename'].split('package://morph_sim/')[1];assert f.exists() and f.stat().st_size>100
 for j in robot.findall('joint'):
  if j.attrib['type']=='revolute':assert float(j.find('limit').attrib['upper'])>float(j.find('limit').attrib['lower'])
 assert float(robot.find("joint[@name='fl_steer_joint']/limit").attrib['upper'])-float(robot.find("joint[@name='fl_steer_joint']/limit").attrib['lower'])==pytest.approx(math.pi)

def test_wheel_axis_moves_forward_for_positive_command(robot):
 for name in LEGS:
  axis=[float(x) for x in robot.find("joint[@name='"+name+"_wheel_joint']/axis").attrib['xyz'].split()]
  # Rolling without slip: center velocity = -(angular velocity cross contact radius).
  assert -np.cross(axis,[0,0,-1])[0]>0

def test_gazebo_keyboard_bridge_and_controlled_shape(robot):
 bridge=yaml.safe_load((P/'config/bridge.yaml').read_text())
 assert any(x['ros_topic_name']=='/keyboard/keypress' and x['direction']=='GZ_TO_ROS' for x in bridge)
 world=ET.parse(P/'worlds/competition.sdf')
 assert world.find("./world/gui/plugin[@filename='KeyPublisher']") is not None
 assert sum(j.find('command_interface') is not None and j.find('command_interface').attrib['name']=='position' for j in robot.findall('./ros2_control/joint'))>=20

@pytest.mark.parametrize('vx,vy',[(-1.,0.),(0.,1.),(0.,-1.),(.3,.2),(-.4,-.1),(0.,0.)])
def test_swerve_reconstructs_command(vx,vy):
 a,w=optimized_wheel(vx,vy);assert -math.pi/2<=a<=math.pi/2;assert w*.04*math.cos(a)==pytest.approx(vx,abs=1e-8);assert w*.04*math.sin(a)==pytest.approx(vy,abs=1e-8)

def test_command_does_not_drive_without_feedback():
 c=AdaptationCore()
 for _ in range(200):o=c.step(.02,[1,0,0],{})
 assert o.wheels==[0.]*4

def test_stop_latches_and_reset():
 c=AdaptationCore();s=states()
 for _ in range(100):o=c.step(.02,[.2,0,0],s)
 assert max(o.wheels)>0
 o=c.step(.02,[.2,0,0],s,underbody_hit=True);assert c.estop
 for _ in range(100):o=c.step(.02,[.2,0,0],s)
 assert o.wheels==[0.]*4
 c.event('reset');assert not c.estop

def test_stop_holds_captured_targets_despite_measurement_drift():
 c=AdaptationCore();s=states();s[LEGS[0]+'_height_joint']=-.015000001
 c.event('estop');first=c.step(.02,[0,0,0],s)
 s[LEGS[0]+'_height_joint']=-.014;s[STAGES[0]]=.002
 later=c.step(.02,[0,0,0],s)
 assert later.heights==first.heights
 assert later.shape==first.shape
 assert min(later.heights)>=-.015

def test_morph_rate_and_bounds():
 c=AdaptationCore();s=states();c.event('compact')
 for _ in range(2000):
  o=c.step(.02,[0,0,0],s)
  for i,n in enumerate(STAGES+ARMS):s[n]=o.shape[i]
 assert D_MIN<=o.shape[0]<=D_MAX;assert o.shape[0]==pytest.approx(D_MIN,abs=.0002)
 c.event('expand')
 for _ in range(3000):
  o=c.step(.02,[0,0,0],s)
  for i,n in enumerate(STAGES+ARMS):s[n]=o.shape[i]
 assert o.shape[0]==pytest.approx(D_MAX,abs=.0002)

def test_combined_morph_compensates_wheel_velocity():
 c=AdaptationCore();s=states();c.event('expand');moving=False
 for _ in range(500):
  o=c.step(.01,[.12,.04,.1],s)
  for i,n in enumerate(STAGES+ARMS):s[n]=o.shape[i]
  for i,n in enumerate(LEGS):s[n+'_steer_joint']=o.steering[i]
  if o.status['morph_rate_m_s'] and max(abs(w) for w in o.wheels)>0:moving=True
 assert moving and s[STAGES[0]]>.01
 assert o.status['command_speed']>.1

def test_morph_with_position_controller_lag():
 c=AdaptationCore();s=states();c.event('expand')
 for _ in range(800):
  o=c.step(.02,[0,0,0],s)
  for i,n in enumerate(STAGES+ARMS):s[n]+=(o.shape[i]-s[n])*.1
  for i,n in enumerate(LEGS):s[n+'_steer_joint']=o.steering[i]
 assert s[STAGES[0]]>D_MAX-.0003

def test_tilt_and_suspension_bounds():
 c=AdaptationCore();c.auto=True;c.roll=.3;c.pitch=.25;o=c.step(.02,[0,0,0],states(),imu_fresh=True)
 assert o.status['mode']=='climb';assert all(-.015<=v<=.015 for v in o.heights)
 c=AdaptationCore();c.auto=True;c.roll=.3;o=c.step(.02,[0,0,0],states(),imu_fresh=False);assert o.status['mode']=='track'

def test_spring_dissipates_energy():
 for q in np.linspace(-.01,.03,9):
  for v in np.linspace(-.2,.2,9):
   f=spring_force(q,v,1800,65);assert abs(f)<=150
   # For this unclipped range d(total mechanical energy)/dt = -c*v^2.
   assert f*v+1800*q*v<=1e-10

def test_sensor_pbr_enrichment():
 root=ET.fromstring('<sdf version="1.9"><model name="morph"><link name="base_link"><collision name="underbody_collision"/><visual name="base"/></link>'+''.join('<link name="'+n+'_wheel"><collision name="'+n+'_wheel_collision"/><visual name="'+n+'_tire"/></link>' for n in LEGS)+'</model></sdf>')
 ET.SubElement(root.find('model'),'frame',name='camera_link',attached_to='base_link')
 enrich_sdf(root,P.resolve(),camera_enabled=True);assert len(root.findall('.//sensor'))==8
 for sensor in root.findall('.//sensor[@type="contact"]'):
  assert sensor.find('contact/collision') is not None and sensor.find('topic') is not None
 assert len(root.findall('.//pbr/metal'))==5
 for tex in root.findall('.//pbr/metal/albedo_map'):assert pathlib.Path(__import__('urllib.parse',fromlist=['urlparse']).unquote(__import__('urllib.parse',fromlist=['urlparse']).urlparse(tex.text).path.lstrip('/') if sys.platform=='win32' else __import__('urllib.parse',fromlist=['urlparse']).urlparse(tex.text).path)).exists()

def test_camera_sensor_and_lamp():
 root=ET.fromstring('<sdf version="1.9"><model name="morph"><link name="base_link"><collision name="underbody_collision"/></link><link name="camera_link"/>'+''.join('<link name="'+n+'_wheel"><collision name="'+n+'_wheel_collision"/></link>' for n in LEGS)+'</model></sdf>')
 enrich_sdf(root,P.resolve(),camera_enabled=True)
 assert root.find("./model/link[@name='camera_link']/sensor[@type='camera']/camera/image/width").text=='480'
 assert root.find("./model/link[@name='camera_link']/light[@name='camera_fill']") is not None

def test_arena_and_bridge():
 world=ET.parse(P/'worlds/competition.sdf');assert len(world.findall('.//heightmap'))==6
 plugins={p.attrib['name'] for p in world.findall('./world/plugin')};assert {'gz::sim::systems::Physics','gz::sim::systems::Imu','gz::sim::systems::Contact','gz::sim::systems::Sensors'}<=plugins
 bridges=yaml.safe_load((P/'config/bridge.yaml').read_text());assert len({b['ros_topic_name'] for b in bridges})==len(bridges)
 assert any(b['ros_topic_name']=='/morph/actuator/watchdog' and b['direction']=='ROS_TO_GZ' for b in bridges)
 assert any(b['ros_topic_name']=='/morph/watchdog/status' and b['direction']=='GZ_TO_ROS' for b in bridges)
 for el in world.iter():
  if el.text and el.text.startswith('model://morph_sim/'):assert (P/el.text.split('model://morph_sim/')[1]).exists()


def test_flip_sweep_and_no_driving():
 c=AdaptationCore();s=states();c.event('flip');angles=[]
 for _ in range(700):
  o=c.step(.02,[0,0,0],s);angles.append(o.steering[0])
  for i,n in enumerate(LEGS):s[n+'_steer_joint']=o.steering[i]
  assert all(abs(w)<1e-9 for w in o.wheels)
  if c.flip_phase is None:break
 assert min(angles)<-1.45 and max(angles)>1.45 and c.flip_phase is None

@pytest.mark.parametrize('radius',[0.,-.04,.5])
def test_invalid_wheel_radius_rejected(radius):
 with pytest.raises(ValueError):AdaptationCore(radius)


def test_sdf_plugins_and_contact_references():
 root=ET.fromstring('<sdf version="1.9"><model name="morph"><link name="base_link"><collision name="underbody_collision"/></link>'+''.join('<link name="'+n+'_wheel"><collision name="'+n+'_wheel_collision"/></link>' for n in LEGS)+'</model></sdf>')
 ET.SubElement(root.find('model'),'frame',name='camera_link',attached_to='base_link')
 enrich_sdf(root,P.resolve())
 for link in root.findall('./model/link'):
  collisions={c.attrib['name'] for c in link.findall('collision')}
  for sensor in link.findall("sensor[@type='contact']"):assert sensor.find('contact/collision').text in collisions
 assert root.find("./model/plugin[@name='gz::sim::systems::OdometryPublisher']") is not None

def test_marsyard_terrain_collision_and_resources():
 world=ET.parse(P/'worlds/adaptive_arena.sdf')
 assert world.find('./world').attrib['name']=='leo_marsyard'
 terrain=ET.parse(P/'models/marsyard2021_terrain/model.sdf')
 for link in terrain.findall('./model/link'):
  collisions=link.findall('collision')
  visuals=link.findall('visual')
  assert len(collisions)==len(visuals)
  for collision,visual in zip(collisions,visuals):
   assert collision.find('geometry')[0].find('uri').text.strip()==visual.find('geometry')[0].find('uri').text.strip()
   assert float(collision.find('surface/friction/ode/mu').text)>0
 for element in terrain.iter():
  if element.text and element.text.strip().startswith('model://marsyard2021_terrain/'):
   assert (P/'models'/element.text.strip().removeprefix('model://')).exists()

def test_automatic_morph_coordinates_at_limited_speed():
 c=AdaptationCore();c.auto=True;s=states();c.event('rocky')
 for _ in range(2000):
  o=c.step(.02,[.2,0,0],s)
  for i,n in enumerate(STAGES+ARMS):s[n]=o.shape[i]
  for i,n in enumerate(LEGS):s[n+'_steer_joint']=o.steering[i]
  if o.status['transition_pending']:assert o.status['command_speed']<=.15
 assert s[STAGES[0]]==pytest.approx(D_MIN,abs=.0005)
 assert max(o.wheels)>0

@pytest.mark.parametrize('radius',[.063,.075,.10])
def test_generated_radius_diagnostics_and_native_settings_agree(tmp_path,radius):
 from morph_sim.assets import prepare_robot
 import json,shutil
 if not shutil.which('gz'):pytest.skip('Gazebo SDF converter unavailable')
 urdf,sdf=prepare_robot(P,tmp_path,wheel_radius=radius)
 diagnostics=yaml.safe_load((tmp_path/'params.yaml').read_text())
 robot=ET.fromstring(urdf)
 assert diagnostics['robot']['wheel_radius_m']==radius
 for name in LEGS:
  cylinder=robot.find("link[@name='"+name+"_wheel']/collision/geometry/cylinder")
  assert float(cylinder.get('radius'))==radius
 plugin=robot.find("./gazebo/plugin[@name='morph_sim::SuspensionSystem']")
 for tag,key in [('stiffness','stiffness_N_m'),('damping','damping_Ns_m'),('seal_friction','seal_friction_N'),('bump_stiffness','bump_stiffness_N_m')]:
  assert float(plugin.findtext(tag))==diagnostics['suspension'][key]
 manifest=json.loads((tmp_path/'resolved_manifest.json').read_text())
 assert manifest['parameters']==diagnostics
 assert len(manifest['hashes'])==7

@pytest.mark.parametrize('radius',[float('nan'),float('inf'),.001,.2])
def test_invalid_radius_rejected_before_generation(tmp_path,radius):
 from morph_sim.assets import prepare_robot
 with pytest.raises(ValueError,match='wheel_radius'):prepare_robot(P,tmp_path,wheel_radius=radius)
