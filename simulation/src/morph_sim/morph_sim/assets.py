"""Expand XACRO, convert using installed libsdformat, then add native SDF sensors/PBR."""
from pathlib import Path
import os, subprocess, xml.etree.ElementTree as ET
import hashlib, json, math, platform
import yaml
import xacro

def child(parent, tag, text=None, **attrs):
 node=ET.SubElement(parent,tag,attrs)
 if text is not None:node.text=str(text)
 return node

def add_surface(collision, mu):
 surface=collision.find('surface')
 if surface is None:surface=child(collision,'surface')
 friction=surface.find('friction')
 if friction is None:friction=child(surface,'friction')
 ode=friction.find('ode')
 if ode is None:ode=child(friction,'ode')
 for key in ('mu','mu2'):
  node=ode.find(key)
  if node is None:node=child(ode,key)
  node.text=str(mu)

def enrich_sdf(root, share, camera_rate=10., camera_width=480, range_sensor=True, camera_enabled=False):
 model=root.find('model')
 if model is None:raise ValueError('gz sdf conversion returned no model')
 links={link.attrib['name']:link for link in model.findall('link')}
 if 'base_link' not in links:raise ValueError('base_link missing after SDF conversion')
 for name,link in links.items():
  for visual in link.findall('visual'):
   material=visual.find('material')
   if material is None:material=child(visual,'material')
   old=material.find('pbr')
   if old is not None:material.remove(old)
   vname=visual.attrib.get('name','')
   # Preserve CAD/component finishes. Textures must not multiply tinted CAD
   # colours by another dark albedo, which previously hid the wheel assembly.
   rgba=material.findtext('diffuse') or '0.45 0.49 0.54 1'
   for field in ('ambient','diffuse'):
    item=material.find(field)
    if item is None:item=child(material,field)
    item.text=rgba
   metal=child(child(material,'pbr'),'metal')
   rubber='tire' in vname
   tex='rubber' if rubber or 'coil' in vname else 'encoder' if 'encoder' in vname else 'gold' if 'damper' in vname else 'chrome' if 'piston' in vname else 'cyan' if 'hexarm' in name else 'metal'
   material.find('ambient').text='1 1 1 1'
   material.find('diffuse').text='1 1 1 1'
   child(metal,'albedo_map',(share/'models/materials/textures'/f'finish_{tex}.png').as_uri())

   child(metal,'metalness',0.0 if rubber or 'encoder' in vname else .75)
   child(metal,'roughness',.86 if rubber else .22 if 'piston' in vname else .42)
   if 'status_led' in vname:child(material,'emissive','0.02 0.8 1.0 1')
  for collision in link.findall('collision'):add_surface(collision,1.2 if name.endswith('_wheel') else .6)
  # No global self-collision: adjacent simplified collision shapes deliberately overlap.
  sc=link.find('self_collide')
  if sc is None:sc=child(link,'self_collide')
  sc.text='false'
 odom=child(model,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher')
 child(odom,'odom_frame','world');child(odom,'robot_base_frame','base_link');child(odom,'odom_publish_frequency',50);child(odom,'dimensions',3);child(odom,'odom_topic','/morph/ground_truth/odom');child(odom,'tf_topic','/morph/ground_truth/tf')
 base=links['base_link'];imu=child(base,'sensor',name='base_imu',type='imu');child(imu,'topic','/morph/imu');child(imu,'always_on','true');child(imu,'update_rate',100);child(imu,'gz_frame_id','base_link')
 camera_link=links.get('camera_link')
 camera_frame=model.find("frame[@name='camera_link']")
 if camera_link is None and camera_frame is None:raise ValueError('Camera mount frame disappeared during SDF conversion')
 if camera_link is None:camera_link=base
 if camera_enabled and camera_link is not None:
  lamp=child(camera_link,'light',name='camera_fill',type='point')
  child(lamp,'pose','0 0 0.01 0 0 0',**({'relative_to':'camera_link'} if camera_link is base else {}));child(lamp,'diffuse','0.08 0.65 0.9 1');child(lamp,'specular','0.03 0.2 0.3 1');child(lamp,'cast_shadows','false')
  attenuation=child(lamp,'attenuation');child(attenuation,'range',0.8);child(attenuation,'constant',.6);child(attenuation,'linear',.8);child(attenuation,'quadratic',1.5)
  camera=child(camera_link,'sensor',name='front_camera',type='camera')
  child(camera,'pose','0 0 0 0 0 0',**({'relative_to':'camera_link'} if camera_link is base else {}))
  child(camera,'topic','/morph/camera/image');child(camera,'always_on','true');child(camera,'update_rate',camera_rate);child(camera,'gz_frame_id','camera_link')
  optics=child(camera,'camera');child(optics,'horizontal_fov',1.15)
  image=child(optics,'image');child(image,'width',camera_width);child(image,'height',int(camera_width*9/16));child(image,'format','R8G8B8')
  clip=child(optics,'clip');child(clip,'near',0.05);child(clip,'far',25)
  child(optics,'camera_info_topic','/morph/camera/camera_info')
 if range_sensor:
  lidar=child(base,'sensor',name='terrain_lidar',type='gpu_lidar');child(lidar,'pose','0.22 0 0.12 0 0 0');child(lidar,'topic','/morph/range');child(lidar,'gz_frame_id','perception_link');child(lidar,'always_on','true');child(lidar,'update_rate',10)
  ray=child(lidar,'lidar');scan=child(ray,'scan')
  for axis,samples,lo,hi in [('horizontal',96,-1.1,1.1),('vertical',12,-1.15,.25)]:
   el=child(scan,axis);child(el,'samples',samples);child(el,'resolution',1);child(el,'min_angle',lo);child(el,'max_angle',hi)
  distance=child(ray,'range');child(distance,'min',.12);child(distance,'max',5);child(distance,'resolution',.005)
  noise=child(ray,'noise',type='gaussian');child(noise,'mean',0);child(noise,'stddev',.003)
 data=child(imu,'imu')
 for field,std in [('angular_velocity',.002),('linear_acceleration',.015)]:
  axes=child(data,field)
  for axis in ('x','y','z'):
   noise=child(child(axes,axis),'noise',type='gaussian');child(noise,'mean',0);child(noise,'stddev',std)
 for name in ('fl','fr','rl','rr','underbody'):
  link=base if name=='underbody' else links[name+'_wheel']
  collisions=link.findall('collision')
  if not collisions:raise ValueError('Contact sensor without collision: '+name)
  if name=='underbody':
   coll=next((c for c in collisions if 'underbody' in c.attrib['name']),collisions[0])
  else:coll=collisions[0]
  sensor=child(link,'sensor',name=name+'_contact',type='contact');child(sensor,'topic','/morph/contacts/'+name);child(sensor,'always_on','true');child(sensor,'update_rate',100)
  contact=child(sensor,'contact');child(contact,'collision',coll.attrib['name'])
 # Make resource URIs independent of the process working directory.
 for node in model.iter():
  if node.text and node.text.startswith('package://morph_sim/'):
   node.text=(share/node.text[len('package://morph_sim/'):]).as_uri()
 return root

def resolve_position_backend(physics_profile,requested='auto'):
 if requested=='auto':return 'trajectory' if physics_profile else 'forward'
 if requested not in ('trajectory','forward') or (requested=='trajectory' and not physics_profile):raise ValueError('Position backend conflicts with physics profile')
 return requested


def prepare_robot(share, out_dir, wheel_radius=None, physics_profile=False, camera_rate=10., camera_width=480, range_sensor=True, trace_native=False, camera_enabled=False, position_backend='forward'):
 if position_backend not in ('forward','trajectory'):raise ValueError('Unknown position backend')
 if position_backend=='trajectory' and not physics_profile:raise ValueError('Trajectory backend currently requires physics_profile true')
 share=Path(share).resolve();out_dir=Path(out_dir);out_dir.mkdir(parents=True,exist_ok=True)
 from .parameters import resolve_profile
 params,metadata=resolve_profile(share,wheel_radius)
 wheel_radius=params['robot']['wheel_radius_m']
 if not math.isfinite(wheel_radius) or not .02 <= wheel_radius <= .15:
  raise ValueError('wheel_radius must be finite and in [0.02, 0.15] m')
 params['robot']['wheel_radius_m']=wheel_radius
 suspension=params['suspension']
 required=('stiffness_N_m','damping_Ns_m','thermal_capacity_J_K','thermal_resistance_K_W','temperature_coefficient_per_K','seal_friction_N','bump_stiffness_N_m')
 for key in required:
  value=suspension[key]
  if not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
   raise ValueError('Invalid suspension parameter: '+key)
 (out_dir/'params.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
 doc=xacro.process_file(str(share/'urdf/morph.urdf.xacro'),mappings={'controllers_file':str(share/'config/controllers.yaml'),'wheel_radius':str(wheel_radius),'wheel_width':str(params['robot']['wheel_width_m'])})
 tree=ET.fromstring(doc.toxml())
 plugin=tree.find("./gazebo/plugin[@name='morph_sim::SuspensionSystem']")
 if plugin is None:raise ValueError('Missing suspension plugin')
 mapping={'stiffness':'stiffness_N_m','damping':'damping_Ns_m','thermal_capacity':'thermal_capacity_J_K','thermal_resistance':'thermal_resistance_K_W','temperature_coefficient':'temperature_coefficient_per_K','seal_friction':'seal_friction_N','bump_stiffness':'bump_stiffness_N_m','force_cap':'force_cap_N','bump_lower':'bump_lower_m','bump_upper':'bump_upper_m','seal_velocity_scale':'seal_velocity_scale_m_s','adjustment_time':'adjustment_time_s','reference_temperature':'reference_temperature_C'}
 for tag,key in mapping.items():
  element=plugin.find(tag)
  if element is None:element=child(plugin,tag)
  element.text=str(suspension[key])
 for joint in tree.findall('joint'):
  limit=joint.find('limit')
  name=joint.get('name','')
  if limit is not None and name.endswith('_wheel_joint'):
   limit.set('effort',str(params['drive']['output_torque_limit_Nm']))
  if limit is not None and name.endswith('_steer_joint'):
   limit.set('effort',str(params['steering']['output_torque_limit_Nm']));limit.set('velocity',str(params['steering']['output_speed_limit_rad_s']))
  if limit is not None and name.endswith('_spring_joint'):
   limit.set('lower',str(suspension['lower_m']));limit.set('upper',str(suspension['upper_m']));limit.set('effort',str(suspension['force_cap_N']))
 for link in tree.findall('link'):
  if link.get('name','').endswith('_wheel') and not math.isclose(float(link.find('inertial/mass').get('value')),params['robot']['wheel_mass_kg'],abs_tol=1e-8):
   raise ValueError('Wheel mass conflicts with generated component inertia')
 mass=sum(float(link.find('inertial/mass').get('value')) for link in tree.findall('link'))
 if not math.isclose(mass,params['robot']['mass_kg'],abs_tol=1e-8):
  raise ValueError('Generated link masses conflict with robot.mass_kg; component mass derivation must be updated')
 urdf=ET.tostring(tree,encoding='unicode')
 if physics_profile:
  config=yaml.safe_load((share/'config/controllers.yaml').read_text())
  for controller in ('drive_controller',):
   config[controller]['ros__parameters']['interface_name']='effort'
  if position_backend=='trajectory':
   # Native 200-ms wall watchdog stops axes using fresh ECM measurements.
   # JTC timeout's snap-to-measured target would bypass the bounded trajectory.
   # Retain its last bounded command; the native driver owns fault stopping.
   for name in ('steering_controller','height_controller','shape_controller'):
    config['controller_manager']['ros__parameters'][name]['type']='joint_trajectory_controller/JointTrajectoryController'
    section=config[name]['ros__parameters'];section.pop('interface_name',None)
    section.update(command_interfaces=['position'],state_interfaces=['position','velocity'],interpolate_from_desired_state=True,set_last_command_interface_value_as_state_on_activation=False,cmd_timeout=0.)
  controller_file=out_dir/'controllers.yaml'
  controller_file.write_text(yaml.safe_dump(config,sort_keys=False))
  tree=ET.fromstring(urdf)
  for joint in tree.findall('./ros2_control/joint'):
   if joint.get('name','').endswith('_wheel_joint'):
    for interface in joint.findall('command_interface'):interface.set('name','effort')
  for element in tree.findall('./gazebo/plugin/parameters'):element.text=str(controller_file)
  urdf=ET.tostring(tree,encoding='unicode')
 urdf_file=out_dir/'morph.urdf';urdf_file.write_text(urdf)
 result=subprocess.run(['gz','sdf','-p',str(urdf_file)],text=True,capture_output=True,timeout=60,check=False)
 if result.returncode:raise RuntimeError('URDF to SDF conversion failed: '+result.stderr)
 try:root=ET.fromstring(result.stdout)
 except ET.ParseError as exc:raise RuntimeError('Invalid SDF conversion output: '+result.stdout[:500]) from exc
 model=root.find('model')
 if model is None or not any('gz_ros2_control' in (p.attrib.get('filename','')+' '+p.attrib.get('name','')) for p in model.findall('plugin')):
  raise RuntimeError('gz_ros2_control plugin was lost during URDF to SDF conversion; robot would spawn but not move')
 root=enrich_sdf(root,share,camera_rate,camera_width,range_sensor,camera_enabled)
 if physics_profile:
  watchdog=child(root.find('model'),'plugin',filename='libmorph_watchdog.so',name='morph_sim::DriveWatchdog')
  child(watchdog,'capture_speed',params['control_safety']['hold_capture_speed_rad_s'])
  child(watchdog,'mode',params['control_safety']['watchdog_mode']);child(watchdog,'hold_kp',params['control_safety']['watchdog_hold_kp_Nm_rad'])
  child(watchdog,'trace','true' if trace_native else 'false')
  child(watchdog,'timeout',params['control_safety']['command_timeout_s']);child(watchdog,'torque_limit',params['drive']['output_torque_limit_Nm'])
  drive=params['drive'];child(watchdog,'brake_gain',drive['kt_Nm_A']*drive['ke_Vs_rad']*drive['gear_ratio']**2*drive['gear_efficiency']/drive['resistance_ohm'])
  for name in ('fl','fr','rl','rr'):child(watchdog,'wheel',name+'_wheel_joint')
  for joint in ET.fromstring(urdf).findall('joint'):
   if joint.get('type') in ('prismatic','revolute') and not joint.get('name','').endswith(('_spring_joint','_height_joint')):child(watchdog,'position_joint',joint.get('name'))
 if physics_profile:
  servo=child(root.find('model'),'plugin',filename='libmorph_height_servo.so',name='morph_sim::HeightServo')
  child(servo,'kp',params['height_servo']['kp_N_m']);child(servo,'kd',params['height_servo']['kd_Ns_m']);child(servo,'timeout',params['control_safety']['state_timeout_s'])
  robot_tree=ET.fromstring(urdf)
  for name in ('fl','fr','rl','rr'):
   axis=child(servo,name);joint=robot_tree.find("joint[@name='"+name+"_height_joint']");limit=joint.find('limit');dynamics=joint.find('dynamics')
   child(axis,'mass',robot_tree.find("link[@name='"+name+"_height']/inertial/mass").get('value'))
   for tag,attribute in [('lower','lower'),('upper','upper'),('force_limit','effort'),('speed_limit','velocity')]:child(axis,tag,limit.get(attribute))
   child(axis,'damping',dynamics.get('damping'));child(axis,'friction',dynamics.get('friction'))
 ET.indent(root);sdf_file=out_dir/'morph.sdf';ET.ElementTree(root).write(sdf_file,encoding='utf-8',xml_declaration=True)
 manifest={'schema_version':1,'parameter_status':'assumed','parameters':params,'metadata':metadata,'overrides':{'robot.wheel_radius_m':wheel_radius},'python_version':platform.python_version(),'ros_distro':os.environ.get('ROS_DISTRO'),'hashes':{}}
 for file in [share/'config/parameter_schema.yaml',share/'config/params.yaml',share/'urdf/morph.urdf.xacro',share/'urdf/leg.xacro',urdf_file,sdf_file,out_dir/'params.yaml']:
  manifest['hashes'][str(file)]=hashlib.sha256(file.read_bytes()).hexdigest()
 (out_dir/'resolved_manifest.json').write_text(json.dumps(manifest,indent=2))
 return urdf,sdf_file
