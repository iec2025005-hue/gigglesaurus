"""Jazzy/Harmonic launch: spawn -> controller activation -> adaptation + teleop."""
from pathlib import Path
import os,tempfile,json,hashlib,subprocess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, EmitEvent, LogError
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from morph_sim.assets import prepare_robot,resolve_position_backend
from morph_sim.course import Course

def setup(context):
 share=Path(get_package_share_directory('morph_sim'));prefix=Path(get_package_prefix('morph_sim'))
 headless=LaunchConfiguration('headless').perform(context).lower()=='true';teleop=LaunchConfiguration('teleop').perform(context).lower()=='true'
 controls=LaunchConfiguration('controls').perform(context).lower()
 if controls not in ('console','gazebo','pygame','terminal','none'):raise RuntimeError('controls must be console, gazebo, pygame, terminal, or none')
 rviz_enabled=LaunchConfiguration('rviz').perform(context).lower()=='true'
 demo_enabled=LaunchConfiguration('demo').perform(context).lower()=='true'
 if demo_enabled and controls!='none':raise RuntimeError('Use controls:=none when demo:=true so only one source commands /cmd_vel')
 from morph_sim.parameters import resolve_profile
 defaults,_=resolve_profile(share)
 radius_arg=LaunchConfiguration('wheel_radius').perform(context)
 radius=defaults['robot']['wheel_radius_m'] if radius_arg=='auto' else float(radius_arg)
 if not .02<=radius<=.15:raise RuntimeError('wheel_radius must be in [0.02, 0.15] m')
 profile=LaunchConfiguration('physics_profile').perform(context).lower()=='true'
 position_backend=resolve_position_backend(profile,LaunchConfiguration('position_backend').perform(context))
 range_sensor=LaunchConfiguration('range_sensor').perform(context).lower()=='true'
 perception_auto=LaunchConfiguration('perception_auto').perform(context).lower()=='true'
 if perception_auto and not range_sensor:raise RuntimeError('perception_auto requires range_sensor:=true')
 camera_rate=float(LaunchConfiguration('camera_rate').perform(context));camera_width=int(LaunchConfiguration('camera_width').perform(context))
 if not 1<=camera_rate<=30 or camera_width not in (320,480,640):raise RuntimeError('camera_rate must be 1–30 Hz and camera_width 320, 480 or 640')
 effects=LaunchConfiguration('camera_effects').perform(context).lower()=='true'
 (Path.cwd()/'log').mkdir(parents=True,exist_ok=True)
 generated=Path(tempfile.mkdtemp(prefix='run_',dir=str(Path.cwd()/'log')));urdf,sdf=prepare_robot(share,generated,radius,profile,camera_rate,camera_width,range_sensor,trace_native=LaunchConfiguration('trace_positions').perform(context).lower()=='true',camera_enabled=LaunchConfiguration('camera_enabled').perform(context).lower()=='true',position_backend=position_backend)
 controller_file=generated/'controllers.yaml' if profile else share/'config/controllers.yaml'
 resources=os.pathsep.join([str(share),str(share/'models'),str(share.parent),os.environ.get('GZ_SIM_RESOURCE_PATH','')]);plugins=str(prefix/'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
 arena=LaunchConfiguration('arena').perform(context).lower()
 if arena not in ('multisector','adaptive','competition','warehouse'):raise RuntimeError('arena must be multisector, adaptive, warehouse or competition')
 auto_terrain=LaunchConfiguration('automatic_terrain').perform(context).lower()=='true'
 if auto_terrain and perception_auto:raise RuntimeError('Choose perception_auto or automatic_terrain; both cannot own chassis targets')
 world=str(share/'worlds'/({'multisector':'multisector.sdf','adaptive':'adaptive_arena.sdf','warehouse':'warehouse.sdf','competition':'competition.sdf'}[arena]))
 import xml.etree.ElementTree as ET
 step=float(LaunchConfiguration('physics_step').perform(context))
 if step not in (.001,.002):raise RuntimeError('physics_step must be 0.001 or 0.002 seconds')
 tree=ET.parse(world)
 resolved=json.loads((generated/'resolved_manifest.json').read_text())
 scenario=LaunchConfiguration('scenario').perform(context)
 if scenario not in resolved['parameters']['environment']['scenarios']:raise RuntimeError('Unknown scenario: '+scenario)
 if arena=='multisector':
  from morph_sim.surface_parameters import synchronize_world
  resolved['native_surface_assignments']=synchronize_world(tree,share,resolved['parameters'],resolved['parameters']['environment']['scenarios'][scenario]['wetness'])
 from morph_sim.resources import validate_local_resources
 resolved['validated_local_world_resources']=validate_local_resources(tree,share)
 resolved['validated_local_robot_resources']=validate_local_resources(ET.parse(sdf),share)
 resolved['native_friction_scope']='Coulomb mu/mu2 from kinetic table; static/kinetic transition and soil deformation not modeled'
 git_head=subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True,check=False)
 git_status=subprocess.run(['git','status','--porcelain'],capture_output=True,text=True,check=False)
 for library in ('libmorph_suspension.so','libmorph_watchdog.so','libmorph_height_servo.so','libFrameMeter.so'):
  path=prefix/'lib'/library
  if path.exists():resolved.setdefault('native_binary_hashes',{})[library]=hashlib.sha256(path.read_bytes()).hexdigest()
 bridge_binary=prefix/'lib/morph_sim/lifecycle_bridge'
 if bridge_binary.exists():resolved.setdefault('native_binary_hashes',{})['lifecycle_bridge']=hashlib.sha256(bridge_binary.read_bytes()).hexdigest()
 resolved['git']={'commit':git_head.stdout.strip() if git_head.returncode==0 else None,'dirty':bool(git_status.stdout.strip())}
 resolved['render_environment']={name:os.environ.get(name) for name in ('LP_NUM_THREADS','LIBGL_ALWAYS_SOFTWARE','GZ_PARTITION')}
 resolved['position_backend_resolved']=position_backend
 resolved['launch']={k:LaunchConfiguration(k).perform(context) for k in ('arena','sector','scenario','position_backend','physics_step','headless','rviz','camera_enabled','camera_rate','camera_width','render_shadows','range_sensor')}
 tree.find('./world/physics/max_step_size').text=str(step)
 if LaunchConfiguration('render_shadows').perform(context).lower()=='false':
  shadow=tree.find('./world/scene/shadows')
  if shadow is not None:shadow.text='false'
  for light in tree.findall('./world/light'):
   element=light.find('cast_shadows')
   if element is not None:element.text='false'
 gui=tree.find('./world/gui')
 if gui is not None:
  meter=ET.SubElement(gui,'plugin',{'filename':'FrameMeter','name':'FrameMeter'})
  settings=ET.SubElement(meter,'gz-gui')
  for key,value,kind in [('showTitleBar','false','bool'),('state','floating','string'),('width','1','double'),('height','1','double')]:ET.SubElement(settings,'property',{'key':key,'type':kind}).text=value
 generated_world=generated/'world.sdf';tree.write(generated_world,encoding='utf-8',xml_declaration=True);world=str(generated_world)
 for directory,dirs,files in os.walk(share,followlinks=True):
  dirs[:]=[d for d in dirs if d!='__pycache__']
  for filename in files:
   source=Path(directory)/filename
   resolved['hashes'][str(source.resolve())]=hashlib.sha256(source.read_bytes()).hexdigest()
 import morph_sim.parameters as parameter_module
 for source in Path(parameter_module.__file__).resolve().parent.glob('*.py'):
  resolved['hashes'][str(source)]=hashlib.sha256(source.read_bytes()).hexdigest()
 resolved['hashes'][world]=hashlib.sha256(Path(world).read_bytes()).hexdigest()
 for name in ('gz_sim_vendor','gz_physics_vendor','gz_transport_vendor','gz_ros2_control','controller_manager'):
  result=subprocess.run(['dpkg-query','-W','-f=${Version}',name if name.startswith('gz-') else 'ros-'+os.environ.get('ROS_DISTRO','lyrical')+'-'+name.replace('_','-')],capture_output=True,text=True,check=False)
  resolved.setdefault('package_versions',{})[name]=result.stdout.strip() if result.returncode==0 else 'not resolved by dpkg query'
 (generated/'resolved_manifest.json').write_text(json.dumps(resolved,indent=2))
 gazebo_gui=LaunchConfiguration('gazebo_gui').perform(context).lower()=='true'
 gz_args=('-r -s ' if headless or not gazebo_gui else '-r ')+world
 gz=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),launch_arguments={'gz_args':gz_args,'on_exit_shutdown':'true'}.items())
 rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':urdf,'use_sim_time':True}],output='screen')
 bridge=Node(package='morph_sim',executable='lifecycle_bridge',parameters=[{'config_file':str(share/'config'/('bridge.yaml' if arena!='competition' else 'bridge_competition.yaml')),'use_sim_time':True}],output='screen')
 spawn_xyz=Course(share).spawn(LaunchConfiguration('sector').perform(context)) if arena=='multisector' else [-4,10,1.7] if arena=='adaptive' else [0,0 if arena=='warehouse' else -9,.03+.26+radius+.04]
 spawn=Node(package='ros_gz_sim',executable='create',arguments=['-world',('leo_marsyard' if arena!='competition' else 'morph_arena'),'-name','morph','-file',str(sdf),'-x',str(spawn_xyz[0]),'-y',str(spawn_xyz[1]),'-z',str(spawn_xyz[2]),'-Y',('1.570796' if arena=='multisector' and LaunchConfiguration('sector').perform(context)!='start' else '0')],output='screen')
 controllers=Node(package='controller_manager',executable='spawner',arguments=['joint_state_broadcaster','drive_controller','steering_controller','height_controller','shape_controller','--param-file',str(controller_file),'--controller-manager','/controller_manager','--controller-manager-timeout','120','--switch-timeout','60','--service-call-timeout','90','--activate-as-group'],output='screen')
 manager=Node(package='morph_sim',executable='terrain_manager',parameters=[{'use_sim_time':True,'params_file':str(generated/'params.yaml'),'force_height_servo':profile,'position_backend':position_backend,'robot_urdf_file':str(generated/'morph.urdf'),'controllers_file':str(controller_file),'position_filter':LaunchConfiguration('position_filter').perform(context).lower()=='true','trace_positions':LaunchConfiguration('trace_positions').perform(context).lower()=='true','wheel_radius':radius,'automatic_terrain':auto_terrain,'arena':arena,'perception_auto':perception_auto}],remappings=[('/drive_controller/commands','/morph/actuator/drive_target')] if profile else [],output='screen')
 interactions=Node(package='morph_sim',executable='interaction_runtime',parameters=[{'use_sim_time':True,'resolved_manifest_file':str(generated/'resolved_manifest.json'),'params_file':str(generated/'params.yaml'),'scenario':LaunchConfiguration('scenario').perform(context),'log_dir':str(Path.cwd()/'log/interactions'),'actuate':profile,'arena':arena}],remappings=[('/morph/actuator/steer_target','/steering_controller/commands')]+([] if profile else [('/morph/actuator/drive_target','/drive_controller/commands')]),output='screen')
 camera_effects=Node(package='morph_sim',executable='camera_environment',parameters=[{'use_sim_time':True,'params_file':str(generated/'params.yaml')}],output='screen')
 ui=Node(package='morph_sim',executable=('control_console' if controls=='console' else 'teleop'),parameters=[{'use_sim_time':True}],output='screen')
 terminal=Node(package='teleop_twist_keyboard',executable='teleop_twist_keyboard',prefix='xterm -e',output='screen')
 keyboard=Node(package='morph_sim',executable='gazebo_keyboard',output='screen')
 demo=Node(package='morph_sim',executable='demo_sequence',parameters=[{'use_sim_time':True}],output='screen')
 observer=Node(package='morph_sim',executable='perception_adapter',parameters=[{'use_sim_time':True,'wheel_radius':radius}],output='screen')
 visualization=Node(package='morph_sim',executable='rviz_state',parameters=[{'use_sim_time':True,'arena':arena}],output='screen')
 camera_gate=Node(package='morph_sim',executable='camera_gate',output='screen')
 rviz=Node(package='rviz2',executable='rviz2',arguments=['-d',str(share/'config/morph.rviz')],parameters=[{'use_sim_time':True}],output='screen')
 def after_spawn(event,ctx):
  if event.returncode!=0:return [LogError(msg='Robot spawn failed. Controllers not started.'),EmitEvent(event=Shutdown(reason='spawn failure'))]
  return [controllers]
 def after_controllers(event,ctx):
  if event.returncode!=0:return [LogError(msg='Controller activation failed.'),EmitEvent(event=Shutdown(reason='controller failure'))]
  return [manager,camera_gate,interactions,visualization]+([observer] if range_sensor else [])+([camera_effects] if effects else [])+([keyboard] if teleop and controls=='gazebo' and not headless else [])+([ui] if teleop and controls in ('console','pygame') and not headless else [])+([terminal] if teleop and controls=='terminal' and not headless else [])+([demo] if demo_enabled else [])+([rviz] if rviz_enabled and not headless else [])
 return [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',resources),SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',plugins),SetEnvironmentVariable('GZ_GUI_PLUGIN_PATH',str(prefix/'lib')+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')),rsp,bridge,gz,RegisterEventHandler(OnProcessExit(target_action=spawn,on_exit=after_spawn)),RegisterEventHandler(OnProcessExit(target_action=controllers,on_exit=after_controllers)),spawn]

def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('position_backend',default_value='auto'),DeclareLaunchArgument('position_filter',default_value='true'),DeclareLaunchArgument('trace_positions',default_value='false'),DeclareLaunchArgument('gazebo_gui',default_value='true'),DeclareLaunchArgument('range_sensor',default_value='true'),DeclareLaunchArgument('perception_auto',default_value='false'),DeclareLaunchArgument('render_shadows',default_value='true'),DeclareLaunchArgument('camera_enabled',default_value='false'),DeclareLaunchArgument('camera_rate',default_value='10'),DeclareLaunchArgument('camera_width',default_value='480'),DeclareLaunchArgument('camera_effects',default_value='false'),DeclareLaunchArgument('physics_step',default_value='0.002'),DeclareLaunchArgument('physics_profile',default_value='true'),DeclareLaunchArgument('scenario',default_value='indoor'),DeclareLaunchArgument('headless',default_value='false'),DeclareLaunchArgument('teleop',default_value='true'),DeclareLaunchArgument('controls',default_value='console'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('demo',default_value='false'),DeclareLaunchArgument('arena',default_value='multisector'),DeclareLaunchArgument('sector',default_value='start'),DeclareLaunchArgument('automatic_terrain',default_value='false'),DeclareLaunchArgument('wheel_radius',default_value='auto'),OpaqueFunction(function=setup)])
