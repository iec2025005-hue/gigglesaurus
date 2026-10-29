from pathlib import Path
import copy
import xml.etree.ElementTree as ET
import pytest
import yaml
from morph_sim.parameters import resolve_schema, resolve_profile
from morph_sim.surface_parameters import synchronize_world
P=Path(__file__).parents[1]


def test_schema_has_units_and_explicit_unknown_uncertainty():
    params,metadata=resolve_profile(P)
    assert len(metadata)>200
    assert all(v['units']!='unspecified' for v in metadata.values())
    assert all(v['status']=='assumed' and v['uncertainty'] is None for v in metadata.values())
    assert params==yaml.safe_load((P/'config/params.yaml').read_text())


def test_conflicting_compatibility_file_fails(tmp_path):
    (tmp_path/'config').mkdir()
    for file in ('params.yaml','parameter_schema.yaml'):
        (tmp_path/'config'/file).write_bytes((P/'config'/file).read_bytes())
    bad=yaml.safe_load((tmp_path/'config/params.yaml').read_text())
    bad['robot']['wheel_radius_m']=.063
    (tmp_path/'config/params.yaml').write_text(yaml.safe_dump(bad))
    with pytest.raises(ValueError,match='conflicts'):resolve_profile(tmp_path)


@pytest.mark.parametrize('key,value',[('stiffness_N_m',4500),('force_cap_N',151),('seal_velocity_scale_m_s',0),('thermal_capacity_J_K',0)])
def test_schema_rejects_unsafe_settings(tmp_path,key,value):
    data=yaml.safe_load((P/'config/parameter_schema.yaml').read_text())
    data['parameters']['suspension'][key]['default']=value
    file=tmp_path/'schema.yaml';file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):resolve_schema(file)


@pytest.mark.parametrize('wetness',[0.,1.])
def test_every_course_collision_agrees_with_table(wetness):
    params,_=resolve_profile(P);tree=ET.parse(P/'worlds/multisector.sdf')
    records=synchronize_world(tree,P,params,wetness)
    assert len(records)==len(tree.findall('.//collision'))
    rocks=[r for r in records if r['collision']=='rocks_collision']
    assert rocks[0]['surface']=='wet_rock'
    for record in records:
        profile=params['surfaces'][record['surface']]
        assert record['mu']==pytest.approx((1-wetness)*profile['mu_kinetic_x']+wetness*profile['wet_mu_x'])
        assert record['mu2']==pytest.approx((1-wetness)*profile['mu_kinetic_y']+wetness*profile['wet_mu_y'])


def test_launch_supervisor_preserves_guard_and_cleans_children(tmp_path):
    import subprocess,sys,signal,time
    supervisor=P/'tools/launch_supervisor.py'
    ready=tmp_path/'ready'
    script=tmp_path/'child.py'
    script.write_text('import signal,time,pathlib\nsignal.signal(signal.SIGINT,lambda *_:exit(0))\npathlib.Path('+repr(str(ready))+').touch()\nwhile True:time.sleep(.1)\n')
    command=[sys.executable,str(supervisor),str(tmp_path/'lock'),sys.executable,str(script)]
    child=subprocess.Popen(command)
    try:
        deadline=time.monotonic()+5
        while not ready.exists() and time.monotonic()<deadline:time.sleep(.02)
        assert ready.exists()
        assert subprocess.run(command,timeout=3,capture_output=True).returncode==75
        child.send_signal(signal.SIGINT)
        assert child.wait(timeout=5)==0
        assert subprocess.run([sys.executable,str(supervisor),str(tmp_path/'lock'),sys.executable,'-c','pass'],timeout=3).returncode==0
    finally:
        if child.poll() is None:child.kill();child.wait()


def test_local_resource_validation_rejects_missing_or_remote(tmp_path):
    from morph_sim.resources import validate_local_resources
    for uri in ('model://missing_mesh/model.sdf','https://example.com/terrain.obj'):
        tree=ET.fromstring('<sdf><uri>'+uri+'</uri></sdf>')
        with pytest.raises(ValueError):validate_local_resources(tree,tmp_path)


def test_world_resources_resolve_locally():
    from morph_sim.resources import validate_local_resources
    assert len(validate_local_resources(ET.parse(P/'worlds/multisector.sdf'),P))>=100


@pytest.mark.parametrize('section,key,value',[('robot','mass_kg',0),('power','soc_initial',1.1),('drive','gear_efficiency',-1),('power','ocv_full_V',8),('drive','output_torque_limit_Nm',.6)])
def test_physical_schema_rejects_conflicts(tmp_path,section,key,value):
    data=yaml.safe_load((P/'config/parameter_schema.yaml').read_text())
    data['parameters'][section][key]['default']=value
    file=tmp_path/'schema.yaml';file.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):resolve_schema(file)


@pytest.mark.parametrize('width',[320,480,640])
def test_fixed_camera_mount_survives_sdf_joint_reduction(tmp_path,width):
    import xml.etree.ElementTree as ET
    import math
    from morph_sim.assets import prepare_robot
    share=Path(__file__).parents[1]
    prepare_robot(share,tmp_path,physics_profile=True,camera_width=width,camera_rate=15,camera_enabled=True)
    root=ET.parse(tmp_path/'morph.sdf').getroot()
    camera=root.find("./model/link/sensor[@type='camera']")
    assert camera is not None
    assert int(camera.findtext('camera/image/width'))==width
    assert int(camera.findtext('camera/image/height'))==int(width*9/16)
    assert float(camera.findtext('update_rate'))==15
    assert camera.find('pose').get('relative_to')=='camera_link'
    frame=root.find("./model/frame[@name='camera_link']")
    assert frame is not None
    assert camera.findtext('camera/camera_info_topic')=='/morph/camera/camera_info'


def test_generated_trajectory_controllers_retain_position_interfaces_and_joint_limits(tmp_path):
    from morph_sim.assets import prepare_robot
    prepare_robot(P,tmp_path,physics_profile=True,position_backend='trajectory')
    config=yaml.safe_load((tmp_path/'controllers.yaml').read_text())
    for name in ('steering_controller','height_controller','shape_controller'):
        assert config['controller_manager']['ros__parameters'][name]['type']=='joint_trajectory_controller/JointTrajectoryController'
        assert config[name]['ros__parameters']['command_interfaces']==['position']
        assert config[name]['ros__parameters']['interpolate_from_desired_state']
        assert not config[name]['ros__parameters']['set_last_command_interface_value_as_state_on_activation']
        assert config[name]['ros__parameters']['cmd_timeout']==0.
    native=ET.parse(tmp_path/'morph.sdf').getroot().find("./model/plugin[@name='morph_sim::DriveWatchdog']")
    assert float(native.findtext('timeout'))<=.2
    root=ET.parse(tmp_path/'morph.urdf').getroot()
    assert float(root.find("joint[@name='telR1_joint']/limit").get('velocity'))==.006
    assert float(root.find("joint[@name='fl_steer_joint']/limit").get('velocity'))==1.
    assert float(root.find("joint[@name='fl_height_joint']/limit").get('effort'))==100.


def test_unsupported_position_backend_fails_before_generation(tmp_path):
    from morph_sim.assets import prepare_robot
    with pytest.raises(ValueError):prepare_robot(P,tmp_path,position_backend='unknown')
    with pytest.raises(ValueError):prepare_robot(P,tmp_path,position_backend='trajectory',physics_profile=False)


def test_auto_backend_preserves_legacy_profile_and_validates_explicit_requests():
    from morph_sim.assets import resolve_position_backend
    assert resolve_position_backend(True)=='trajectory'
    assert resolve_position_backend(False)=='forward'
    assert resolve_position_backend(True,'forward')=='forward'
    with pytest.raises(ValueError):resolve_position_backend(False,'trajectory')
    with pytest.raises(ValueError):resolve_position_backend(True,'typo')
