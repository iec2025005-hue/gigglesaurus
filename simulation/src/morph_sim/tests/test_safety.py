import math
import pytest
from morph_sim.safety import ACTUATED_JOINTS,valid_joint_sample,fresh_snapshot
from morph_sim.control import AdaptationCore


def test_complete_fresh_sample_and_wall_watchdog():
    assert valid_joint_sample(ACTUATED_JOINTS,[0.]*28,[0.]*28,1.,1.01)
    assert fresh_snapshot(1.,100.,1.01,100.01)
    assert not fresh_snapshot(1.,100.,1.01,100.3)
    assert not fresh_snapshot(1.,100.,1.3,100.01)


@pytest.mark.parametrize('missing',ACTUATED_JOINTS)
def test_every_actuated_axis_requires_measurement(missing):
    names=[n for n in ACTUATED_JOINTS if n!=missing]
    assert not valid_joint_sample(names,[0.]*27,[0.]*27,1.,1.)
    result=AdaptationCore().step(.01,[.2,.2,.2],{n:0. for n in names})
    assert not result.status['feedback_ready']
    assert result.wheels==[0.]*4


@pytest.mark.parametrize('stamp',[float('nan'),float('inf'),.5,1.1])
def test_bad_or_stale_timestamps_rejected(stamp):
    assert not valid_joint_sample(ACTUATED_JOINTS,[0.]*28,[0.]*28,stamp,1.)


def test_partial_velocity_and_duplicate_names_rejected():
    assert not valid_joint_sample(ACTUATED_JOINTS,[0.]*28,[0.]*27,1.,1.)
    assert not valid_joint_sample(ACTUATED_JOINTS+ACTUATED_JOINTS[:1],[0.]*29,[0.]*29,1.,1.)


@pytest.mark.parametrize('field,value',[('source_sim_ns',0),('source_sim_ns',2000000000),('source_wall_ns',0),('sequence',-1),('values',[float('nan'),0,0])])
def test_timed_command_rejects_bad_origins(field,value):
    import json,socket
    from morph_sim.timing import decode_command
    data=dict(values=[.2,0,0],source_sim_ns=1000000000,source_wall_ns=1000000000,sequence=1,source_id='test',host=socket.gethostname())
    data[field]=value
    with pytest.raises(ValueError):decode_command(json.dumps(data),1000000000,1000000000)


def test_timed_command_preserves_both_clocks_and_source_sequence():
    import json,socket
    from morph_sim.timing import decode_command
    data=dict(values=[.2,0,.1],source_sim_ns=1000000000,source_wall_ns=100000000000,sequence=5,source_id='test',host=socket.gethostname())
    assert decode_command(json.dumps(data),1010000000,100010000000)==data


def test_step_and_reversal_position_trajectories_respect_existing_limits(tmp_path):
    from pathlib import Path
    from morph_sim.assets import prepare_robot
    from morph_sim.trajectory_limits import PositionTrajectoryLimiter
    share=Path(__file__).parents[1]
    prepare_robot(share,tmp_path,physics_profile=True)
    limiter=PositionTrajectoryLimiter(tmp_path/'morph.urdf',tmp_path/'controllers.yaml')
    measured={name:0. for name in limiter.limits}
    for direction in (1,-1):
        requested={name:high if direction>0 else low for name,(low,high,_) in limiter.limits.items()}
        for i in range(300):
            result,trace=limiter.apply(requested,measured,.012 if i%4==0 else .01)
            assert all(abs(row['command_velocity'])<=.951*row['velocity_limit'] for row in trace)
            assert not any(row['lead_conflict'] for row in trace)
            for name,value in result.items():
                low,high,velocity=limiter.limits[name]
                assert low<=value<=high
                # The installed native position interface converts lead to velocity.
                requested_speed=limiter.servo_rate*(value-measured[name])
                assert abs(requested_speed)<=.901*velocity
                measured[name]+=requested_speed*.01


def test_powered_hold_captures_only_after_braking_and_releases_for_motion():
    from morph_sim.safety import StopPositionHold
    hold=StopPositionHold(.2,4.,.5)
    assert hold.apply([0.]*4,[1.]*4,[2.]*4)==[0.]*4
    assert hold.positions==[None]*4
    assert hold.apply([0.]*4,[2.]*4,[.1]*4)==[0.]*4
    assert hold.apply([0.]*4,[2.1]*4,[.1]*4)==pytest.approx([-.4]*4)
    assert hold.apply([0.]*4,[3.]*4,[.1]*4)==[-.5]*4
    assert hold.apply([2.]*4,[3.]*4,[.1]*4)==[2.]*4
    assert hold.positions==[None]*4
    hold.apply([0.]*4,[3.]*4,[0.]*4)
    hold.apply([0.]*4,[3.]*4,[0.]*4,False)
    assert hold.positions==[None]*4
