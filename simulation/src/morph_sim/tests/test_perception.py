import numpy as np
import pytest
from morph_sim.perception_core import assess,DecisionFilter
from morph_sim.design_physics import load_parameters,PowerPlant
from pathlib import Path

def ground(slope=0):
 x,y=np.meshgrid(np.linspace(.4,1.7,32),np.linspace(-.45,.45,24));z=-.43+np.tan(np.radians(slope))*(x-.4)
 return np.column_stack([x.ravel(),y.ravel(),z.ravel()])
def test_smooth_ground_is_normal():
 d=assess(ground());assert d['valid'] and d['shape']=='normal' and abs(d['slope_deg'])<.1
@pytest.mark.parametrize('slope',[12.,-12.])
def test_slope_expands_without_map(slope):
 d=assess(ground(slope));assert d['valid'] and d['shape']=='expand';assert d['slope_deg']==pytest.approx(slope,abs=.1)
def test_rough_ground_compacts():
 p=ground();p[:,2]+=.035*np.sin(p[:,0]*32)*np.cos(p[:,1]*21);d=assess(p);assert d['shape']=='compact'
def test_obstacle_stops_before_body():
 p=ground();y,z=np.meshgrid(np.linspace(-.2,.2,12),np.linspace(-.1,.4,10));wall=np.column_stack([np.full(y.size,.56),y.ravel(),z.ravel()]);d=assess(np.vstack([p,wall]));assert d['valid'] and d['nearest_obstacle_m']<.85 and d['speed_limit_m_s']==0.
def test_missing_returns_are_unknown():assert not assess(np.empty((0,3)))['valid']
def test_transient_slope_does_not_trigger_shape():
 f=DecisionFilter();d=assess(ground(12));assert f.update(d)['mode']=='track';assert f.update(d)['mode']=='track';assert f.update(d)['mode']=='climb'
def test_acceleration_feedforward_retains_current_limits():
 p=load_parameters(Path(__file__).parents[1]/'config/params.yaml');a=PowerPlant(p);b=PowerPlant(p);b.previous_targets=[.1]*4
 out=a.step(.01,[.1]*4,[0.]*4);baseline=b.step(.01,[.1]*4,[0.]*4)
 assert out['torques_Nm'][0]>baseline['torques_Nm'][0]
 assert max(out['torques_Nm'])<=p['drive']['output_torque_limit_Nm']
 assert max(out['motor_current_A'])<=p['drive']['current_limit_A']
