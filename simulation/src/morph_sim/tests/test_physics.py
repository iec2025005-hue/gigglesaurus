from pathlib import Path
import math
import pytest
from morph_sim.design_physics import load_parameters, design_report, PowerPlant, traction_prediction, contact_patch, soft_ground_prediction
P=load_parameters(Path(__file__).parents[1]/'config/params.yaml')

def test_design_is_consistent():
    r=design_report(P)
    assert r['level_load_per_wheel_N']==pytest.approx(24.525)
    assert 0<r['spring_static_sag_m']<P['suspension']['upper_m']/2
    assert r['scrub_Nm_at_mu_0_8']<P['steering']['output_torque_limit_Nm']

@pytest.mark.parametrize('name',list(P['surfaces']))
def test_traction_is_bounded(name):
    s=P['surfaces'][name]
    for vx in [-.5,0.,.5]:
        for omega in [-25.,0.,25.]:
            o=traction_prediction(25,omega,vx,.3,.04,s)
            assert (o['Fx_N']/(s['mu_kinetic_x']*25))**2+(o['Fy_N']/(s['mu_kinetic_y']*25))**2<=1.000001
            assert all(math.isfinite(v) for v in o.values())
    assert traction_prediction(0,100,0,0,.04,s)['Fx_N']==0

def test_power_limits_and_thermal_feedback():
    plant=PowerPlant(P)
    first=plant.step(.01,[20.]*4,[0.]*4)
    assert first['battery_V']<P['power']['ocv_full_V']
    for _ in range(2000):out=plant.step(.01,[20.]*4,[0.]*4)
    assert max(out['torques_Nm'])<=.5
    assert out['battery_current_A']<=P['power']['bms_limit_A']+1e-9
    assert out['winding_C'][0]>22 and out['soc']<1
    plant.temp=[105.]*4
    assert plant.step(.01,[20.]*4,[0.]*4)['torques_Nm']==[0.]*4
    plant.temp=[70.]*4
    assert plant.step(.01,[20.]*4,[0.]*4)['torques_Nm']==[0.]*4
    plant.temp=[60.]*4;plant.driver_temp=[60.]*4
    assert plant.step(.01,[20.]*4,[0.]*4)['torques_Nm'][0]>0

def test_brownout_requires_stationary_homing():
    plant=PowerPlant(P);plant.needs_home=True
    assert plant.step(.01,[20.]*4,[0.]*4)['torques_Nm']==[0.]*4
    assert not plant.home(False)
    assert plant.home(True)
    assert plant.step(.01,[20.]*4,[0.]*4)['torques_Nm'][0]>0

def test_patch_sinkage_bounded():
    length,pressure,radius=contact_patch(24.525,.04,.03,25000.)
    assert 0<length<.08 and pressure>0 and .02<radius<.04
    assert 0<soft_ground_prediction(pressure,.03,P['surfaces']['mud'])['sinkage_m']<=.02

def test_motor_rejects_invalid_input():
    with pytest.raises(ValueError):PowerPlant(P).step(.01,[float('nan')]*4,[0.]*4)
