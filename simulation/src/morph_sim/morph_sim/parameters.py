"""Resolve the authoritative schema; unknown uncertainty is never fabricated."""
import copy
import math
from pathlib import Path
import yaml


def resolve_schema(path):
    schema = yaml.safe_load(Path(path).read_text())
    if schema.get('schema_version') != 1:
        raise ValueError('Unsupported parameter schema version')
    metadata = {}

    def visit(node, key=''):
        if isinstance(node, dict) and 'default' in node:
            required = {'default', 'units', 'status', 'uncertainty', 'source'}
            if not required.issubset(node):
                raise ValueError('Incomplete parameter metadata: ' + key)
            value = node['default']
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Nonfinite numerical parameter: ' + key)
            if node['status'] not in ('assumed', 'measured'):
                raise ValueError('Invalid provenance status: ' + key)
            uncertainty = node['uncertainty']
            if uncertainty is not None and (not math.isfinite(uncertainty) or uncertainty < 0):
                raise ValueError('Invalid uncertainty: ' + key)
            if node['status'] == 'measured' and uncertainty is None:
                raise ValueError('Measured parameter requires uncertainty: ' + key)
            if not node['units'] or not node['source']:
                raise ValueError('Missing parameter units/source: ' + key)
            metadata[key] = copy.deepcopy(node)
            return value
        if isinstance(node, dict):
            return {k: visit(v, f'{key}.{k}' if key else k) for k, v in node.items()}
        if isinstance(node, list):
            return [visit(v, f'{key}[{i}]') for i, v in enumerate(node)]
        return node

    result = visit(schema['parameters'])
    for section, keys in {
        'robot':('mass_kg','wheel_radius_m','wheel_width_m','wheel_mass_kg','unsprung_mass_per_module_kg','wheelbase_m','track_m','com_height_m'),
        'drive':('resistance_ohm','inductance_H','kt_Nm_A','ke_Vs_rad','gear_ratio','current_limit_A','thermal_capacity_J_K','thermal_resistance_K_W'),
        'steering':('gear_ratio','motor_holding_torque_Nm','pullout_speed_rad_s','steps_per_motor_turn'),
        'power':('capacity_Ah','ocv_full_V','ocv_empty_V','bms_limit_A','battery_capacity_J_K','battery_thermal_resistance_K_W'),
        'sensors':('encoder_counts_per_wheel_turn','encoder_max_edges_s','magnetic_encoder_resolution_rad')}.items():
        for key in keys:
            if result[section][key] <= 0:raise ValueError('Expected positive parameter: '+section+'.'+key)
    for section,key in [('drive','gear_efficiency'),('steering','gear_efficiency'),('power','buck_efficiency')]:
        if not 0 < result[section][key] <= 1:raise ValueError('Invalid efficiency: '+section+'.'+key)
    if not 0 <= result['power']['soc_initial'] <= 1 or result['power']['ocv_full_V'] <= result['power']['ocv_empty_V']:
        raise ValueError('Invalid battery state or voltage range')
    if 4*result['robot']['unsprung_mass_per_module_kg'] >= result['robot']['mass_kg']:
        raise ValueError('Unsprung mass must leave a positive sprung mass')
    for name, surface in result['surfaces'].items():
        for key in ('mu_static_x','mu_static_y','mu_kinetic_x','mu_kinetic_y','wet_mu_x','wet_mu_y'):
            if not 0 <= surface[key] <= 1.2:raise ValueError('Surface coefficient exceeds native tyre material range: '+name+'.'+key)
        if surface['Crr'] < 0:raise ValueError('Negative rolling resistance: '+name)
    for key,limit in [('state_timeout_s',.2),('command_timeout_s',.2),('future_stamp_tolerance_s',.02)]:
        if 'control_safety' in result and not 0 < result['control_safety'][key] <= limit:raise ValueError('Safety deadline exceeds validated bound: '+key)
    if result['control_safety'].get('watchdog_mode') not in ('short_brake','powered_hold'):raise ValueError('Invalid watchdog mode')
    if result['control_safety']['watchdog_hold_kp_Nm_rad'] <= 0:raise ValueError('Nonpositive watchdog hold gain')
    for key in ('hold_capture_speed_rad_s','hold_position_gain_s_inv','hold_speed_limit_rad_s'):
        if not math.isfinite(result['control_safety'][key]) or result['control_safety'][key]<=0:raise ValueError('Invalid hold control setting: '+key)
    if not .05<=result['control_safety']['trajectory_horizon_s']<=.2:raise ValueError('Unsupported position trajectory horizon')
    s = result['suspension']
    for key, lo, hi in [('stiffness_N_m', 500, 4000), ('damping_Ns_m', 20, 180),
                        ('force_cap_N', 0, 150)]:
        if not lo <= s[key] <= hi:
            raise ValueError('Suspension setting outside existing safety bounds: ' + key)
    if s['lower_m'] < -.01 or s['upper_m'] > .03:
        raise ValueError('Suspension travel may not exceed the existing mechanism bounds')
    for section,key,limit in [('drive','output_torque_limit_Nm',.5),('steering','output_torque_limit_Nm',.7),('steering','output_speed_limit_rad_s',1.)]:
        if not 0 < result[section][key] <= limit:
            raise ValueError('Actuator setting exceeds existing limit: '+section+'.'+key)
    if not s['lower_m'] < s['bump_lower_m'] < 0 < s['bump_upper_m'] < s['upper_m']:
        raise ValueError('Suspension bump stops must lie within joint travel')
    for key in ('thermal_capacity_J_K', 'thermal_resistance_K_W', 'seal_velocity_scale_m_s', 'adjustment_time_s'):
        if s[key] <= 0:
            raise ValueError('Nonpositive suspension setting: ' + key)
    for name, mode in result.get('control_modes', {}).items():
        if not 500 <= mode['stiffness'] <= 4000 or not 20 <= mode['damping'] <= 180:
            raise ValueError('Control mode exceeds native suspension bounds: '+name)
    return result, metadata


def resolve_profile(share, radius=None):
    share = Path(share)
    params, metadata = resolve_schema(share/'config/parameter_schema.yaml')
    legacy = yaml.safe_load((share/'config/params.yaml').read_text())
    if legacy != params:
        raise ValueError('params.yaml conflicts with parameter_schema.yaml; regenerate the compatibility file')
    if radius is not None:
        if not math.isfinite(radius) or not .02 <= radius <= .15:
            raise ValueError('wheel_radius must be finite and in [0.02, 0.15] m')
        params['robot']['wheel_radius_m'] = radius
        metadata['robot.wheel_radius_m']['resolved_value'] = radius
        metadata['robot.wheel_radius_m']['override_source'] = 'wheel_radius launch argument'
    return params, metadata
