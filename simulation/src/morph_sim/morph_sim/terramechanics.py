"""Reference wheel-soil model for the V2 terramechanics emulation (mirrors src/terramechanics_model.hpp).

What it is: a force-level emulation layered on Gazebo's rigid Coulomb contact. Sinkage and soil slip
are computed STATE VARIABLES; the ground mesh does not deform and the tyre does not physically slip.
The only thing applied to the simulation is a resisting torque on each wheel.

Model, per wheel, with normal load W, wheel diameter D, width b and soil (kc, kphi, n, c, phi, K):

1. Rigid-wheel static sinkage (Bekker):
       z0 = [3 W / ((3 - n)(kc + b kphi) sqrt(D))] ^ (2 / (2n + 1))
2. Slip needed for the demanded soil thrust H (Janosi-Hanamoto), contact length l = sqrt(D z):
       H / H_max = 1 - (K / (s l)) (1 - exp(-s l / K)),   H_max = b l c + W tan(phi)
3. Slip sinkage (Lyasko):   z = z0 [1 + g ((1 + s)/(1 - 0.5 s) - 1)]
   Steps 2 and 3 are solved together by damped fixed-point iteration (a deeper wheel has a longer
   contact and needs less slip for the same thrust).
4. Compaction resistance with repetitive loading (Wong): work is only done on soil that has not been
   compacted to this depth before. With rut depth z_r left by a leading wheel:
       R_c = V(z) - V(min(z, z_r)) + rho V(min(z, z_r)),   V(z) = (kc + b kphi) z^(n+1) / (n + 1)
5. Optional slip power-loss equivalent R_s = H s / (1 - s). OFF by default: a rigid contact cannot
   slip, and charging the loss as torque would stall a torque-limited motor that in reality would
   simply spin faster. Soil slip is therefore reported, and acts through slip sinkage only.

Wheelbase enters only through the normal loads: a short wheelbase transfers more load to the rear
axle on a grade or under acceleration, the rear wheels then sink below the front ruts, and V(z) is
convex. On level ground at constant speed the model predicts almost no wheelbase effect.

Parameters are ASSUMED (config/params.yaml: surfaces.<name>.bekker and terramechanics).
"""
import math

SOLVER_ITERATIONS = 24
SOLVER_RELAXATION = .5


def solve_slip_parameter(fraction):
    """x >= 0 with 1 - (1 - exp(-x))/x = fraction (monotonic); x = s l / K."""
    fraction = max(0., min(.98, fraction))
    if fraction < 1e-9: return 0.
    low, high = 0., 1.
    while 1 - (1 - math.exp(-high)) / high < fraction: high *= 2.
    for _ in range(60):
        mid = .5 * (low + high)
        value = 1 - (1 - math.exp(-mid)) / mid if mid > 1e-12 else 0.
        if value < fraction: low = mid
        else: high = mid
    return .5 * (low + high)


def virgin_work(z, width, soil):
    """Compaction work per unit distance on undisturbed soil, N."""
    return (soil['kc'] + width * soil['kphi']) * max(0., z) ** (soil['n'] + 1) / (soil['n'] + 1)


def wheel_soil_state(normal, thrust, radius, width, soil, config, rut_depth=0.):
    """All forces in N, lengths in m. `thrust` is the soil thrust demanded at this wheel (>= 0)."""
    zero = dict(static_sinkage_m=0., sinkage_m=0., slip_ratio=0., contact_length_m=0., thrust_limit_N=0.,
                compaction_resistance_N=0., slip_resistance_N=0., resistance_N=0., rut_depth_m=max(0., rut_depth))
    if not normal > 0 or not math.isfinite(normal): return zero
    n = soil['n']; modulus = soil['kc'] + width * soil['kphi']; diameter = 2 * radius
    limit = config['sinkage_limit_radius_fraction'] * radius
    z0 = min(limit, (3 * normal / ((3 - n) * modulus * math.sqrt(diameter))) ** (2 / (2 * n + 1)))
    z = z0; slip = 0.; length = math.sqrt(diameter * z0); thrust_limit = 0.
    for _ in range(SOLVER_ITERATIONS):
        length = math.sqrt(diameter * z)
        thrust_limit = width * length * soil['cohesion_Pa'] + normal * math.tan(soil['phi_rad'])
        x = solve_slip_parameter(abs(thrust) / thrust_limit if thrust_limit > 0 else 0.)
        target = min(config['slip_ratio_limit'], x * soil['shear_length_m'] / length) if length > 0 else 0.
        slip += SOLVER_RELAXATION * (target - slip)
        z = min(limit, z0 * (1 + config['slip_sinkage_gain'] * ((1 + slip) / (1 - .5 * slip) - 1)))
    rut = max(0., rut_depth); shared = min(z, rut)
    compaction = virgin_work(z, width, soil) - virgin_work(shared, width, soil) + config['reload_resistance_fraction'] * virgin_work(shared, width, soil)
    slip_resistance = abs(thrust) * slip / (1 - slip) if config['slip_drag_enabled'] >= .5 else 0.
    return dict(static_sinkage_m=z0, sinkage_m=z, slip_ratio=slip, contact_length_m=length, thrust_limit_N=thrust_limit,
                compaction_resistance_N=compaction, slip_resistance_N=slip_resistance, resistance_N=compaction + slip_resistance,
                rut_depth_m=max(z, rut))


def tandem_resistance(front_load, rear_load, thrust_per_wheel, radius, width, soil, config):
    """Leading wheel on undisturbed soil followed by a wheel in its rut; returns (front, rear) states."""
    front = wheel_soil_state(front_load, thrust_per_wheel, radius, width, soil, config, 0.)
    rear = wheel_soil_state(rear_load, thrust_per_wheel, radius, width, soil, config, front['rut_depth_m'])
    return front, rear
