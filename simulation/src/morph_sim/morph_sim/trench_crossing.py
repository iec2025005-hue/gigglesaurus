"""One-wheel-at-a-time trench crossing: which wheel to lift, and whether the geometry allows it.

Why this exists. Head-on, both wheels of an axle enter a trench together and the axle load sits on the
gap edges; climbing out needs more push than the other axle can give (measured: every shape is stuck in
the 100 mm trench without the torque boost). Crossing obliquely with the body yawed puts the four wheels
over the gap at four different times. The wheel over the gap is lifted clear by the corner height
actuators while the other three carry the robot, with the body leaned towards the opposite corner so
the centre of mass stays inside the support triangle.

Wheelbase matters: the wheels must be far enough apart ALONG THE DIRECTION OF TRAVEL that only one lift
window is open at a time. For travel along the trench normal with body yaw psi the wheel offsets are
+-(L/2) cos(psi) +- (T/2) sin(psi); the smallest spacing between neighbours limits the trench width.
"""
import math
from .control import LEGS, wheel_position

TRACK = .54


def wheel_offsets(d, yaw):
    """Offsets of the wheel centres along world +x for a body yawed by `yaw` [m], order fl, fr, rl, rr."""
    return [wheel_position(i, d)[0] * math.cos(yaw) - wheel_position(i, d)[1] * math.sin(yaw) for i in range(4)]


def minimum_stagger(d, yaw):
    """Smallest along-travel spacing between any two wheels [m]."""
    offsets = sorted(wheel_offsets(d, yaw))
    return min(b - a for a, b in zip(offsets, offsets[1:]))


def best_yaw(d, step=math.radians(.25)):
    """Body yaw in (0, 90 deg) that maximises the stagger, and that stagger [rad, m]."""
    candidates = [i * step for i in range(1, int(math.pi / 2 / step))]
    yaw = max(candidates, key=lambda a: minimum_stagger(d, a))
    return yaw, minimum_stagger(d, yaw)


def lift_window(trench_width, lead=.03, trail=.03):
    """Distance a wheel centre travels while it must be held clear of one trench [m]."""
    return trench_width + lead + trail


def feasible(d, yaw, trench_width, lead=.03, trail=.03):
    """True when consecutive wheels cannot be inside their lift windows at the same time."""
    return minimum_stagger(d, yaw) > lift_window(trench_width, lead, trail)


def plan(x, d, yaw, trenches, lead=.03, trail=.03, approach=.05, done=()):
    """What the suspension should do now, for body position x travelling along +x.

    trenches: iterable of (centre x, width). done: (leg, trench index) pairs already crossed, which are
    ignored so that position jitter at a window edge cannot re-select a wheel that has just finished.
    Returns dict with
      lift: leg name to hold clear, or None;  trench: index of that trench, or None
      inside: True if that wheel centre is already inside its window (it must be up NOW)
      conflict: True if two wheels need lifting at once (geometry infeasible at this yaw and shape)
    """
    needed = []
    for index, offset in enumerate(wheel_offsets(d, yaw)):
        wx = x + offset
        for number, (centre, width) in enumerate(trenches):
            start, end = centre - width / 2 - lead, centre + width / 2 + trail
            if start - approach <= wx <= end and (LEGS[index], number) not in done: needed.append((wx >= start, start - wx, LEGS[index], number))
    if not needed: return {'lift': None, 'trench': None, 'inside': False, 'conflict': False}
    needed.sort(key=lambda item: (not item[0], item[1]))        # wheels already inside first, then the nearest
    return {'lift': needed[0][2], 'trench': needed[0][3], 'inside': needed[0][0], 'conflict': sum(1 for n in needed if n[0]) > 1}
