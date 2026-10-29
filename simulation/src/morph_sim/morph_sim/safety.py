"""Shared feedback guards; zero effort alone does not provide grade holding."""
import math
from .control import LEGS,STAGES,ARMS
ACTUATED_JOINTS=STAGES+ARMS+tuple(n+suffix for n in LEGS for suffix in ('_steer_joint','_height_joint','_wheel_joint'))


def valid_joint_sample(names,positions,velocities,stamp,now,timeout=.2,future_tolerance=.02):
    if len(names)!=len(set(names)) or len(names)!=len(positions) or len(names)!=len(velocities):return False
    if not math.isfinite(stamp) or not -future_tolerance<=now-stamp<=timeout:return False
    positions=dict(zip(names,positions));velocities=dict(zip(names,velocities))
    return all(name in positions and name in velocities and math.isfinite(positions[name]) and math.isfinite(velocities[name]) for name in ACTUATED_JOINTS)


def fresh_snapshot(stamp,receipt_wall,now,wall,timeout=.2,future_tolerance=.02):
    return -future_tolerance<=now-stamp<=timeout and 0<=wall-receipt_wall<=timeout


class StopPositionHold:
    """Brake first, then latch local wheel position; releases on motion command.

    Converts hold error to a bounded speed request for the electrical motor plant.
    This is powered control, and does not provide a power-loss parking brake.
    """
    def __init__(self, capture_speed, gain, speed_limit):
        self.capture_speed=capture_speed;self.gain=gain;self.speed_limit=speed_limit
        self.positions=[None]*4

    def apply(self,targets,positions,velocities,enabled=True):
        output=list(targets)
        for i,(target,q,v) in enumerate(zip(targets,positions,velocities)):
            if not enabled or abs(target)>1e-6:
                self.positions[i]=None;continue
            if self.positions[i] is None:
                if abs(v)>self.capture_speed:continue
                self.positions[i]=q
            output[i]=max(-self.speed_limit,min(self.speed_limit,self.gain*(self.positions[i]-q)))
        return output
