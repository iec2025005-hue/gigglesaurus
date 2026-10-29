"""Ground-truth arena zones. Bounds match worlds/adaptive_arena.sdf."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Zone:
    name: str
    x_min: float
    x_max: float
    mode: str


ZONES = (
    Zone('grass', 0., 4., 'grass'),
    Zone('mud', 4., 8., 'mud'),
    Zone('rocky', 8., 12., 'rocky'),
    Zone('sand', 12., 16., 'sand'),
    Zone('ramps', 16., 20., 'climb'),
    Zone('debris', 20., 24., 'debris'),
    Zone('snow', 24., 28., 'snow'),
    Zone('mixed', 28., 32., 'mixed'),
)


def zone_at(x, y, previous=None, hysteresis=.20):
    """Keep the previous zone near a boundary to avoid chattering."""
    if not all(math.isfinite(v) for v in (x, y)) or abs(y) > 2.9:
        return None
    if previous is not None and previous.x_min-hysteresis <= x < previous.x_max+hysteresis:
        return previous
    for zone in ZONES:
        if zone.x_min <= x < zone.x_max:
            return zone
    return None
