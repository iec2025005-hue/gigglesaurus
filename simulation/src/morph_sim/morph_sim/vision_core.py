"""Ground-surface classification and boundary distance from the forward camera (ROS-independent).

The camera looks straight ahead from 0.515 m above the ground, 0.13 m ahead of the body origin. For a flat
ground plane an image row v below the horizon sees the ground at

    d(v) = h_cam / tan(atan((v - c_y) / f)) = h_cam * f / (v - c_y),      f = (W / 2) / tan(HFOV / 2)

Each row of the lower image is classified from the median colour of its central columns by hue, saturation
and brightness. The boundary is the nearest row whose class differs from the class directly ahead of the
robot. Classes are colour classes: this identifies painted surfaces of the simulated arena, it is not a
general terrain classifier, and it assumes flat ground between the robot and the boundary.
"""
import colorsys
import math
import numpy as np

CAMERA_HEIGHT = .515          # m above ground: base origin 0.335 + camera mount 0.18
CAMERA_FORWARD = .13          # m ahead of the body origin
HORIZONTAL_FOV = 1.15         # rad (assets.enrich_sdf)
CLASSES = ('tarmac', 'concrete', 'sand', 'mud', 'marker', 'unknown')
# Friction the controller should assume on each colour class (V2 brief values for sand and mud).
CLASS_FRICTION = {'tarmac': .765, 'concrete': .7225, 'sand': .35, 'mud': .25}


# Thresholds calibrated on frames rendered by Gazebo in the Tetra-Arena (run v2s4_tour_fixed_compact_try1,
# 52 frames, 320 x 180). Rendered ground colours, HSV: tarmac (grey, v 0.45), concrete (grey, v 0.65),
# sand (43 deg, 0.27, 0.79), mud (32 deg, 0.32, 0.55). Rendered values differ from the SDF material values
# (lighting and tone mapping roughly double the brightness of dark surfaces).
GREY_SATURATION = .18         # below this the sample is grey
GREY_SPLIT = .54              # geometric mean of rendered tarmac and concrete brightness
SOIL_HUE_SPLIT = 37.5         # deg: mud is redder than sand; hue does not change with light level


def classify_colour(r, g, b):
    """Colour class of one RGB sample in [0, 1], and a confidence in [0, 1]."""
    h, s, v = colorsys.rgb_to_hsv(float(r), float(g), float(b)); hue = h * 360.
    if s < GREY_SATURATION:                       # grey: split by brightness (tolerates about +-15 % light change)
        return ('tarmac', min(1., .5 + (GREY_SPLIT - v) / .18)) if v < GREY_SPLIT else ('concrete', min(1., .5 + (v - GREY_SPLIT) / .22))
    if 20. <= hue <= 60.:                         # yellow-brown family: split by hue
        return ('sand', min(1., .5 + (hue - SOIL_HUE_SPLIT) / 11.)) if hue >= SOIL_HUE_SPLIT else ('mud', min(1., .5 + (SOIL_HUE_SPLIT - hue) / 11.))
    if hue < 20. or hue > 330. or s > .5: return 'marker', .8      # red walls, orange kerbs, painted start pad
    return 'unknown', .3


def row_distance(row, height, width, camera_height=CAMERA_HEIGHT, fov=HORIZONTAL_FOV):
    """Ground distance from the camera seen at image row `row` (0 = top); None at or above the horizon."""
    focal = (width / 2) / math.tan(fov / 2); below = row - (height - 1) / 2
    return camera_height * focal / below if below > .5 else None


def analyse(image, camera_height=CAMERA_HEIGHT, fov=HORIZONTAL_FOV, column_fraction=.30, minimum_run=3):
    """image: H x W x 3 uint8 RGB. Returns the class under the robot's path, the next class and its distance."""
    image = np.asarray(image)
    if image.ndim != 3 or image.shape[2] < 3: raise ValueError('expected an RGB image')
    height, width = image.shape[:2]; half = max(1, int(width * column_fraction / 2)); centre = width // 2
    rows = []
    for row in range(height - 1, -1, -1):         # nearest ground first
        distance = row_distance(row, height, width, camera_height, fov)
        if distance is None or distance > 12.: break
        r, g, b = np.median(image[row, centre - half:centre + half, :3].reshape(-1, 3), axis=0) / 255.
        name, confidence = classify_colour(r, g, b); rows.append((distance, name, confidence, (r, g, b)))
    if not rows: return {'valid': False, 'reason': 'no ground rows'}
    here = rows[0][1]; nearest = rows[0][0]; boundary = None; run = 0; candidate = None
    for distance, name, confidence, _ in rows:
        if name != here and name == candidate: run += 1
        elif name != here: candidate, run, first = name, 1, distance
        else: candidate, run = None, 0
        if candidate and run == 1: first = distance
        if candidate and run >= minimum_run: boundary = (first, candidate); break
    same = [c for d, n, c, _ in rows if n == here]
    h, s, v = colorsys.rgb_to_hsv(*rows[0][3])
    return {'valid': True, 'class_here': here, 'confidence': float(np.mean(same)), 'nearest_ground_m': nearest + CAMERA_FORWARD,
            'class_ahead': boundary[1] if boundary else None, 'boundary_distance_m': boundary[0] + CAMERA_FORWARD if boundary else None,
            'friction_here': CLASS_FRICTION.get(here), 'hsv_here': [h * 360., s, v], 'rows_used': len(rows),
            'scope': 'colour classes of the simulated arena; flat ground assumed up to the boundary'}
