"""V2 commit 3: camera colour classification and boundary distance."""
import math
import numpy as np
import pytest
from morph_sim.vision_core import analyse, classify_colour, row_distance, CAMERA_HEIGHT, CAMERA_FORWARD, HORIZONTAL_FOV, CLASS_FRICTION

# Colours as RENDERED by Gazebo in the Tetra-Arena (measured, run v2s4_tour_fixed_compact_try1), not the SDF material values.
ARENA_COLOURS = {'tarmac': (.45, .44, .445), 'sand': (.79, .73, .577), 'mud': (.55, .468, .374), 'concrete': (.65, .642, .611)}


def synthetic(near, far, boundary_m, height=270, width=480, shade=1.):
    """Flat ground of colour `near` up to boundary_m from the camera, colour `far` beyond, sky above the horizon."""
    image = np.zeros((height, width, 3), dtype=np.uint8); image[:] = (140, 170, 215)
    for row in range(height):
        d = row_distance(row, height, width)
        if d is None: continue
        colour = ARENA_COLOURS[near] if d < boundary_m else ARENA_COLOURS[far]
        image[row] = [int(255 * min(1., c * shade)) for c in colour]
    return image


@pytest.mark.parametrize('name', sorted(ARENA_COLOURS))
def test_arena_colours_are_classified_under_brighter_and_darker_light(name):
    for shade in (.85, 1., 1.15):
        r, g, b = (min(1., c * shade) for c in ARENA_COLOURS[name])
        assert classify_colour(r, g, b)[0] == name, (name, shade)


def test_sdf_material_colours_of_the_soils_are_still_separated_by_hue():
    # Hue is independent of light level, so the unlit material colours classify the same way.
    assert classify_colour(.76, .66, .42)[0] == 'sand' and classify_colour(.33, .24, .16)[0] == 'mud'
    assert classify_colour(.02, .30, .31)[0] == 'marker'                           # painted start pad (teal)


def test_row_distance_is_the_flat_ground_projection():
    height, width = 270, 480; focal = (width / 2) / math.tan(HORIZONTAL_FOV / 2)
    assert row_distance(100, height, width) is None                               # above the horizon
    assert row_distance(269, height, width) == pytest.approx(CAMERA_HEIGHT * focal / (269 - 134.5))
    assert row_distance(200, height, width) > row_distance(260, height, width)    # higher rows look further


@pytest.mark.parametrize('near,far,boundary', [('tarmac', 'sand', 2.5), ('sand', 'mud', 4.0), ('mud', 'tarmac', 1.8), ('tarmac', 'concrete', 3.0)])
def test_boundary_class_and_distance(near, far, boundary):
    result = analyse(synthetic(near, far, boundary))
    assert result['valid'] and result['class_here'] == near and result['class_ahead'] == far
    # Row quantisation: one pixel at 3 m is about 0.05 m on a 480-px image.
    assert result['boundary_distance_m'] == pytest.approx(boundary + CAMERA_FORWARD, rel=.06)
    assert result['friction_here'] == CLASS_FRICTION[near] and result['confidence'] > .5


def test_uniform_ground_reports_no_boundary_and_noise_does_not_invent_one():
    image = synthetic('tarmac', 'tarmac', 99.); result = analyse(image)
    assert result['class_here'] == 'tarmac' and result['class_ahead'] is None and result['boundary_distance_m'] is None
    rng = np.random.default_rng(3); noisy = np.clip(image.astype(int) + rng.integers(-12, 13, image.shape), 0, 255).astype(np.uint8)
    assert analyse(noisy)['class_ahead'] is None
    with pytest.raises(ValueError): analyse(np.zeros((10, 10), dtype=np.uint8))
