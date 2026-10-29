# Current controls and physics

See [RESEARCH_UPGRADE.md](RESEARCH_UPGRADE.md) for the new 150 mm wheels, combined control console, drive effort model and suspension thermal model. The command below launches this upgrade.

# MORPH connected arena

The default `run_gazebo.sh` now opens the combined **64 × 44 metre** course. It is a new world file, `src/morph_sim/worlds/multisector.sdf`, not the old standalone Marsyard or warehouse.

```bash
cd /home/diya/Documents/Codex/2026-10-24/ac/outputs/morph_spec_v4_ws
bash run_gazebo.sh
```

The nine parallel lanes run north, from y=0 to y=12. Open crossways at both ends connect every lane. Imported low barriers separate their sides; the starts and exits remain open.

1. Smooth road: asphalt, minimal elevation variation.
2. Green grass: grass PBR material and mild survey-derived unevenness.
3. Sand: granular material and undulating survey-derived geometry.
4. Mud: wet mud material, reduced friction and uneven ground.
5. Ramp lab: imported NIST ramp meshes, scaled to an approximately 5–25 degree ladder (mesh slopes 4.92–24.63 degrees), plus an optional scaled staircase.
6. Debris: imported collapsed-building mesh fragments.
7. Rock garden: 34 boulders extracted from the existing Leo Rover rock mesh, with smaller obstacles in the central route.
8. Snow: snow material and assumed low friction.
9. Mixed terrain: grass, sand, mud and gravel segments in one lane.

The side sectors contain an imported village with houses and trees, an industrial/pipeline inspection station with standpipes, a rubble/disaster-response area and an open warehouse depot. All are in the same world; no world reload is needed to drive between them.

## Start directly at a sector

```bash
bash run_gazebo.sh sector:=grass
bash run_gazebo.sh sector:=sand
bash run_gazebo.sh sector:=mud
bash run_gazebo.sh sector:=ramps
bash run_gazebo.sh sector:=rocks
bash run_gazebo.sh sector:=mixed
bash run_gazebo.sh sector:=pipeline
bash run_gazebo.sh sector:=village
```

Other valid names: `start`, `smooth`, `debris`, `snow`, `warehouse`, `rubble`. Each sector entrance faces north. Close the current launch with Ctrl+C before starting another.

## Controls

Focus the MORPH combined controls window. W/S move forward/backward, A/D turn, Q/C strafe, B brakes. E expands the upper chassis, X compacts it, N returns to normal; H/L raise/lower; K emergency stops and U resets when stopped. P toggles the presentation camera relay.

For a supervised transformation demo at the clear start:

```bash
bash run_gazebo.sh controls:=none demo:=true
```

Automatic morphology by sector is optional: `automatic_terrain:=true`. It uses the bounds of this new course. The telemetry node reports the current sector even when automatic morphology is off.

## Physics scope

Every course ground patch references the same OBJ for its visual and collision geometry. Neighbouring patches meet at z=0. The ground comes from the existing Leo Rover DEM, with elevation detrended, normalized and tapered for this small-wheel engineered course; this is not an untouched real-world site. The original Marsyard remains available with `arena:=adaptive`.

Surface friction is applied through native collision mu/mu2 with the DART engine and Bullet collision detector. Roughness is physical mesh geometry plus visual normal maps. Sand, mud and snow remain rigid surfaces: there is no claim of deformable soil, accumulating ruts or snow compaction. Obstacles can exceed the capability of 75 mm radius wheels; each lane has an accessible entrance for demonstration and testing. The default now uses electrical drive effort and position steering; see RESEARCH_UPGRADE.md.

## Assets and rebuild

- OSRF Gazebo model library, commit `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`: https://github.com/osrf/gazebo_models — CC BY 3.0, attribution and individual model authors retained in the model directories. Imported houses, trees, industrial plant, standpipes, NIST fixtures, barriers, cones and rubble. Geometry was scaled/placed; classic material bindings were converted for Ogre2. Decorative line primitives were removed from the power-plant Collada file because they crashed DART/FCL; triangle geometry remains.
- Leo Rover ERC2021 DEM and rocks: existing MIT-licensed package, see `worlds/LEO_MARSYARD_LICENSE.txt`.
- AWS warehouse asset licence: `worlds/AWS_WAREHOUSE_LICENSE.txt`.
- ambientCG Grass001, Ground037, Snow001, Gravel001, Asphalt012 and Concrete034, CC0: https://docs.ambientcg.com/license/ .
- Poly Haven brown_mud_02, CC0: https://polyhaven.com/a/brown_mud_02 and https://polyhaven.com/license .

All required assets are local. Full texture URLs and map names are recorded in `worlds/COURSE_TEXTURE_SOURCES.json`.

To regenerate the course after editing the composition, run `python3 src/morph_sim/tools/build_sectors.py`, then rebuild with `colcon build --symlink-install` in a sourced ROS environment. The current imported assets already contain the compatibility fixes.

## Camera shortcuts

From a second terminal in this workspace: `bash arena_view.sh overview` for the whole course or `bash arena_view.sh grass` (any sector name) for a close view of that entrance. `bash arena_view.sh start` looks at the initial robot position.

### Camera rendering and performance validation

`bash run_gazebo.sh` keeps camera rendering disabled by default, preserving the prior launch workload. Earlier conversion silently removed the camera sensor because its fixed link was reduced into `base_link`; that defect is corrected. Enable the sensor explicitly with `bash run_gazebo.sh camera_enabled:=true`. Its pose refers to the retained `camera_link` mount frame. `camera_width:=320|480|640` and `camera_rate:=1..30` set the native pinhole stream dimensions/rate. The HQ benchmark explicitly enables 640 px / 30 Hz rendering. A working Gazebo viewport alone does not prove that camera images are being produced.

The current execution environment has software rendering and no GPU device nodes. Restored-camera testing could not advance a two-second settling phase within a 20-second wall-time budget. Hardware-accelerated camera performance must be rechecked on the host. Do not use earlier camera-disabled runs as evidence for an HQ camera claim.

`python3 src/morph_sim/tools/run_performance_matrix.py --prefix UNIQUE_NAME` performs five interleaved repeats per declared preset after sourcing ROS, the workspace and `transport_env.sh`. It preserves failures and refuses to overwrite previous evidence. `benchmark_response_trials.py` records twenty command-response trials; `check_console_input.py` exercises synthetic Tk events through the real console handlers. These are application/simulation measurements, not calibrated physical keyboard or hardware results.

### Current controller selection

The normal electrical-drive launch now uses `position_backend:=auto`, resolving to short trajectories interpolated by ROS joint_trajectory_controller. For an explicit comparison, use `position_backend:=forward`. Legacy `physics_profile:=false` resolves `auto` to the original forward controller; an incompatible explicit trajectory request fails startup. Measured-state activation avoids the previous stored-zero height jump. Position, velocity and effort limits are retained.

The local `lifecycle_bridge` uses the installed ROS–Gazebo converters with transport-first cleanup. It logs `Bridge transport quiesced before ROS context shutdown`; detailed shutdown acknowledgements are written by the supervisor. The twenty-attempt frozen comparison had twenty clean exits. Preserve earlier failed cleanup runs as part of the evidence.

Snow mode requests at most 0.10 m/s. The 0.25 m/s snow-sector stress test exceeded this mode's envelope and failed two one-second stopping checks. Even at the lower request a later release took 1.042 seconds, so a universal one-second stopping guarantee has not been established. Select the actual terrain mode and retain conservative hardware operation until physical validation.

`LP_NUM_THREADS=2 bash run_gazebo.sh ...` is an explicit throughput preset. It improved RTF in repeated software-rendering tests but reduced GUI render cadence, so it is not a global default. Camera-disabled timing and HQ camera timing are separate cases; do not claim the former as camera-enabled performance.
