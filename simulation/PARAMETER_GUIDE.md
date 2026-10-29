# Reproducible parameters and run records

The numerical assumption source is `src/morph_sim/config/parameter_schema.yaml`. Each entry records a default, units, source, assumed/measured status and uncertainty. Null uncertainty means unknown, not zero. The physical hardware has not been measured.

After an intentional schema edit, run:

```bash
python3 src/morph_sim/tools/resolve_parameters.py
```

This regenerates the compatibility `config/params.yaml`. Launch rejects a conflict between those files. Existing actuator and travel bounds remain enforced. Component mass edits that are not reflected in the generated inertias fail startup rather than silently changing diagnostic mass.

`wheel_radius:=auto` uses the schema default. A supported radius override propagates to robot geometry, manager, diagnostics and terrain perception. Resolved numerical values, metadata, versions, source/asset hashes and the generated robot/world are saved under `log/run_*/` for each launch. The interactions log retains the manifest alongside telemetry.

For the main multisector arena, ground and obstacle material coefficients are assigned from the surface table. Per-wheel diagnostic predictions use the native collision's material name. At a multi-material contact, diagnostics conservatively select the lowest longitudinal coefficient and record all candidate names. A nominal sector fallback is explicitly reported if collision names cannot be matched. Native Coulomb coefficients use the kinetic table; this does not implement a separate static-to-kinetic tyre law. Runtime wetness changes require relaunch so native materials and predictions stay synchronized.

The Gazebo transport partition belongs to this workspace. `arena_view.sh` selects it automatically. For hand-written Gazebo transport commands, first `source transport_env.sh` from this workspace. ROS topic commands are unaffected. `MORPH_GZ_PARTITION` is an optional dedicated partition override; do not share it with another project.

Validation commands (after sourcing the ROS and workspace setup files):

```bash
PYTHONPATH=src/morph_sim:$PYTHONPATH python3 -m pytest src/morph_sim/tests -q
python3 src/morph_sim/tools/preflight.py
python3 src/morph_sim/tools/measure_engine_friction.py validation/friction incline
python3 src/morph_sim/tools/measure_engine_friction.py validation/friction_pull pull
python3 src/morph_sim/tools/check_shutdown.py
```

The two friction protocols use analytical metrology fixtures, not presentation scenery. They validate numerical rigid-contact behavior and do not calibrate real tyres or deformable soil. Legacy alternate arenas are retained; the material reconciliation and acceptance results here apply to the main multisector profile.
