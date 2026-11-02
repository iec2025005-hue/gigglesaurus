# Simulation fixes and outstanding acceptance

These changes are software fixes reviewed on Windows. They do not close the Linux/Gazebo release gates or replace recorded failures.

## Code changes

- Complete feedback loss clears stored drive slew state, including partial joint samples, so recovery ramps from zero.
- Timed commands consistently reject JSON scalars, null and missing fields with validation errors, and no longer accept boolean velocities as numerical motion.
- Perception decisions retain the producer timestamp. Delayed, future, malformed and invalid-speed decisions fail closed rather than being refreshed by receipt.
- Manual command JSON must be an object; nonfinite durations are rejected before clamping can hide them.
- Enabled-camera response benchmarks require both image and CameraInfo streams, at least 20 messages/matched pairs, requested dimensions, valid geometry, at least 90% matched images, and at least 90% requested average simulated frame rate. Wall-rate acceptance additionally uses the existing 0.95 RTF target. Whole-interval rates include silent tails. These are explicit proposed acceptance criteria, not new measured results.
- Camera-disabled runs state that camera acceptance was not assessed. Missing manifests fail the protocol rather than allowing an unsupported camera claim.
- Stop/performance matrix runners default to `position_backend:=auto`, matching normal launch. Explicit forward/trajectory experiments remain available. Output log directories are created before opening locks/logs.

## Validation here

99 portable tests passed; one Gazebo-dependent test was excluded. The prior portable subset passed 63 tests before changes. Python compilation, Bash syntax checking and Git whitespace checks passed. The full Linux suite, C++ plugins, ROS callback integration and live sensor behavior cannot be run in this Windows session, which has no WSL/ROS/Gazebo installed. No historical result files were rewritten.

## Follow-up fixes — 3 November 2026

- Position-limit history starts from measured feedback rather than URDF zero values, and resets on feedback loss. Regression tests cover nonzero startup/recovery positions and zero/normal/long callback intervals. Live joint-limit warning elimination remains unverified.
- The perception adapter rejects delayed clouds, stale/missing joint feedback and old/future IMU samples. Valid decisions carry the cloud's source time through to the manager instead of replacing it with callback time. Live ROS integration remains pending.
- Normal launches now default to `rviz:=false`. Physics, control, sensor generation and local perception do not depend on RViz. Explicit `rviz:=true`, `run_smart.sh` and standalone `run_rviz.sh` remain available. Visualization TF/path publishers remain available for attaching RViz later. This mitigates duplicate graphics work and bypasses the recorded viewer crash during normal use; it does not repair RViz's segmentation fault.
- `validate_linux.sh` selects the sourced or workspace-recorded ROS distro instead of hardcoding Jazzy, resolves the workspace relative to its script, and tolerates ROS setup files referencing unset variables. Supported compatibility choices remain Humble/Jazzy/Lyrical; actual build compatibility still requires the corresponding Linux test.

## Ubuntu follow-up

From the repository's `simulation` directory, booted into Ubuntu:

```bash
python3 src/morph_sim/tools/check_graphics.py
bash setup_ubuntu.sh
```

The diagnostic writes `log/graphics_check.json`. If `glxinfo` is unavailable, Ubuntu's `mesa-utils` package provides it. An unknown result is not a GPU failure diagnosis. Inspect the renderer and actual GPU identity before selecting a driver. The diagnostic never changes drivers or rendering environment variables.

Start with the lower camera workload:

```bash
bash run_gazebo.sh rviz:=false camera_enabled:=true camera_width:=320 camera_rate:=5 render_shadows:=false
```

After closing that launch, source the installed ROS environment, `install/setup.bash` and `transport_env.sh` in the current shell, then run uniquely named matrices:

```bash
python3 src/morph_sim/tools/run_performance_matrix.py --prefix ubuntu_camera_fix_01
python3 src/morph_sim/tools/run_stop_matrix.py --prefix ubuntu_stop_fix_01 --low-speed .1 --low-mode snow
```

Do not reuse a prefix or run experiments concurrently. Stress stopping at 0.25 m/s is a separate experiment; the 0.1 m/s snow envelope cannot substitute for it.

## Still open

- HQ camera throughput on the intended GPU and actual Gazebo renderer.
- RViz shutdown segmentation fault and combined Gazebo/RViz/console throughput and joint-limit warnings. The saved log includes an RViz settings-write error, but it does not establish the segmentation fault's cause; no speculative shutdown patch is claimed as a fix.
- The recorded low-friction 1.042-second stop miss and stress-envelope failures. Existing limits and acceptance thresholds are unchanged; these need fresh stop traces and physical-model investigation.
- Stage C mechanics/actuator/energy/feedback realism and mechanism interference; Stage D traversal, thermal and timestep protocols.
- Measured masses, inertias, actuator identity, traction, thermal/suspension calibration, physical keyboard/display measurements and hardware holding behavior.

Only the code defects above are addressed by this patch. All other items remain open until their corresponding evidence is collected and reviewed.
