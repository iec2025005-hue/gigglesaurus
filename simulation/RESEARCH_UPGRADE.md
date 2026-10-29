# Latest RViz and local sensor adaptation

See [SMART_RVIZ_GUIDE.md](SMART_RVIZ_GUIDE.md) for the new RViz setup, faster command path, motor acceleration feedforward and range/IMU-driven chassis mode. The earlier review below describes the preceding implementation and its limitations.

# MORPH research upgrade

This upgrade lives in `morph_spec_v4_ws` (the directory name is retained so the launch command stays the same). It replaces the old small wheel visuals and combined-control restrictions. The previous source is archived in `backups/before_research_upgrade.tar.gz`.

## Start

```bash
cd /home/diya/Documents/Codex/2026-10-24/ac/outputs/morph_spec_v4_ws
bash run_arena.sh
```

Gazebo opens together with **MORPH — combined controls**. Keep the control window focused. Hold **W/S**, **Q/C**, and **A/D** together for longitudinal motion, strafing and yaw. Press **E/X/N** while driving for expand/compact/normal. **H/L** adjust ride height; **1–4** select terrain modes; **Shift** gives precision speed; **B** brakes; **Space/K** latches stop; **U** resets; **P** switches the camera relay. Release keys or leave the control window to command zero velocity. A steering change still takes physical time: the assumed steering output limit is 1 rad/s.

The controller coordinates the moving wheel attachment during transformation using vx_module = vx_body − yaw_rate*y + dx_attachment/dt and vy_module = vy_body + yaw_rate*x. Transformation is limited to low body speeds, and its stages wait for wheel alignment. Ordinary driving continues during alignment. Wheel velocity uses cosine projection during steering instead of a hard all-wheel wait. The console publishes full simultaneous key state at about 60 Hz; the manager updates at 100 Hz. Gazebo's older keypress-only control is still available as `controls:=gazebo`.

## Mechanical design changes

The assumed wheel is now **150 mm diameter × 45 mm width**, instead of 80 × 30 mm. This is a proposed mechanical change for the future build, not a cosmetic rescaling that preserves the old robot. All four wheel collision cylinders, visual scale, wheel inertia, kinematics and encoder conversion use the new dimensions. Total modeled mass remains 10 kg; masses are provisional.

Each module has a continuous rounded rubber tyre, tread, two hubs with fasteners, axle fork, geared motor casing, encoder cover, damper barrel, chrome piston and helical spring visual. The existing passive prismatic joint provides 40 mm travel. Barrel and piston are on different moving links. The helical coil is a fixed visual approximation and does not deform continuously. Module inertia is derived from the new component geometry with assumed mass allocations; see `validation/module_inertia_assumptions.json`. Cosmetic motor/encoder meshes do not introduce additional drivetrain joints.

## Applied physics versus predictions

Applied to the simulation:

- Rigid body mass, inertia, gravity and mesh contact with per-surface Coulomb friction, using DART with Bullet collision detection.
- Four passive suspension forces: spring, temperature-dependent viscous damper, smooth seal friction, and progressive end-of-travel bump stops, capped at 150 N per module.
- Damper heating from dissipated mechanical power, cooling to ambient, and a bounded viscosity/temperature relation. The native plugin publishes `/morph/suspension/telemetry` with temperature, dissipated watts and effective damping. Values in the plugin and parameter file are **assumed**.
- Drive effort from the DC motor electrical model: back EMF, winding resistance/inductance, gear reduction, current and voltage limits, battery sag, thermal derating and shutdown. `physics_profile:=true` is now the default and applies motor torques through effort controllers. Steering, height and shape still use position servos; their detailed motor electronics are not modeled.
- Quantized wheel encoders and steering angle measurements, encoder odometry and raw ground-truth odometry as separate topics. IMU/camera effects remain identified in the existing parameter model.

Diagnostic predictions only: Pacejka tyre forces, Bekker sinkage/drag, contact-patch size, estimated normal loads and steering step-loss risk. These do not add a second friction force to native contact. Ground is rigid; mud and snow do not deform. Tyre tread is visual; collision uses a smooth cylindrical envelope. The upper chassis is a reduced-order open chain with controlled visual closure, not a verified closed-loop CAD constraint model.

`physics_profile:=false` is the older ideal velocity-drive comparison mode. It is useful for separating controller/geometry faults from assumed motor limitations, and should not be reported as motor-accurate behavior.

## Performance and numerical settings

The default physics step is 0.002 s, with controller updates at 0.01 s. RTF is achieved simulation time divided by wall time; a requested RTF of 1 does not guarantee that the computer can deliver it. The console displays the observed ratio.

Use the smaller reference step for numerical sensitivity runs:

```bash
bash run_arena.sh physics_step:=0.001
```

Use `headless:=true controls:=none` for repeatable automated runs. The measured input response is command publication to ground-truth speed exceeding 0.03 m/s. It excludes human key timing and window focus delay. The benchmark writes samples, trajectory, electrical state, shock temperatures and RTF to `validation/research_motion.json`; 1 ms / GUI comparisons are saved separately. Entrance tests check settling and a short forward movement; they do not establish full obstacle traversability.

## Visual assets

The course retains its nine terrain columns and connected village, industrial, depot and rubble sectors. A surrounding landscape derived from the existing Leo Rover elevation survey replaces the floating course edge. Its rendered and contact meshes are identical. Source terrain is transformed into an engineered test site, not a reconstruction of the original survey location.

Near-course trees use **Poly Haven Tree Small 02**, and the rock lane uses **Poly Haven Rock Moss Set 01**, both CC0. Source glTF files, texture dependencies and download hashes are stored under `models/ph_*`. Tree geometry is decimated with transferred source UVs and deterministic foliage thinning; trunks have collisions, foliage is visual. Rock collision and visual use identical source-derived meshes. Existing OSRF/AWS models and ambientCG/Poly Haven materials retain their attribution from `ARENA_GUIDE.md`.

Sources: https://polyhaven.com/a/tree_small_02 , https://polyhaven.com/a/rock_moss_set_01 , https://polyhaven.com/license .

## Before claiming real-to-sim accuracy in a paper

No hardware measurements exist yet, so no measured real-to-sim error or calibrated predictive accuracy can be claimed. Required measurements include loaded wheel radius, component masses/CoM/inertia, drive torque-speed-current curves, gearbox losses, steering step response, spring force versus travel, damping versus velocity/temperature, tyre friction versus load/surface, and encoder/sensor noise. Use repeated controlled trials, report confidence intervals and timestep sensitivity, and compare simulation trajectories against those measurements. Visual realism and a passing software test suite are not evidence of calibrated physical fidelity.

Full deformable soil/tyres, chassis flex, detailed steering electronics, wear, rain/wind and the entire earlier specification remain outside this implemented model. Parameters and telemetry explicitly identify assumptions.

## Current performance review

See `reports/MORPH_Project_Review_and_Upgrade_Plan.md` and the shareable PDF under `output/pdf/`. Raw controlled benchmarks and their summary are under `validation/performance/`. The current default uses a 480 x 270 camera at 10 Hz with optional image effects disabled. `run_realtime.sh` provides a lower-cost visible mode with a 320 x 180 camera at 5 Hz, disabled shadows and two Mesa software-rendering workers where that driver is used. The same 2 ms physical timestep and motor limits apply. These are simulation-performance presets; hard real-time guarantees and hardware calibration require the release gates in the report.

The controlled 208.5-second low-cost visible run measured mean RTF 0.998, sampled one-second RTF p05 0.994 and one forward response of 151 ms. This supports near-real-time operation for that configuration. Five repeats, twenty response trials, ten-minute loaded operation and hardware calibration remain open.
