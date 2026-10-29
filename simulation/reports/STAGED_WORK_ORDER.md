You are continuing work on the MORPH swerve-robot simulation workspace
(/home/diya/Documents/Codex/2026-10-24/ac/outputs/morph_spec_v4_ws, ROS 2 Lyrical / Gazebo Sim 10).
Read MORPH_Project_Review_and_Upgrade_Plan.pdf, the spec, ARENA_GUIDE.md,
validation/performance/summary.json and the current source before changing anything.
Treat the review as your work order. Fix every shortcoming it lists that can be fixed in software/simulation.

GROUND RULES
1. Honesty over appearance. Never claim a result you did not measure. Label every
   hardware-dependent parameter "assumed" until a real measurement exists. Keep
   demo outcomes, simulation evidence and hardware validation clearly separate.
2. Back up first (tar the current source), commit in small steps, keep the
   duplicate-launch guard, and don't weaken actuator limits to make a test pass.
3. After every stage: run the full software suite (currently 51 checks, 0 failures),
   re-run the benchmark, and record before/after numbers. Do not regress RTF.
4. Maintain MANUAL_TODO.md throughout. Anything that needs a human, real hardware,
   a physical measurement, a driver/OS change, a licence decision, or a choice only
   I can make goes there with: what, why, exact steps, what file/parameter to update
   afterwards, and priority. Do not guess or invent values for these.
5. At the end, give me a report: done / partially done / not done (and why) / manual items.

STAGE A: Reproducible baseline (P0)
- Create ONE validated parameter schema (units, default, uncertainty, source,
  "assumed/measured" flag). Generate from it: URDF/Xacro args, native suspension
  plugin settings (seal friction, bump stiffness, force caps are currently hard-coded),
  and the diagnostic params.yaml. Reject conflicting launch overrides at startup.
- Fix the wheel-radius mismatch: changing launch radius must update the diagnostic
  parameters (encoder, torque, contact predictions). Add a test that fails if they diverge.
- Reconcile surface physics: one friction table drives the diagnostic model AND the
  world SDF/obstacle materials (rocks are wet_rock 0.35 in diagnostics but ~0.70/0.665 in SDF).
  Add slip/pull tests that measure effective engine friction per surface.
- Export a resolved parameter manifest + source hashes + versions with every run.
- Fix clean shutdown paths.
- Exit: generated URDF, SDF and diagnostics agree; missing/conflicting params fail startup.

STAGE B: Safety, controls, performance (P0/P1)
- Moving emergency-stop test (high- and low-friction ground), measuring stop time
  and distance; plus key-release, focus-loss, stale-command, stale-state and
  control-node-crash tests. Require fresh measured joint states before any actuator command.
- Trace the throttled controller command-limit messages during stage movement and
  steering transitions (log requested vs clamped position/velocity, tracking error).
  Fix the command trajectory/timing/lead, NOT the limits.
- Timing instrumentation: command stamp, manager receipt, actuator command, joint
  response, body response, in both monotonic wall time and sim time; ROS callback
  timing; achieved sensor rates; dropped samples; graphics frame time.
- Benchmarks: >= 20 response trials (report p95), 5 repeated runs per config,
  a 10-minute loaded soak, and compare visible / headless / high-quality camera /
  2-render-worker presets. Targets: mean RTF >= 0.95, one-second RTF p05 >= 0.90,
  response p95 <= 250 ms. Report honestly if missed. Make the 2-render-worker
  setting the default only if repeat runs support it.
- Add a console/keyboard-to-motion latency test and display-rate measurement
  (currently unmeasured).

STAGE C: Mechanics and actuators (software side)
- Derive component mass/inertia consistently from a single source; make the
  10 kg assumption explicit and traceable.
- Add configurable actuator realism: steering/height/expansion torque-speed limits,
  rate limits, backlash, stiction, homing/step-loss behaviour, and include their
  energy in the shared power budget.
- Add a sensor-feedback path (timestamped sampling, delay, quantization, dropout)
  so controllers can run on encoder data instead of raw sim state; validate encoder
  odometry vs ground truth on straight, circle and mixed-slip runs; report swerve
  observability limits.
- Add rolling-resistance as a measured/configurable parameter, then a regularized
  combined-slip tyre model with relaxation ONLY if it replaces or explicitly
  coordinates with native friction (no double counting). Gate it behind a flag.
- Check mechanism closure/interference across the full motion range.
- Document the drive-limit feasibility bound (0.5 N m/wheel, 75 mm radius, 10 kg:
  ~15.8 deg zero-resistance grade; 25 deg needs ~0.80 N m/wheel at Crr 0.015).
  Do not promise climbs beyond it; record such ramps as expected-failure tests.

STAGE D: Benchmark facility
- Add: measured side-slope lanes, step/gap ladders scaled to loaded wheel radius,
  washboard, potholes/ruts, split-friction lane, unpowered coast-down lane,
  1 m ground markers, fixture IDs and safe reset poses. Store dimensions and
  collision geometry together; verify with automated measurements.
- Add the true indoor block (tile/grout, carpet, rubber mat, thresholds, doors,
  narrow corridors, turns) connected into the existing loop. Use only assets with
  recorded licences/provenance (SOURCE.json).
- Automated protocols: ramp, step/gap, side-slope, drawbar, coast-down, washboard,
  with repeatable reset, declared pass/fail criteria and full data logs.
- Also cover: timestep sensitivity (1 ms vs 2 ms, same seed/command trace,
  comparing trajectory, suspension extrema, contact impulse, energy), battery sag,
  heat soak/cool-down, motor stall.
- Scale CameraInfo with resolution and validate it against the image stream.

Work through the stages in order, finish each fully, and update the review
document/validation JSON with new results. Then write the final report and MANUAL_TODO.md.