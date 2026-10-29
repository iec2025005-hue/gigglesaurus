# Staged upgrade execution status

28 October 2026. Stage A's main-arena software changes and isolated native-contact checks are implemented. The overall staged work order is NOT complete. Stages B-D have not passed their acceptance gates.

## Done in Stage A

- Source archived before changes in `backups/morph-before-stage-a-20261028.tar.gz`; local incremental commits created.
- Numerical defaults, units, provenance and unknown uncertainties consolidated in `config/parameter_schema.yaml`; compatibility YAML conflicts fail startup.
- Generated robot, native suspension settings, controller terrain profiles and diagnostic settings resolve from that source. Existing torque/travel bounds are retained. Inconsistent component mass changes fail rather than silently disagreeing with inertias.
- Wheel radius propagates to collision/visual geometry, manager, diagnostics and local terrain perception. Non-default geometry is covered by generated-robot tests.
- Main-arena ground and imported obstacle coefficients come from the same surface table. Per-wheel diagnostic material is resolved from native contact names; multi-material/fallback cases are labeled. Native Coulomb behavior remains distinct from diagnostic tyre/soil predictions.
- Per-run resolved manifests include values, metadata, source/asset hashes, versions and generated files. Explicit world/robot resource references are checked locally.
- Launch supervision preserves duplicate rejection, uses a workspace transport partition, shuts down Gazebo through its control service and then closes ROS nodes while their contexts remain valid.

## Measured evidence

- Full software suite: 81 passed, zero failures; initial pre-change suite: 58 passed. The review's 51 count is historical.
- SDF/resource preflight: passed. Main world has 102 distinct explicit local resources.
- Isolated DART/Bullet fixtures: 30/30 directional incline cases and 30/30 directional horizontal-pull cases passed declared coefficient tolerance 0.05 and position-fit RMSE 0.01 m. Raw data and exact fixture SDFs are under `validation/friction*`.
- Normal shutdown: 1.066 s; duplicate launch rejected, lock released, no traceback, forced termination or native crash in the accepted check.
- Restored original-source headless reference: mean RTF 0.9898; one response 90.5 ms over 58.5 simulated seconds.
- Upgraded pre-material-binding run: mean RTF 0.9933; one response 90.4 ms.
- Latest per-wheel material-binding run: mean RTF 0.9857; one response 62.6 ms over 58.5 simulated seconds. This is slightly below the single reference RTF; no statistical non-regression or improvement is established. Other verification work occurred during this run. The configured mean target 0.95 was met in these short headless runs.

These are sequential single runs, not a controlled repeated comparison. Response p95, one-second RTF p05 release acceptance, visible performance, five repeats/configuration and the ten-minute moving soak remain Stage B work. Do not use a single fast response as a latency guarantee.

## Remaining software/simulation work

Stage B: moving stop and failure matrix; fresh-state gating for all actuator axes; command-limit tracing and bounded trajectories; timing/sensor/display instrumentation; repeat and loaded performance experiments. Existing controller limit messages remain unresolved.

Stage C: consistent component mass/inertia derivation; non-drive actuator energy/torque-speed/backlash/homing realism; delayed/dropout encoder feedback and trajectory validation; coordinated rolling resistance/combined-slip model; full mechanism interference/closure analysis.

Stage D: dimensioned benchmark fixtures and indoor block; automated traversal/drawbar/coast-down/washboard protocols; timestep sensitivity, power/thermal/stall protocols and CameraInfo validation.

Stage A material acceptance is scoped to the main multisector electrical-drive profile. Alternate legacy arenas remain available but are not covered by that material reconciliation result. Generated compatibility artifacts do not establish hardware calibration or a faithful reproduction of deformable terrain.

## Manual work

See MANUAL_TODO.md for physical measurements, hardware identity/ratings, suspension and surface calibration, and host graphics-driver verification. All unmeasured design values remain assumed, with unknown uncertainty.

## Stage B implementation checkpoint — 29 October 2026

Stage B remains in progress. Full software suite currently passes 123 checks. The force-controlled ride-height test at the assumed 10 kg robot mass reached both ±10 mm targets: maximum error 0.208 mm, peak force 18.48 N, existing 100 N / ±15 mm / 25 mm/s limits retained (`validation/performance/height_under_load`).

All 28 actuated axes require complete, finite, fresh position and velocity feedback. The manager gates position/suspension commands on feedback and controller discovery. Position commands are limited in increment and native velocity-servo lead; a dedicated combined-motion trace passed with zero warnings, but later fault runs reproduced occasional native limit clamps under irregular delivery. Therefore command-limit elimination is **not established**.

Native drive watchdog and force-limited ride-height plant are implemented. The watchdog offers passive motor short braking or a proposed powered encoder-position hold. Powered hold requires working local feedback, power and controller; it is not a power-loss parking guarantee. The initial powered implementation latched the position before braking, causing rebound and failures; it was replaced with brake-first, capture-after-slowing behavior. Normal zero-speed control uses the electrical motor plant for bounded powered holding. Software tuning is labeled assumed. Watchdog and height mechanical work are logged, but complete shared electrical/thermal allocation remains Stage C work.

The `stop_v2` and `stop_v3` evidence includes failures and is retained. The v3 matrix was interrupted at an existing output-directory collision; it is incomplete and must not be reported as 12 passing tests. The runner now requires unique experiment prefixes. A v4 matrix is being captured; its exact per-run manifests identify code versions. Instrumentation-only changes occurred while the matrix ran, so it is not a uniform frozen-source release matrix.

New timing instrumentation and protocols retain source/manager/actuator/joint/body timestamps, sensor observation gaps and callback execution times. `benchmark_response_trials.py` provides 20 trials and optional moving loaded soak; `run_performance_matrix.py` schedules five interleaved repeats for headless, visible, HQ camera and two-worker rendering. Protocol existence does not establish measured performance. GUI widget-update timing excludes compositor/Gazebo frame time. Producer sequence counters are unavailable on some sensor messages, so actual dropped samples remain unknown rather than inferred as measured drops.

Console integration testing uses synthetic Tk events through the real bindings and publisher. It excludes physical keyboard and OS input delivery. A focus check now distinguishes the console's own top-level window from other owned windows. Results will be recorded after execution.

Stages C and D have not begun under this staged work order; their earlier listed gaps remain open. Host hardware acceleration is unavailable in the execution environment (software llvmpipe; no GPU device nodes); this does not diagnose the host driver's health. Repeated visible tests can establish software-rendering behavior only.

### Subsequent Stage B measurements

- V4 brake-first/capture-after-slowing matrix: 10/12 passed declared stopping criteria. Low-friction release (1.056 wall s / 0.674 sim s) and stale-state (1.199 wall s / 0.774 sim s) missed the one-second wall target. All stopped within distance and residual-speed criteria. All 12 cleanups had zero traceback/native crash/forced termination; some native position-limit warnings remain. This does not erase earlier intermittent shutdown failures.
- `response_pilot_20`: 20/20 initially settled trials; response p95 162.13 ms, mean RTF 0.60848, one-second RTF p05 0.51680. Response target met; real-time targets missed. No camera frames existed in this pre-restoration run.
- `console_input_20`: synthetic application-key response p95 145.60 ms; moving key-release stop 0.437 s; owned-window focus-loss stop 0.324 s; widget update rate 6.457 Hz. All declared console criteria passed. One native position-limit warning occurred; physical keyboard/OS/compositor latency remains unmeasured.
- `response_visible_lp2_pilot`: response p95 150.49 ms, mean RTF 0.68827; GUI render sections were commonly about 1.8–2.0 seconds on software rendering. This was one run, with camera absent, and exit status 241; it does not establish repeated non-regression or clean release shutdown.
- Camera generator bug found: conversion merged the fixed camera link, so the previous enrichment branch silently skipped the sensor. Generated cameras now attach to the retained mount frame via the base link, tested at 320/480/640 px. The initial restored-camera launch hit a controller service-call retry race; service-call timeout was aligned with the longer switch budget. A later run launched but failed to advance its settling phase within the wall budget. Both failures are preserved. Camera rendering is explicit opt-in (`camera_enabled:=true`) so default workload is not silently increased; HQ preset opts in. Live CameraInfo correctness remains unverified until matched images/info are observed.

The full software suite now contains 126 passing checks. Repeated preset and loaded-soak evidence is still pending; no Stage B completion is claimed.

### Interpolating controller and shutdown candidate

All twenty original preset attempts finished. Five run means averaged 0.6142 headless, 0.6191 visible and 0.6884 with two render workers. All fifteen completed camera-disabled response protocols met 250 ms p95; none met both RTF targets. Five HQ camera attempts failed during settling; 67 matched image/CameraInfo pairs were captured and all passed the declared pinhole geometry/dimension check. The original visible cleanup frequently exited nonzero despite no crash text. Those runs are failed cleanup, not clean release evidence.

A selectable `position_backend:=trajectory` now sends short trajectories to ROS joint_trajectory_controller, interpolated on controller updates. The original `forward` backend remains available. Existing position, speed and effort bounds are retained. The manager's 24-axis rate/lead limiter still applies before sending setpoints. This addresses irregular ROS command arrival without loosening joint limits. Compatibility command topics remain available for the force-controlled height plant and observers; the selected native controller consumes only the trajectory topic. Height still has one force actuator authority, not two physical servos.

The headless and visible combined-motion candidate runs passed with zero native limit warnings and observed joint motion. An explicit GUI quit service now precedes server shutdown. Both candidate runs exited zero; visible shutdown diagnostics acknowledged GUI quit and server stop with no forced termination. This is a successful mitigation, not proof that every upstream shutdown path is solved.

The subsequent candidate profile (`position_backend:=trajectory`, `LP_NUM_THREADS=2`, camera disabled) passed a 601.09-wall-second moving soak: 599.50 simulated seconds, overall mean RTF 0.99753, one-second RTF p05 0.98643, response p95 91.23 ms, zero missing-feedback status messages. The moving sample fraction was 0.612. Expanded/raised alternating forward/reverse commands used the assumed robot mass, with no extra calibrated payload. Five additional twenty-trial headless repeats all passed: mean RTF 0.9970–0.9997 and per-run body-response p95 72.86–89.27 ms; all five exited cleanly with zero native limit warnings. Exact logs, clocks, manifest/source hashes and mechanical/thermal model samples are retained. This soak was ten minutes of wall time, just under 600 simulated seconds; do not describe it as ten full simulated minutes.

These later-session results cannot isolate controller changes from machine load or renderer-worker changes. A new stopping matrix and visible preset comparison are required before changing defaults. Stage B remains in progress; Stages C/D remain pending. The current full software suite passes 128 checks.

The first interpolating-controller stopping matrix (`stop_jtc_v1`) measured 10/12 physical stop passes but remained rejected for two low-friction stop-time misses and timeout-triggered joint-command clamps. A trace isolated the remaining clamp: JointTrajectoryController's timeout replaces its last target with measured position, creating a step larger than the unchanged per-period position limit. The native watchdog already independently stops all axes within its 200 ms wall deadline using fresh ECM states. The candidate therefore disables the duplicate JTC timeout snap while retaining native watchdog stopping, all manager freshness guards and actuator limits. The drive brake-to-hold handoff now resets its stored PI braking integral only when a wheel's resting position is captured, avoiding inherited braking bias.

Experiment scheduling fault: an interrupted queued parent survived and ran later, causing duplicate guard rejections for `stop_jtc_v2` and conflicting `perf_jtc_v2`/`perf_jtc_v3` attempts. These batches are invalid and excluded, annotated in `validation/INVALID_BATCHES.json`. A new batch-level exclusive lock serializes experiments, forwards interruption to its owned runner and stops after setup failure. Single-run launch guarding remains intact. A fresh `stop_jtc_v3` matrix is capturing the coordinated stop-path candidate.

The coordinated watchdog / brake-to-hold handoff suite passes 130 checks, including exclusive experiment batching and cancelled-run rejection. The high-friction half of the fresh `stop_jtc_v3` matrix passed all six failure cases in 0.381–0.601 wall seconds with zero native limit warnings and clean shutdown. The matrix is not accepted until its low-friction half completes.

The fresh coordinated `stop_jtc_v3` matrix eliminated native limit warnings (zero in all 12 runs) and had 10/12 physical stop passes at the 0.25 m/s stress request, including the snow sector without snow-mode speed limiting. The two low-friction stop-time misses remain recorded. Testing at the existing snow mode's 0.10 m/s request subsequently achieved 12/12 physical stop passes, but two installed-bridge cleanup crashes rejected that batch overall (`stop_jtc_snow_envelope`). These are distinct experiments, not interchangeable release acceptance.

A local `lifecycle_bridge` executable now reuses the installed ros_gz_bridge conversion library and configuration, disabling automatic ROS context shutdown on signals. On shutdown it cancels bridge heartbeat, removes Gazebo subscriptions while node/publishers/context remain alive, releases handles/services, then closes ROS. No system package was patched. This is based on the installed public/protected API, with teardown ordering checked against ros_gz 3.0.10 source. Binary hashes are included in run manifests. The subsequent `stop_bridge_v1` matrix produced 12/12 clean exits and zero native limit warnings. Physical stopping passed 11/12: a snow-mode release required 1.042 wall seconds, missing the existing 1.000-second target. Therefore neither a universal one-second stopping guarantee nor overall safety release acceptance is claimed.

Sensor benchmark callbacks now capture middleware publication-sequence metadata, when provided, per publisher GID. Writer-sequence gaps are separately reported for the bridge-to-observer connection; unknown or upstream Gazebo/bridge losses are not reported as measured sensor drops. A new final preset comparison will assess the revised bridge/controller configuration without changing defaults mid-experiment.

### Final bridge/controller timing comparison and activation fix

Frozen `perf_bridge_v1` finished twenty attempts with twenty clean shutdowns. Each camera-disabled preset completed five twenty-trial response protocols (300 trials total). Run-mean RTF averaged 0.99973 headless, 0.95608 visible, 0.99148 with two render workers. All fifteen completed protocols met mean RTF ≥0.95, one-second RTF p05 ≥0.90 and response p95 ≤250 ms. Per-run response-p95 ranges were 75.64–89.52 ms, 89.90–94.77 ms and 83.80–91.88 ms respectively. Available ROS writer-sequence metadata showed no observed gaps in the assessed joint-state/odom/IMU connections; this excludes upstream Gazebo/bridge loss. All five HQ attempts failed during settling; matched live image/CameraInfo geometry checks passed but HQ real-time acceptance was not achieved.

Two render workers improved simulation throughput but worsened GUI render cadence on this software-rendering environment, so the global worker setting was not changed. Renderer timing excludes compositor/display latency. Timing success is distinct from presentation-quality rendering.

Some visible starts still produced a single height clamp before manager operation. The trace showed the loaded height axis moved slightly before activation while JointTrajectoryController initialized from stored zero. It now initializes from measured state (`set_last_command_interface_value_as_state_on_activation: false`). Five subsequent visible startup checks passed with zero native limit warnings and clean shutdown. `position_backend:=auto` now selects interpolation for the electrical-drive profile and the original forward backend for `physics_profile:=false`, preserving legacy profile compatibility. Explicit incompatible combinations fail startup. The current full suite passes 134 checks.

A combined Gazebo/RViz/console test is being measured separately because RViz was disabled for the repeated preset comparison. Do not extend the preset results to additional displays before measurement. Stage C and D source changes have not begun; the Stage B residual gaps remain HQ/software-rendered visual performance, hardware input/display validation and universal one-second low-friction stopping. These remain explicit failed/unverified criteria rather than hidden pass claims.
