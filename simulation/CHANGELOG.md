# Changelog

Format: what was wrong, what was fixed, how it was verified. Simulation only; all robot
parameters are assumed values.

## V2 complete research suite, Commits 4-5 — dashboard, ablation, figures, video, paper documents (2026-11-13)

Reports: `reports/v2_status.md`, `reports/paper_ready_summary.md` (generated), `reports/research_paper_proposals.md`.

| What was wrong | What was fixed | Verified by |
|---|---|---|
| A tour could not move the robot past a failed zone: the protocol process had no `GZ_PARTITION`, so `gz service set_pose` timed out and every later zone was recorded as failed | `tools/run_benchmark_case.py` gives the protocol the workspace's Gazebo partition | Sim: fixed-Compact tour fails trenches and ramp, is placed past both, passes the other 8 zones |
| The camera classifier's thresholds were set from the SDF material colours; Gazebo renders them about twice as bright, so tarmac read as concrete and mud as sand | Thresholds calibrated on 52 rendered frames; sand and mud are now separated by hue | Unit tests use the rendered colours |
| For a fixed policy the explanation said "no stance can pass" and listed stances that were never candidates | "fixed stance X cannot pass: reason" | Unit test |
| Keys pressed in the Gazebo window replaced each other, so forward + strafe + rotate could not be combined | One value and time-out per axis | Unit test |
| Dashboard radar showed a ramp as BLOCKED | Raised points that fit a plane of 3-30 deg are reported as SLOPE with the grade | Unit test on synthetic scans; replay of recorded clouds |
| Soil constants were marked "from memory" | Checked against NASA/TM-20250006958 Table 1 (citing Wong 2001); they match | Source notes in `parameter_schema.yaml` |

Added:

- `dashboard/` — Flask + server-sent events page: camera, chassis twin, LiDAR sectors, explanations, cost
  terms, motor and battery gauges, whitelisted override buttons. Binds to 127.0.0.1.
- `benchmarks/run_ablation_suite.py` (four policies, interleaved repeats), `summarise_fixtures.py`,
  `generate_paper_figures.py` (four IEEE-width figures as PDF and PNG, `reports/table1_ablation.tex`),
  `generate_paper_summary.py`.
- `video_pipeline/record_conference_video.py` — 1920x1080, 60 fps, composed in simulation time; three
  body-mounted cameras enabled with `chase_cameras:=true`.
- Launch arguments `chase_cameras`, `chase_camera_rate`.
- 10 tests (`tests/test_v2_tools.py` and additions); total 247.

## V2 complete research suite, Commit 3 — decisions, explanations, camera classifier, autonomous tour (2026-11-13)

- `morph_sim/decision_core.py`: stance cost (tip-over margin, winding temperature rise, traction utilisation,
  clearance, morph time) and the lead-time trigger `d / v <= t_morph + 1 s`.
- `decision_synthesizer` node: `/morph/decision_explanation` (text) and `/morph/cost_breakdown` (numbers).
- `vision_core.py` / `terrain_vision` node: colour class under the robot, next class and its distance.
- `tour_core.py` / `autonomous_tour` node: ten-zone tour with policies `fixed_compact`, `fixed_extended`,
  `reactive`, `proposed`; failed zones are recorded and skipped.
- Arena renamed to `arena:=tetra`; tour ramp 16 deg; sand and mud friction 0.35 / 0.25; default rail holding
  force 25 N; S-curve jerk limit 6 m/s^3; steering rate 3.2 rad/s; A/D strafe, Q/E rotate.

## V2 complete research suite, Commit 2 — on-the-fly morphing, rail holding, active suspension (2026-11-13)

Report: `reports/v2_commit2.md`.

| What was wrong | What was fixed | Verified by |
|---|---|---|
| The rails gave no holding force when no shape change was active (commanded length followed the measurement), and the command was tied to the mean stage position so slipped stages dragged the others in | Hold the last commanded length; per-joint bounding only | Sim: 15 deg ramp now climbed in Extended with self-locking rails; unit tests |
| Shape changes forced a slow-down to 0.15 m/s | `continuous` morph policy (opt-in) | Sim at 0.3 m/s: lateral drift 0.6-0.7 mm, heading 0.0 deg; unit tests |

Added: explicit coupled kinematics functions, corner-height suspension presets and topic, thermally
governed +20 % torque boost, one-wheel-at-a-time trench stepping planner, 19 tests (196 total).

Measured: head-on trench crossing with suspension pitch works for 100 mm only; yawed crossing gets
Extended over 100/130/155 mm; Compact stalls on the 15 deg ramp that Extended climbs; morphing
while crabbing does not work.

## V2 complete research suite, Commit 1 — chassis, arena, terramechanics (2026-11-13)

Report: `reports/v2_commit1.md`.

- `chassis:=trimmed` robot variant (default): Compact chassis 330 mm long in the crab pose with drive
  units inward (CAD model 469 mm). `chassis:=cad` keeps the original.
- Envelope collision shapes so the collision model matches the visible robot.
- `stage_hold:=self_locking` option (off by default) for rails that hold their length.
- `arena:=v2`: trenches 100/130/155 mm, loose sand, mud, ramps 12/15/18/20/22 deg, 620 mm corridor,
  380 mm pinch, rocks, rubble (`tools/build_v2_arena.py`, seed 42).
- `MorphTerramechanics` Gazebo plugin with a Python reference model and three literature soil sets.
- Launch arguments `chassis`, `stage_hold`, `spawn_x`, `spawn_y`, `spawn_yaw`.
- ROS discovery is localhost-only by default: a foreign ROS graph on the shared network corrupted
  one run with its `/clock`.
- 20 tests (total 177).

## V2 Phase 0 — audit and gap fixes (2026-11-13)

Reports: `reports/audit_phase0.md`, `reports/baseline.md`.

### Fixed

| What was wrong | What was fixed | Verified by |
|---|---|---|
| Body crept 21–170 mm during a standstill morph: wheel feedforward used the commanded stage rate (4.5 mm/s) while the stages move at 3.2 mm/s, and the wheel speed loop could not follow | Feedforward from the measured stage rate with a 150 ms decay envelope and a commanded-direction floor; wheel-angle tracking while the stages move (`control.py`, `terrain_manager.py`) | Sim: 170 → 3.3 mm, 118 → 0.3 mm, 21 → 0.2 mm, 35 → 0.3 mm; unit tests |
| Morph speed gate (0.22 m/s) tested only the commanded speed | Gate also checks the encoder body-speed estimate | Unit tests. Sim: stages still start at up to 0.25 m/s by ground truth — partly fixed |
| Speed clamp during a shape change was per axis (diagonal 0.21 m/s) | Planar vector clamp at 0.15 m/s | Unit test |
| "Transition pending" sat 0.09 mm from latching on for ever (servo rest error 0.41 mm vs 0.5 mm threshold) | Completion latch with hysteresis and a stall rule | Sim and unit tests |
| Crab travel with a small yaw or stage component demanded a 180° re-steer at the ±90° limit | Flip hysteresis (0.12 rad) in `optimized_wheel` | Sim and unit tests |
| Robot textures failed to load when the workspace path contains spaces (60 errors per launch) | Plain absolute texture paths (`assets.py`) | Launch log 60 → 0 errors; regression test |
| Contact sensors dead in `arena:=competition` (wrong Gazebo topic names) | World-scoped sensor topics (`bridge_competition.yaml`) | Live census 0 → 502 Hz; regression test |
| `/morph/status` had no timestamp and ran at 16.7 Hz | `sim_time_s` field; 20 Hz | Live census 20.0 Hz |
| No signal for a jammed morph | `morph_stalled` in `/morph/status` | Unit test |
| Stale values: 40 mm default wheel radius, `model_assumptions.yaml`, Jazzy text, `validate_linux.sh` sourcing a missing distro, undeclared dependencies, non-executable `run_smart_fast.sh` | Corrected | Unit tests, syntax checks |

### Added

- `tools/baseline_v2.py`, `tools/baseline_v2_report.py`: motion, stopping, jerk, morph, ramp and
  slip protocol with plots.
- `tools/topic_census.py`: live topic, QoS, rate, frame and TF census.
- `tools/phase0_feasibility.py`: geometry, CoM and arena-spec calculations from the URDF.
- `tests/test_morph_compensation.py` (19 tests), `tests/test_phase0_gaps.py` (4 tests).
  Total 134 → 157 passing.
- New status fields: `sim_time_s`, `body_speed_estimate`, `stage_rate_measured_m_s`,
  `wheel_angle_lock`, `transition_settled`, `morph_stalled`.

### Behaviour changes to be aware of

- `optimized_wheel` default radius is now 0.075 m (was 0.04 m) and it takes an optional previous
  steering angle.
- `AdaptationCore.step` takes an optional `velocities` argument. Without it the old commanded-rate
  feedforward is used.
- A shape change requested while crabbing first re-steers each axle pair by up to 170°; the body
  stops while that happens.

### Not changed

- No `[ORIGINAL]` value from the V2 brief, and no assumed robot parameter, was altered.
- Open spec conflicts S1–S12 are listed in `reports/audit_phase0.md` and wait for a decision.
