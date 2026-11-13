# Changelog

Format: what was wrong, what was fixed, how it was verified. Simulation only; all robot
parameters are assumed values.

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
