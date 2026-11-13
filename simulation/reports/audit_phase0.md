# MORPH V2 — Phase 0 gap and shortcoming audit

Date: 2026-11-13. Scope: `simulation/src/morph_sim` (the running package), the workspace scripts,
and the V2 execution brief itself. Measurements are in `reports/baseline.md`; calculations are
reproducible with `src/morph_sim/tools/phase0_feasibility.py`
(output: `reports/baseline_data/phase0_feasibility.json`).

**Status: Phase 0 is complete. Workstream A has not started. Eleven spec conflicts need a decision
(Section 3) before the arena and the motion limits can be built.**

Everything here is simulation of an assumed-parameter model. "Measured" means measured in
simulation, once (n = 1) unless stated. "Calculated" means static rigid-body arithmetic.

---

## 1. What the audit found, in short

1. **All four arena zones conflict with the robot as modelled.** A two-axle rover cannot bridge a
   trench wider than its wheel (150 mm) at any wheelbase; the body is 700–800 mm wide, not 540 mm;
   drive torque limits climbing to about 15° (measured: stalls on the 14.8° ramp); and no slip
   occurs at mu 0.35 (measured: none even at mu 0.22).
2. **Several numbers in the brief do not match the simulation.** Morph takes 5–17 s, not about 2 s.
   Top speed is 0.76 m/s, not 2.0 m/s. CoM height is 291 mm, not 340 mm. GUI RTF is 1.0, not 0.34.
3. **Much of what the brief treats as existing does not exist.** There is no `shape_planner.py`,
   no `hazard_grid.py`, no `_TERRAIN_ACCEL`, no camera terrain classifier, no
   `/morph/vision/telemetry`, no `/morph/wheel_speed_mismatch`, and the LiDAR code cannot see
   gaps or measure passage width.
4. **Eleven implementation gaps were fixed and verified**, the largest being that the body crept up
   to 170 mm during a standstill morph (now 3 mm). Tests went from 134 to 157, all passing.

---

## 2. Implementation gaps — fixed (rule 3a)

Format per finding: Severity | Type | What was wrong | Why it matters | Fix | How verified | Decision?

### [I1] High | Implementation gap | Body creeps during a standstill morph
- **What was wrong:** `control.py` fed the wheels the *commanded* stage rate (4.5 mm/s) while the
  position servo actually moves the stages at 3.2 mm/s, and the wheel speed loop is too soft to
  follow at these speeds (it needs about 0.19 rad/s of error just to overcome rolling resistance).
  The stage actuators dragged the wheels instead; whichever axle resisted less moved.
- **Why it matters:** the body moved 21–170 mm while "standing still", always forward. A morph in
  front of a trench or a wall would end in the wrong place.
- **Fix:** wheel feedforward now uses the measured stage rate (50 ms low-pass, 150 ms decay envelope
  after the command ends — this is the A4 envelope), with a small commanded-direction floor so
  wheels steer and lead before the stages move; and while the stages move the wheel *angle* is
  tracked (gain 4 s⁻¹, bounded 0.6 rad/s). `terrain_manager.py` passes joint velocities in.
- **How verified:** sim, before → after: 672→237 mm 170 → 3.3 mm; 237→672 mm 118 → 0.3 mm;
  540→672 mm 21 → 0.2 mm; 672→540 mm 35 → 0.3 mm. Morph duration unchanged. 19 unit tests in
  `tests/test_morph_compensation.py` (shared with I2–I5 and I9).
- **Decision:** N.
- **Note:** my first version of this fix caused two regressions (I4, I5) and one deadlock (morph
  while crabbing never started). All three were caught by re-running the protocol, fixed, and
  re-verified in run `v2_fix3_full_headless_1`.

### [I2] Medium | Implementation gap | Morph speed gate checked the setpoint, not the robot
- **What was wrong:** the 0.22 m/s gate tested the ramped command only. Stages started with the
  body at 0.241 m/s (ground truth).
- **Fix:** the gate now also requires the encoder body-speed estimate (mean of wheel contact
  velocities; yaw and stage terms cancel) to be below 0.22 m/s.
- **How verified:** unit tests. In sim the ground-truth speed at stage start was 0.204 and 0.251 m/s
  — **not fully fixed**: the wheels slow before the body does, so the encoder estimate passes early.
- **Decision:** Y — see S6/A3. The proposed deceleration-rate condition (|dv/dt| < 0.15 m/s²) is
  what closes this.

### [I3] Low | Implementation gap | Transition speed clamp was per axis
- **What was wrong:** while a shape change is pending the request was clamped to ±0.15 m/s on x and
  y separately, so a diagonal request reached 0.21 m/s and changed direction.
- **Fix:** the planar vector is scaled to 0.15 m/s. **Verified:** unit test. **Decision:** N.

### [I4] Medium | Implementation gap | "Transition pending" could latch on for ever
- **What was wrong:** pending meant |target − stage| > 0.5 mm, but the servo rests 0.3–0.5 mm short
  in Compact (measured 0.41 mm before any change). One tenth of a millimetre more and the robot
  stays capped at 0.15 m/s indefinitely. My first fix pushed it over that edge.
- **Fix:** completion latch with hysteresis (in at 0.5 mm, out at 1.2 mm) and a stall rule: stages
  at rest for 0.8 s inside 1.2 mm count as complete.
- **How verified:** sim (speed returns to 0.59 m/s after a moving morph), 3 unit tests. **Decision:** N.

### [I5] Medium | Implementation gap | 180° steering flip at the ±90° limit
- **What was wrong:** steering is limited to ±90°, and `optimized_wheel` picked the representation
  from fixed quadrants. During crab travel any small backward component (yaw correction, stage
  motion, noise) put the direction just past 90° and demanded a 180° re-steer (3 s at 1 rad/s).
- **Fix:** hysteresis — within 0.12 rad past the limit the wheel stays at the limit. It is disabled
  while a shape change needs the exact direction.
- **How verified:** unit tests; sim crab + yaw holds 0.165 m/s and 0.088 rad/s. **Decision:** N.
- **Remaining:** a shape change while crabbing still needs one 170° re-steer per axle pair (the
  body stops for it). That is inherent in ±90° steering. See S2.

### [I6] Medium | Implementation gap | Robot textures fail to load in this workspace
- **What was wrong:** `assets.py` wrote texture paths with `Path.as_uri()`, which percent-encodes
  the spaces in `swirl drive new`; the renderer does not decode them. 60 errors per launch and an
  untextured robot. The existing test decoded the URI itself, so it passed.
- **Fix:** plain absolute paths. **Verified:** launch log errors 60 → 0; regression test.
- **Decision:** N.

### [I7] High | Implementation gap | Contact sensors dead in the `competition` arena
- **What was wrong:** `bridge_competition.yaml` bridged `/morph/contacts/*`, but Gazebo publishes
  contacts on the world-scoped sensor topic. Wheel-contact flags stayed false and the underbody
  e-stop could not fire in that arena.
- **Fix:** scoped topic names for world `morph_arena`. **Verified:** live census, 0 → 502 Hz on all
  four wheels; regression test for both bridge files. **Decision:** N.

### [I8] Low | Implementation gap | Status topic had no timestamp and ran at 16.7 Hz
- **What was wrong:** `/morph/status` carries no stamp, and the `>= 0.05 s` test with a 10 ms timer
  gave 60 ms periods.
- **Fix:** `sim_time_s` field added; bound set just below five periods. **Verified:** census 20.0 Hz.
- **Decision:** N.

### [I9] Low | Implementation gap | No signal for a failed morph
- **What was wrong:** nothing reported stages that are commanded but not moving. Workstream E's
  "on MORPH failure: revert and recover" needs it.
- **Fix:** `morph_stalled` in `/morph/status` after 3 s at rest outside the completion band; the
  wheel lead stops while it is set. **Verified:** unit test only. **Decision:** N.

### [I10] Low | Implementation gap | Stale distro and wheel values
- **What was wrong:** `optimized_wheel` defaulted to a 40 mm radius (robot is 75 mm);
  `model_assumptions.yaml` (unreferenced) still said 40 mm wheels, 1800 N/m, Jazzy; the launch
  docstring and `package.xml` said Jazzy/Humble; `tools/validate_linux.sh` sourced
  `/opt/ros/jazzy` (absent) and aborted; `run_smart_fast.sh` was not executable; `xterm`,
  `controller_manager_msgs`, `rosidl_runtime_py` were undeclared.
- **Fix:** all corrected. **Verified:** unit tests, `bash -n`, XML/YAML parse. **Decision:** N.

### [I11] Low | Implementation gap | No reproducible measurement tooling for V2
- **Fix:** added `tools/baseline_v2.py`, `baseline_v2_report.py`, `topic_census.py`,
  `phase0_feasibility.py`. They run under the existing `run_benchmark_case.py`. **Decision:** N.

---

## 3. Spec conflicts — decisions needed (rule 3b)

Nothing in this section has been changed. `[ORIGINAL]` values stay in force until you decide.

### [S1] Critical | Spec conflict | Zone 1: a longer wheelbase does not bridge a trench
- **Calculation:** the rover has 4 wheels on 2 axles (the "6" is telescope stages per side). When
  the front axle is over a gap, support is one line under the rear axle, and the CoM is ahead of
  it for every wheelbase, so the nose drops (pitch acceleration 12–17 rad/s²; the Extended front
  axle is about 0.2 m down after 0.2 s). A rigid 150 mm wheel spans at most a 150 mm gap (drop 19 mm at 100 mm, 48 mm at
  140 mm). 25/35/45 cm and 25/40/55 cm are all wider. Ground clearance is 325 mm, so the belly
  does not help.
- **What does depend on wheelbase:** straddling a trench that runs *along* the direction of travel
  while crabbing. The free gap between front and rear wheels is 192 mm Compact, 495 mm Normal,
  627 mm Extended. Driving straight, the gap is the fixed 495 mm for every shape.

  | Trench width | Compact (crab) | Normal (crab) | Extended (crab) |
  |---|---|---|---|
  | 25 cm | fails (−29 mm/side) | +122 mm/side | +188 mm/side |
  | 35 cm `[ORIGINAL]` | fails | +72 | +138 |
  | 45 cm `[ORIGINAL]` | fails | +22 | +88 |
  | 40 cm `[PROPOSED]` | fails | +47 | +113 |
  | 55 cm `[PROPOSED]` | fails | fails (−28) | **+38** |

- **Options:**
  1. **(Recommended)** Make Zone 1 a longitudinal trench crossed in crab orientation, widths
     25/40/55 cm. Only Extended straddles 55 cm: a clean "morph needed" proof, and it matches the
     Workstream F wording. Cost: 38 mm margin per side needs good lateral tracking.
  2. Keep transverse trenches but narrow them to ≤ 10 cm. Every shape crosses; no morph proof.
  3. Keep 25/35/45 cm transverse trenches as a *negative* demo (every shape fails) — not useful
     for the paper claim.
- **How verified:** calculation only. Not simulated: no world contains a trench.
- **Decision:** Y.

### [S2] Critical | Spec conflict | Zone 2: the robot is much wider than 540 mm
- **Calculation (from URDF and meshes):**

  | Entry | Collision model | Visible model |
  |---|---|---|
  | Straight, any shape | 700 mm (underbody box) | 800 mm |
  | Crab, Compact | **282 mm** | 374 mm at wheel height, 469 mm above 300 mm |
  | Crab, Normal / Extended | 585 / 717 mm | 778 / 976 mm |

  540 mm is the wheel-centre track; 585 mm over the tyres. The reviewer's "about 29–31 cm" matches
  the collision model only.
- **Consequence:** in Compact crab the *collision* model clears 32 cm by 19 mm per side and 34 cm by
  29 mm per side, but the *visible* robot is 54–149 mm wider than the passage and would be seen
  passing through the walls. The collision model under-represents the chassis (the telescope
  stages and motors have no collision shapes).
- **Options:**
  1. **(Recommended)** Passage 52 cm, Compact crab only. Add collision shapes that match the
     visible body (469 mm) so physics and picture agree; margin 25 mm per side. Normal (778 mm)
     and Extended (976 mm) cannot enter, so the morph is still required.
  2. Keep 32/34 cm with walls no taller than 0.14 m (kerbs), so only the legs pass between them and
     the body goes over. Honest physically, but it is a kerb slalom, not a "labyrinth of boulders".
  3. Keep 34 cm and accept that the picture clips. Not recommended for an exhibition or a paper.
  4. Corridor ≥ 85 cm for straight entry (800 mm + margin). No morph needed, so no proof.
- **How verified:** calculation from geometry; crab travel and crab + morph measured in sim on flat
  ground (Compact crab holds 0.32 m/s). No passage exists in any world.
- **Decision:** Y.

### [S3] Critical | Spec conflict | Zone 3: 15–25° cannot be climbed, and Extended does not hold
- **Calculation:** drive force limit 4 × 0.5 Nm / 0.075 m = 26.7 N; weight 98.1 N. Grade limit
  asin(26.7/98.1) = **15.8°** with no losses; about 14.4° with the measured 2.2 N rolling
  resistance. 20° needs 0.63 Nm per wheel, 25° needs 0.78 Nm. `parameters.py` rejects limits above
  0.5 Nm; the motor model itself could give 1.4 Nm at its 2 A current limit.
- **Measured:** Extended and Compact both climb 4.9° and 9.8° and stall on the 14.8° ramp.
- **Second problem (measured):** under slope load the telescopes are pushed in. Commanded 672 mm,
  actual 570 mm on 9.8° and 454 mm on 14.8°. Stage actuators are limited to 8 N each (URDF).
- **Tip angle:** static pitch tip with the URDF CoM (291 mm) is 21.9° Compact, 42.7° Normal, 49.0°
  Extended. The "about 19° / 45°" in the brief comes from the assumed 340 mm.
- **Options:**
  1. **(Recommended)** Ramp of 8–12° (climbable with margin), plus a separate static tilt-table
     test up to 25° for the tip comparison (analytical and simulated, robot parked, not driving),
     and raise the stage holding force so the commanded shape is kept (a lead-screw stage would be
     self-locking; this changes an assumed actuator parameter).
  2. Raise the drive torque limit to about 0.9 Nm so 25° is climbable (within the motor's 1.4 Nm
     current-limited torque). Changes an assumed parameter, the validated bound in `parameters.py`,
     and every earlier stopping result.
  3. Keep 15–25° as an analytical-only zone. No simulated climb.
- **How verified:** calculation and two sim runs (`v2_ramp_extended_1`, `v2_ramp_compact_1`).
- **Decision:** Y (ramp angle; torque limit; stage holding force).

### [S4] High | Spec conflict | Zone 4: no slip at mu 0.35
- **Calculation:** per wheel, drive force limit 6.67 N against friction 0.35 × 24.5 N = 8.58 N.
  The reviewer's estimate is confirmed. Full torque slips only below mu 0.27; the controller's
  1.2 m/s² ramp slips only below mu 0.12.
- **Measured:** median wheel slip exceeded 0.15 for 0.02 s (sand, 0.425), 0.05 s (mud, 0.2975) and
  0.09 s (snow, 0.22) of a 5 s full-speed step; mean slip about 0.02 on all three.
- **Note:** "derate wheel torque" (listed as an option in the brief) makes slip *less* likely.
- **Options:**
  1. **(Recommended)** Label the zone "analytical slip demo": keep mu 0.35, show the controller-
     level derate on an injected or analytically predicted slip signal, claim nothing physical.
  2. Lower mu to about 0.10 on a 5–8° sand slope so the wheels really spin; measure and report.
  3. Raise the torque limit (as S3 option 2); then mu 0.35 slips above about 3.4 m/s².
- **Speed cap:** the `[ORIGINAL]` 0.9 m/s cap is above the 0.76 m/s the robot can reach; see S5.
- **Decision:** Y.

### [S5] High | Spec conflict | Speeds of 0.9, 1.10 and 2.0 m/s are unreachable
- **Calculation:** no-load wheel speed (12.6 V − 2 V) / (0.02 × 50) = 10.6 rad/s = 0.795 m/s.
  **Measured:** 0.760 m/s with the mode cap bypassed; 0.60 m/s is the highest mode cap (track).
- **Options:** (1) **(Recommended)** scale the brief's speeds to the model: burst 0.6 m/s, sand cap
  0.18 m/s (existing sand mode), example cap ≤ 0.6. (2) Change gear ratio or supply voltage so
  2.0 m/s is reachable — this trades away torque and makes S3 worse. (3) Keep the numbers as
  labels only (not recommended; the dashboard would show caps the robot never reaches).
- **Decision:** Y.

### [S6] High | Spec conflict | Morph takes 5–17 s, not about 2 s
- **Measured:** 540↔672 mm 5.0 s; 540↔237 mm 11.9 s; 237↔672 mm 16.7 s (3.2 mm/s per stage).
  URDF limit 6 mm/s gives 9.1 s at best; 2 s needs 27 mm/s (4.5× the limit).
- **Effect on C3:** see the comparison table (Section 6). With real durations the `[PROPOSED]`
  trigger distance is 3–10 m, beyond the 2 m LiDAR assessment window.
- **Options:** (1) **(Recommended)** keep the actuator, fix the servo tracking so the commanded
  4.5 mm/s is delivered (12.1 s full, 8.4 s to Compact, 3.7 s to Extended), and plan around it:
  slow to 0.15 m/s, morph while creeping. (2) Raise the stage velocity limit to reach about 2 s —
  changes an assumed actuator parameter and needs the stage force raised too. (3) Leave 3.2 mm/s.
- **Decision:** Y.

### [S7] Medium | Spec conflict | Acceleration and jerk table (item 1.1G)
- **Calculation:** torque-limited acceleration is 2.67 m/s² (about 2.4 after rolling resistance).
  Road 3.5 and Grass 2.8 m/s² cannot be reached. The table does not follow J = 2.5 × A:

  | Terrain | A_max | J_max in table | 2.5 × A | Ratio J/A | A reachable |
  |---|---|---|---|---|---|
  | Road | 3.5 | 8.0 | 8.75 | 2.29 | no |
  | Grass | 2.8 | 6.0 | 7.0 | 2.14 | no |
  | Sand | 1.5 | 4.0 | 3.75 | 2.67 | yes |
  | Mud | 1.2 | 3.0 | 3.0 | 2.50 | yes |
  | Snow | 0.8 | 2.0 | 2.0 | 2.50 | yes |

- **Also:** `_TERRAIN_ACCEL` does not exist. Today one value, 1.2 m/s², is used everywhere.
- **Options:** (1) **(Recommended)** per-terrain table with A capped at 2.2 m/s² for Road and Grass
  and J = 2.5 × A throughout (5.5 / 5.5 / 3.75 / 3.0 / 2.0). (2) `[ORIGINAL]`: J = 8.0 m/s³ with the
  existing 1.2 m/s² — I will implement this until you decide. (3) The table as written, knowing
  Road/Grass saturate the motors.
- **Decision:** Y.

### [S8] Medium | Spec conflict | A2, A5, A6 details
- **A2:** with the proposed deadband, cos² only acts between 0.03 and 0.10 rad, where it is 0.999
  to 0.990. The scaler is therefore a hard gate at 0.10 rad with a 0.99 → 0 step. Options:
  (1) **(Recommended)** taper from 1 at 0.03 rad to 0 at 0.35 rad (cos²-shaped) and apply it
  before the jerk limiter; (2) the deadband as written, letting the jerk limiter absorb the step.
- **A5:** the tangential formula is what the existing kinematics already produce. A5 becomes a
  validation task plus the alignment gating of A2. No change needed.
- **A6:** steering is limited to ±90°, so "steer θ − π" is already the only representation, and a
  flip takes up to 3.1 s at 1 rad/s. An 80 ms hold is negligible next to that; the A2 gate already
  holds speed at zero until aligned. Recommend dropping the fixed 80 ms.
- **Decision:** Y (A2 shape; A6).

### [S9] Medium | Spec conflict | A8: where is jerk measured?
- Commanded jerk can be bounded by construction. Body jerk from odometry depends on the filter:
  the same forward step reads 8.4 m/s³ with a 0.1 s window and 24.4 m/s³ unfiltered, with no
  limiter involved. A CI gate on raw body jerk would be noise-limited.
- **Options:** (1) **(Recommended)** CI gate on commanded jerk (≤ J_max × 1.05); body jerk reported
  with a stated 0.1 s window and plotted before/after. (2) Gate on filtered body jerk as well.
- **Decision:** Y.

### [S10] Medium | Spec conflict | CoM height is 291 mm, not 340 mm
- From URDF inertials: CoM at x +1.4, y −3.9 mm, 291 mm above ground, the same for all shapes
  (front and rear modules move symmetrically). `params.yaml` says 0.34. The value 291 is below the
  300–400 mm sensitivity range in Section 8.
- **Options:** (1) **(Recommended)** keep `com_height_m: 0.34` as the declared assumption and add
  291 mm to the sensitivity study (291/300/340/400). (2) Set the parameter to 0.291.
- **Decision:** Y.

### [S11] Medium | Spec conflict | Sinkage through `height_servo.cpp` is not visual-only
- The height servo is a physical force servo on the four height joints. Lowering it changes real
  clearance, CoM and the underbody e-stop. Its travel is ±15 mm and track mode already sits at
  −8 mm; the Bekker table allows 20 mm. One out-of-range value makes `interaction_runtime` drop
  the whole command and the servo goes to fault hold after 0.2 s.
- **Options:** (1) **(Recommended)** visual-only: offset the `world → base_link` transform shown in
  RViz and the dashboard twin; do not touch the plant. (2) Physical drop through the servo, clamped
  to its remaining travel, and described as "physical ride-height emulation".
- **Decision:** Y.

### [S12] Medium | Spec conflict | Things the brief assumes exist
- Missing: `shape_planner.py`, `hazard_grid.py` (no costmap, no A*, no heading bins — item 1.1D
  has nothing to audit), `_TERRAIN_ACCEL`, a camera terrain classifier and its "5-frame filter",
  `/morph/vision/telemetry`, `/morph/wheel_speed_mismatch`, RANSAC (the plane fit is trimmed least
  squares), any gap/drop or passage-width detection (points below ground − 0.20 m are discarded).
- Present: `/morph/status`, `/morph/perception/decision`, `/morph/imu`, slip ratio per wheel in
  `/morph/physics/status`, three independent shape authorities (IMU auto-climb in `control.py`,
  ground-truth zone automation, LiDAR perception) with no arbitration beyond launch-time exclusion.
- **Effect:** Workstreams C, E and I must build these first. Re-estimate in Section 8.
- **Decision:** Y (accept the added scope, or cut: e.g. LiDAR-only decisions without a camera classifier).

### [S13] Low | Spec conflict | Smaller inconsistencies in the brief
- "Gazebo Sim Harmonic": the installed simulator is Gazebo Sim 10.5 with ROS 2 Lyrical.
- Layout: four 8 × 4 m zones with 2 m buffers give about 40 × 4 m in a line or 18 × 10 m as a 2 × 2
  loop; neither is "about 36 × 8 m". Proposed: 2 × 2 loop, 20 × 12 m including a 2 m perimeter road.
- "Physics step 2 → 4 ms": the launch file only accepts 1 or 2 ms, and RTF is already 1.0 at 2 ms.
- Slip formula in H needs `v_contact`. In simulation that is ground-truth odometry; say so.
- "Decision log < 50 ms": `/morph/status` is 20 Hz and perception 10 Hz, so define the budget as
  transport latency, not sensing-to-screen.
- **Decision:** N, unless you disagree with the proposed layout.

---

## 4. Implementation gaps — found, not fixed yet

| ID | Severity | What | Why not fixed now |
|---|---|---|---|
| D1 | High | Velocity setpoint is a per-axis rate limiter: acceleration steps (120 m/s³ per tick) and the travel direction bends during ramps | Replaced by A1 |
| D2 | Medium | Wheel speed scaled by cos(error), no alignment deadband; steering holds only below 1e-8 rad/s | Replaced by A2 |
| D3 | High | Telescopes back-driven on slopes (stage force 8 N) | Assumed parameter: S3 |
| D4 | Medium | IMU auto-climb latches Extended above 14° and never releases | Replaced by C1/C2 |
| D5 | Medium | Collision model narrower than the visible robot | Depends on S2 |
| D6 | Medium | `actual_speed`, zone automation and `world → base_link` use ground truth; no estimator | Label in dashboard; J |
| D7 | Medium | Coverage 38 %; all ROS nodes and C++ plugins untested | J |
| D8 | Low | Crab speed tracking +1 % (was +10 %), forward −2 %; soft wheel speed loop | Assumed motor-controller gains |
| D9 | Medium | `controls:=pygame` and `controls:=terminal` cannot start (pygame, xterm not installed); `setup_ubuntu.sh` installs nothing on Lyrical | Needs `sudo apt`; your call |
| D10 | Medium | `controls:=gazebo` stutters: 0.35 s key expiry is shorter than this host's 0.5 s key-repeat delay | Longer expiry also lengthens coast after release: safety trade-off |
| D11 | Low | Tools set `GZ_PARTITION` from the current directory; `arena_view.sh` does not `cd` | Will be rewritten in B |
| D12 | Low | Camera chain is dead by default (`camera_gate` output has no subscriber; RViz shows the raw topic) | Reworked in D |
| D13 | Low | Several nodes and tools publish in `finally` after shutdown (suspected traceback on Ctrl-C) | Not reproduced |
| D14 | Low | `suspension_system.cpp` has no NaN guard; `lifecycle_bridge.cpp` catch block may dereference a reset pointer | Suspected only; needs a rebuild and a test |
| D15 | Low | Watchdog and height-servo timeouts are wall-clock (0.2 s) | By design; note for slow machines |
| D16 | Low | `interaction_runtime` publishes steering zeros at shutdown (a "steer to 0" command) | Harmless today |
| D17 | Low | World name `leo_marsyard` hard-coded in tools and the launch file | Fixed with the new arena in B |

---

## 5. Unresolved items, assumptions

**Unresolved**
- Decisions S1–S12.
- I2 is only partly fixed (stages can start at 0.25 m/s by ground truth).
- Every measurement is n = 1. Repeats belong to Workstream J.
- Not measured: GUI frame rate, camera at 30 Hz, RTF with dashboard and rosbag, trench and passage
  behaviour (no fixtures exist), tip-over on a slope, anything on hardware.
- Coverage after the new tests: 40 % overall (`control.py` 99 %); ROS nodes still 0 %.
- The working tree had one uncommitted change before I started (`shlex.quote` in the launch file).
  It is needed for this path and I left it uncommitted and untouched.

**Assumptions**
- All robot parameters are the project's assumed values (mass 10 kg, 0.5 Nm per wheel, 8 N per
  stage, friction table). None is measured on hardware.
- Gazebo combines friction as min(wheel 1.2, ground mu).
- Feasibility numbers are static and rigid: no suspension deflection, tyre compliance or dynamics.
- The baseline was taken on the tarmac start pad, on this machine, with hardware GL.

Measured-vs-target numbers: `reports/baseline.md`, Section 7.

---

## 6. `[ORIGINAL]` vs `[PROPOSED]` comparison

| Item | `[ORIGINAL]` | `[PROPOSED]` | What Phase 0 found | Recommendation |
|---|---|---|---|---|
| A1 limits | J = 8.0 m/s³, A from `_TERRAIN_ACCEL` | per-terrain A/J table | `_TERRAIN_ACCEL` absent (1.2 m/s² everywhere); Road/Grass A unreachable; table ≠ 2.5 × A | Table with A ≤ 2.2 and J = 2.5 × A (S7) |
| A2 scaler | cos² per wheel | deadband 0.03 / 0.10 rad | Deadband makes cos² inert and adds a 0.99 → 0 step | Smooth taper 0.03 → 0.35 rad (S8) |
| A3 morph slow-down | allowed below 0.22 m/s | smoothstep 1.2 s down, 0.9 s up, gate on v and dv/dt | Today: clamp to 0.15 m/s at 1.2 m/s², jerk 8–9 m/s³; stages start at 0.20–0.25 m/s. Smoothstep 0.6 → 0.15 m/s gives 0.56 m/s² and 1.9 m/s³ | Adopt `[PROPOSED]` |
| A7 throttle | a × (1 − σ) | median σ, floor 0.3, low-pass, on above 0.15 | Raw slip reads up to 1.0 at every launch (ill-conditioned below 0.1 m/s); sustained slip about 0.02 | Adopt `[PROPOSED]`, ignore below 0.10 m/s |
| Zone 1 | 25/35/45 cm, Extended bridges | 25/40/55 cm | Bridging impossible at any width. As a crab straddle: `[ORIGINAL]` never needs Extended; `[PROPOSED]` 55 cm needs it (+38 mm/side) | `[PROPOSED]` widths, longitudinal trench (S1) |
| Zone 2 | 32 cm, Compact | 34 cm | Collision margin 19 vs 29 mm/side in crab; visible robot 374–469 mm does not fit either | 52 cm with matching collision (S2) |
| Zone 3 | 15–25°, Compact unsafe above about 19° | — | Stalls at 14.8°; tip angle 21.9°; Extended collapses to 454 mm | 8–12° ramp + tilt table (S3) |
| Zone 4 | mu 0.35, cap 0.9 m/s | lower mu / derate / analytical | No slip down to mu 0.22; 0.9 m/s unreachable; derating reduces slip | Analytical demo (S4) |
| C1 lattice | — | P0 safety > LiDAR > IMU > camera > slip | Consistent with today's underbody e-stop; three shape authorities need merging | Adopt |
| C3 trigger | T_lead = D/v − T_morph | d = v·T + v²/(2a) | With measured T (5–17 s) at 0.6 m/s: `[ORIGINAL]` is zero or negative for D ≤ 3 m even for the shortest morph (already late); `[PROPOSED]` gives 3.2–10.2 m. Both ignore the 0.22 m/s gate | Gate-aware: d = (v² − 0.15²)/(2a) + 0.15·T(Δshape): 0.9–2.6 m |
| Layout | — | loop about 36 × 8 m | Does not follow from 4 × (8 × 4 m) + 2 m buffers | 2 × 2 loop, 20 × 12 m (S13) |
| Latency budget | — | 200 / 100 / 50 ms | Cannot be tested yet; status is 20 Hz | Define as transport latency |

---

## 7. Risk register (updated)

| Risk | Status after Phase 0 | Mitigation |
|---|---|---|
| Jerk filter unstable at low speed | open | Unit tests with recorded traces (A8) |
| Shape flapping | open; completion latch added (I4) | C2 hysteresis |
| Dashboard WebSocket drop | open | Rosbag fallback |
| Replan latency > 200 ms | open; no planner exists yet | Stop + rotate |
| Arena geometry error | **realised: all four zones conflict** | Decide S1–S4; `phase0_feasibility.py` checks every fixture before it is built |
| RTF < 0.5 headless | **retired on this machine** (1.0); open for a machine without a GPU | Lower camera resolution |
| **New:** telescopes back-driven on slopes | open (S3) | Stage holding force |
| **New:** visible robot wider than its collision model | open (S2) | Matching collision shapes |
| **New:** morph 5–17 s limits predictive morphing | open (S6) | Gate-aware trigger; creep at 0.15 m/s |
| **New:** workspace path with spaces | one bug fixed (I6); others may exist | Regression test added |
| **New:** results rest on single runs | open | Repeats in J |
| **New:** fix to one motion path regresses another | realised twice in Phase 0 | Full protocol re-run after every control change; crab phases added |

---

## 8. Timeline re-estimate (estimate, not measured)

| Workstream | Brief | After Phase 0 | Reason |
|---|---|---|---|
| A | 3–4 d | 3–4 | A4 groundwork done; A5 is validation only |
| B | 4–5 | 5–6 | Fixtures must be redesigned (S1–S4); 80+ runs for the acceptance table |
| C | 3–4 | 6–8 | Gap/width perception and (if kept) a camera classifier must be built first |
| D | 6–8 | 6–8 | — |
| E | 5–6 | 6–8 | No planner or hazard grid exists |
| F / G / H | 1 / 2 / 2 | 1 / 2 / 2 | — |
| I | 5–7 | 5–7 | Shares the grid with E |
| J | 3 | 4 | Node-level tests start from 0 % |
| **Total** | 35–43 | **40–50** | |

---

## 9. Files changed in Phase 0

- Controller: `morph_sim/control.py`, `morph_sim/terrain_manager.py`
- Assets and config: `morph_sim/assets.py`, `config/bridge_competition.yaml`,
  `config/model_assumptions.yaml`, `package.xml`, `CMakeLists.txt`, `launch/simulation.launch.py`
  (docstring only), `tools/validate_linux.sh`, `run_smart_fast.sh` (mode)
- Tests: `tests/test_morph_compensation.py` (new, 19 tests), `tests/test_phase0_gaps.py` (new, 4
  tests), `tests/test_core.py` (one call now passes the radius explicitly)
- Tools: `tools/baseline_v2.py`, `tools/baseline_v2_report.py`, `tools/topic_census.py`,
  `tools/phase0_feasibility.py` (all new)
- Evidence: `validation/performance/v2_*` (24 runs), `reports/baseline_data/`
