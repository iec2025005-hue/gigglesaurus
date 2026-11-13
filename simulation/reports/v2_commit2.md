# V2 complete research suite — Commit 2: on-the-fly morphing, rail holding, active suspension

Date: 2026-11-13. Branch `v2/complete-research-suite`. Simulation of an assumed-parameter model.
Every measurement is a single run (n = 1). All V2 runs use `stage_hold:=self_locking` unless stated.
Data: `reports/baseline_data/v2_commit2_results.json`, raw runs in `validation/performance/v2s2_*`.

## 1. What was built

| Item | Where |
|---|---|
| Coupled swerve + variable-wheelbase kinematics as explicit functions (`wheel_position`, `wheel_velocity`) | `morph_sim/control.py` |
| Morph policy `gated` (default, unchanged) or `continuous` (shape changes at driving speed, no slow-down) | events `morph_gated` / `morph_continuous`, manager parameter `morph_policy` |
| Rail holding: the rails now hold their commanded length | `morph_sim/control.py` |
| Corner-height suspension commands (±15 mm): presets `pitch_nose_up`, `pitch_nose_down`, `lift_fl/fr/rl/rr`, `suspension_level`, and topic `/morph/suspension/corner_heights` | `morph_sim/control.py`, `morph_sim/terrain_manager.py` |
| Peak torque boost, +20 %, requested by the controller, fading with winding/driver temperature | `morph_sim/design_physics.py`, events `boost_on` / `boost_off` |
| One-wheel-at-a-time trench stepping planner | `morph_sim/trench_crossing.py` |
| Tests: 19 new (196 total) | `tests/test_v2_motion.py` |

The kinematics, with `v = (vx, vy, ωz)` in the body frame, stage offset `d`, wheelbase `L = 0.54 + 8d`:

    r_i(L) = ( ±(0.27 + 4d), ±0.27 )                       wheel i contact point
    v_wheel,i = v_body + ω × r_i(L) + (∂r_i/∂L) L̇          with ∂r_i/∂d = (±4, 0)
    δ_i = atan2(v_wheel,i,y , v_wheel,i,x),  ω_i = |v_wheel,i| / R   (folded into ±90° with a sign)

`L̇` is the measured rail rate (50 ms low-pass), not the commanded one.

## 2. A defect found and fixed: the rails went limp

Two behaviours let the telescopes collapse under load, independent of their force limit:

1. When no shape change was active the commanded length was set equal to the measured length, so the
   rails gave no holding force at all. This was in the original controller (it applied whenever the
   robot drove faster than 0.22 m/s) and my Phase 0 completion latch made it apply at rest too.
2. The command was bounded to the *mean* stage position ± 1 mm. A stage that had not moved was then
   commanded inward once its neighbours slipped, at the full 6 mm/s.

Both are removed: the last commanded length is held, and each joint is bounded against its own
position by the existing per-joint limiter. **The Phase 0 audit and my earlier Commit 1 note blamed
the 8 N stage force alone; that was incomplete.** With holding fixed, the force limit still matters:

| 15° ramp, Extended, existing drive limits | Result |
|---|---|
| Rails back-drivable (8 N per stage) | stalls at the foot; wheelbase 672 → 459 mm |
| Rails self-locking (60 N per stage, assumed) | climbs; wheelbase dips to 568 mm and recovers |

Even at 60 N the wheelbase dips by up to 145 mm during the hardest part of a climb.

## 3. On-the-fly morphing (continuous policy, 0.3 m/s, open loop, flat tarmac)

Each row compares a drive with a shape change against the identical drive without one.

| Drive | Shape change while moving | Lateral deviation | Heading difference | Along-track difference | Slowest speed |
|---|---|---|---|---|---|
| Straight, 14 s | 540 → 672 → 540 mm | 0.6 mm (reference 0.5 mm) | 0.0° | +62 mm | 0.281 m/s |
| Straight, 16 s | 540 → 237 mm | 0.7 mm (reference 0.8 mm) | 0.0° | +70 mm | 0.284 m/s |
| Arc, ωz 0.2 rad/s, 14 s | 540 → 672 → 540 mm | — | 1.9° | 52 mm path difference | 0.275 m/s |
| Crab, 14 s | 540 → 672 → 540 mm | — | — | **fails**: 1.07 m travelled instead of 3.58 m | 0.0 m/s |

- The commanded speed stays at 0.3 m/s throughout; the gated policy slowed to 0.15 m/s.
- Sideways scrub is below 1 mm and there is no heading change on a straight line.
- The "< 5 mm drift" target is met laterally, **not** along the track: the morphing run ends
  62–70 mm further on. During a shape change the controller tracks wheel angle, which removes the
  2 % speed deficit of the plain speed loop (0.298 vs 0.293 m/s). The morphing run is the one closer
  to the command.
- **Morphing while crabbing does not work.** With steering limited to ±90° the wheels of one axle
  must re-steer by about 170° before the rails can move, and the body stalls. A shape change has to
  finish before the robot starts to crab. The pinch manoeuvre must be sequenced that way.

## 4. Suspension pitch: how much load it moves

| Corner heights | Body pitch | Front axle | Rear axle |
|---|---|---|---|
| Level | 0.0° | 49.2 N | 48.8 N |
| Front +15 mm, rear −15 mm | 2.5° nose up | 47.7 N | 50.3 N |
| Front −15 mm, rear +15 mm | 2.5° nose down | 50.5 N | 47.3 N |

±15 mm moves about 1.5 N, 3 % of an axle load (calculated 1.9 N).

## 5. Ramps

| Ramp | Shape | Rails | Suspension | Boost | Result | Time | Energy |
|---|---|---|---|---|---|---|---|
| 15° | Extended | self-locking | level | off | **climbs** | 39.0 s | 604 J |
| 15° | Extended | self-locking | nose down | off | climbs | 42.3 s | 667 J |
| 15° | Extended | self-locking | level | on | climbs | 38.3 s | 550 J |
| 15° | Extended | self-locking | nose down | on | climbs | 38.8 s | 554 J |
| 15° | Extended | back-drivable | level | off | stalls at the foot | — | — |
| 15° | Compact | self-locking | level | off | stalls at the foot; pitch 19°; wheelbase pulled to 309 mm | — | — |
| 18° | Extended | self-locking | level | on | stalls part-way (still creeping at the 100 s limit) | — | 5984 J |
| 18° | Extended | self-locking | nose down | on | **climbs** | 38.5 s | 622 J |

- Extended climbs 15° and Compact does not. On the slope Compact carries about 83 % of its weight
  on the rear axle (calculated), so the front wheels run out of grip before they run out of torque.
  This is the load-equalising effect the brief describes, and wheelbase delivers it.
- On 15° the suspension pitch and the boost are not needed; the boost saves about 9 % energy.
- On 18° the nose-down pitch plus boost climbed and boost alone did not. The calculated limit with
  +20 % torque is 17.7°, so 18° sits at the margin where small effects decide. One run each: this
  needs repeats before it is claimed, and I have not established the mechanism.
- 20° and 22° were not attempted: they need 36 N and 39 N of drive force against 32 N available.
- Peak winding temperature over all runs was 36.3 °C (in the run that sat stalled for 100 s). The boost is
  never limited by heat in this model.

## 6. Trenches (100 / 130 / 155 mm)

| Approach | Shape | Extras | Speed | Trenches crossed | Peak pitch |
|---|---|---|---|---|---|
| Head-on | Extended | none | 0.3, 0.6 m/s | none | 3° |
| Head-on | Extended | nose up | 0.3, 0.6 m/s | none | 3° |
| Head-on | Extended | nose up + boost | 0.6 m/s | 100 | 5° |
| Head-on | Normal | none | 0.6 m/s | none | 5° |
| Head-on | Compact | none | 0.6 m/s | none | 20° |
| Yawed 30°, plain driving | Extended | none | 0.2 m/s | 100, 130 | 8° |
| Yawed 30°, plain driving | Extended | boost | 0.3 m/s | **100, 130, 155** (20 s) | 5° |
| Yawed 40°, plain driving | Compact | none | 0.2 m/s | 100, 130 | **31°** |
| Yawed 32°, stepping | Extended | wheel lift | 0.1 m/s | **100, 130, 155** (98 s) | 5° |
| Yawed 27°, stepping | Normal | wheel lift | 0.1 m/s | **100, 130, 155** (100 s) | 7° |
| Yawed 41°, stepping | Compact | wheel lift | 0.1 m/s | 100 only | 28° |

- **The manoeuvre in the brief (head-on, Extended, front +15 / rear −15, boost) crosses only the
  100 mm trench.** Both wheels of an axle enter together and the axle load sits on the gap edges;
  the suspension pitch changes that load by 3 %.
- **Crossing with the body yawed works.** The four wheels meet the gap at four different times, so
  three always carry the robot. Extended crosses all three trenches, including 155 mm, which is
  wider than the wheel.
- Stepping matches the geometry: the wheels must be further apart along the direction of travel
  than one lift window. The largest spacing is 178 mm (Compact), 241 mm (Normal), 284 mm (Extended);
  the windows are 160 / 190 / 215 mm. Compact can therefore only step over the 100 mm trench, and
  that is what happened.
- Normal also steps over all three. A trench that only Extended can cross would be 185–220 mm wide.
- Compact driven through at an angle pitches to 28–31°, beyond its 22° static tip angle. It did not
  roll over in these runs but should be treated as unsafe.

## 7. What this means for the brief

| Brief | Measured |
|---|---|
| < 5 mm drift while morphing and driving | lateral 0.6–0.7 mm; along-track +62–70 mm against an un-morphed run |
| Morph while driving *and steering* | straight and arcs yes; crab no |
| Head-on trench crossing by suspension pitch + rear torque | 100 mm only, and only with the boost |
| Ramps 15°–22° by CoM redistribution + 20 % boost | 15° yes (Extended alone is enough); 18° once, at the margin; 20–22° out of reach |
| Rear motors throttle thermally | hottest winding in any run 36.3 °C (derating starts at 75 °C) |

## 8. Open items

- Rails still give up to 145 mm under peak load at 60 N. The right holding force is a hardware
  question (a lead screw is normally self-locking); 60 N is an assumption.
- Every result is one run. The ablation suite must repeat them.
- The stepping sequence stops for 3 s per wheel; it is safe but slow (98 s for three trenches).
  Plain yawed driving with boost took 20 s.
- The trench planner uses ground-truth pose and known trench positions. LiDAR gap detection is
  Commit 3.
