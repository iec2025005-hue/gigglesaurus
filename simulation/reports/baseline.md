# MORPH V2 — Phase 0 baseline measurements

Date: 2026-11-13. Every number is a **simulation measurement of an assumed-parameter model**
(no hardware exists). Anything not measured is labelled *estimated* or *not measured*.

- Machine: i5-13450HX (16 threads), 14 GB RAM, RTX 4050 Laptop + Intel UHD. Default GL renderer is
  Mesa Intel UHD (hardware). The launch log shows NVIDIA EGL failing to initialise, so rendering is
  assumed to run on the Intel GPU; the renderer is not recorded in the run manifest.
- Software: Ubuntu, ROS 2 Lyrical, Gazebo Sim 10.5.0, Python 3.14.4, DART with Bullet collision.
- Arena `multisector`, start pad (tarmac, mu 0.765), physics step 2 ms unless stated,
  `physics_profile:=true` (electrical motor model, effort-controlled wheels), trajectory backend.
- Code state: "before" = commit `b46c8f8` + the uncommitted `shlex.quote` launch fix that was
  already in the working tree. "after" = with the Phase 0 fixes (see `audit_phase0.md`, I1–I11).
- Protocol: `src/morph_sim/tools/baseline_v2.py`, run through the existing
  `tools/run_benchmark_case.py`. Metrics: `tools/baseline_v2_report.py`. One run per case
  (n = 1); no repeat statistics yet.
- Raw data: `validation/performance/v2_*/raw.json`. Metrics and plots: `reports/baseline_data/`.

## 1. Real-time factor (RTF)

Each case: 67 s of sim time (4 s settle, 20 s driving at 0.3 m/s, 3 s stop, 40 s idle).

| Case | GUI | LiDAR | Camera | Shadows | Step | Mean RTF | Driving | Idle |
|---|---|---|---|---|---|---|---|---|
| headless default | off | on | off | on | 2 ms | 0.996 | 0.990 | 0.999 |
| headless default, repeat after fixes | off | on | off | on | 2 ms | 1.000 | — | — |
| headless, no LiDAR | off | off | off | on | 2 ms | 1.000 | 0.998 | 1.000 |
| headless + camera 320 px | off | on | 320×180 @10 Hz | on | 2 ms | 1.000 | 1.000 | 1.000 |
| headless + camera 480 px | off | on | 480×270 @10 Hz | on | 2 ms | 0.999 | 1.000 | 0.998 |
| headless + camera 640 px | off | on | 640×360 @10 Hz | on | 2 ms | 0.999 | 1.000 | 0.998 |
| headless + camera 640 px | off | on | 640×360 @10 Hz | off | 2 ms | 1.000 | 1.000 | 1.000 |
| headless, 1 ms step | off | on | off | on | 1 ms | 1.012* | 0.983 | 0.989 |
| GUI | on | on | off | on | 2 ms | 1.000 | 1.000 | 1.000 |
| GUI | on | on | off | off | 2 ms | 0.999 | 0.997 | 1.000 |
| GUI + camera 480 px | on | on | 480×270 @10 Hz | on | 2 ms | 1.000 | 0.999 | 1.000 |
| GUI + RViz (default user launch, no console) | on | on | off | on | 2 ms | 1.000 | 1.000 | 1.000 |
| full motion protocol (155 s) | off | on | off | on | 2 ms | 0.999 | — | — |

\* The mean above 1.0 comes from the 4 s settle phase (clock catch-up after start); use the per-phase values.

Reading: the world requests `real_time_factor 1.0`, so 1.0 is a cap, not headroom. Every tested
configuration holds real time on this machine.

- Target "headless ≈ 0.95×": **met** (0.99–1.00).
- "GUI measured about 0.34×": **not reproduced** — GUI runs at 1.00 here. The earlier project
  records were all taken on software rendering (llvmpipe); see `reports/STAGED_UPGRADE_STATUS.md`.
- Not measured: GUI frame rate, camera at 30 Hz, RTF with the dashboard or a rosbag recorder running,
  RTF on a machine without a GPU, physics step 4 ms (the launch file only accepts 1 or 2 ms).

## 2. Speed, acceleration, jerk, stopping (before fixes; "after" is within run-to-run noise)

Body velocity is ground-truth odometry at 50 Hz. Acceleration and jerk are central differences after
a 0.10 s centred moving average; unfiltered peaks are shown to make the sensitivity visible.

| Manoeuvre | Result |
|---|---|
| Forward step to 0.6 m/s (track mode cap) | steady 0.590 m/s, 90 % in 0.46 s |
| — peak acceleration | 1.38 m/s² filtered (1.49 unfiltered); controller ramp is 1.2 m/s² |
| — peak jerk | 8.4 m/s³ filtered (24.4 unfiltered) |
| Commanded 2.0 m/s | reaches 0.598 m/s: clamped by the mode cap |
| Motor-limited top speed (wheel target 12 rad/s, cap bypassed) | **0.760 m/s** steady, 0.787 peak |
| Stop from 0.6 m/s (command zero) | 0.155 m in 0.50 s; peak decel 1.32 m/s²; peak jerk 10.0 m/s³ filtered |
| E-stop from 0.6 m/s | 0.080 m in 0.30 s; peak decel 2.67 m/s²; peak jerk 16.9 m/s³ filtered |
| Reverse 0.6 m/s | steady −0.589 m/s |
| In-place spin 0.7 rad/s | steady 0.709 rad/s, 90 % in 1.5 s, position drift 8.8 mm (1.9 mm after fixes) |
| Crab 0.3 m/s | steady 0.329 m/s (+10 %), 90 % in 3.5 s; after fixes 0.303 m/s in 1.5 s |
| Forward 0.4 → reverse 0.4 | −0.36 m/s after 4 s |
| Whole protocol, worst case | 2.88 m/s² and 21.4 m/s³ filtered (e-stop) |

Command-level jerk today: the velocity setpoint is a rate-limited ramp, so commanded acceleration
steps between 0 and 1.2 m/s² in one 10 ms tick (120 m/s³). There is no jerk limit to compare with yet.

"Max stable speed" is 0.6 m/s by software cap and 0.76 m/s by the motor model; no instability was
seen at either. Speeds above 0.76 m/s cannot be tested with the current drive parameters.

## 3. Morph (wheelbase change) timing

| Transition (standstill) | Time, event → settled | Stage rate | Body displacement before → after fix |
|---|---|---|---|
| 540 → 672 mm | 5.1 s | 3.2 mm/s | 21 mm → 0.2 mm |
| 672 → 237 mm | **16.7 s** | 3.2 mm/s | **170 mm → 3.3 mm** |
| 237 → 672 mm | **16.7 s** | 3.2 mm/s | 118 mm → 0.3 mm |
| 672 → 540 mm | 5.0 s | 3.2 mm/s | 35 mm → 0.3 mm |

| Transition while commanding 0.6 m/s forward | Before | After |
|---|---|---|
| 540 → 237 mm: event → settled | 11.9 s | 11.9 s |
| — ground-truth body speed when the stages start | 0.212 m/s | 0.204 m/s |
| — body speed while the stages move | mean 0.149, max 0.199 m/s | same |
| 237 → 540 mm: event → settled | 11.8 s | 11.9 s |
| — ground-truth body speed when the stages start | **0.241 m/s** | **0.251 m/s** |
| — peak jerk during the slow-down (filtered) | 9.3 / 7.8 m/s³ | 8.5 / 6.7 m/s³ |

| Transition while crabbing at 0.15 m/s (added after the first baseline) | After |
|---|---|
| 540 → 237 mm | 15.5 s; the wheels re-steer by up to 170° first, so the body stops briefly |
| 237 → 540 mm | 11.1 s; same re-steer |

- The stages are commanded at 4.5 mm/s but the position servo delivers 3.2 mm/s. The URDF stage
  velocity limit is 6 mm/s (9.1 s for a full morph at best). The "about 2 s" in the V2 brief would
  need 27 mm/s per stage, 4.5× the URDF limit.
- Speed at which a morph is allowed: the controller clamps the request to 0.15 m/s while a shape
  change is pending and starts the stages once the commanded speed is below 0.22 m/s (and, after the
  fix, the encoder speed estimate too). By ground truth the stages still started at 0.25 m/s in one
  case, because the wheels slow before the body does. Strict compliance needs the deceleration-rate
  condition proposed in A3.

## 4. Extra measurements taken for the spec checks

| Test | Result |
|---|---|
| Ramp row, Extended, 0.12 m/s | climbs 4.9° and 9.8°; **stalls 0.14 m into the 14.8° ramp** (pitch 15.6°) |
| — wheelbase under slope load | commanded 672 mm; pushed to 570 mm on 9.8°, **454 mm on 14.8°** |
| Ramp row, Compact, 0.12 m/s | climbs 4.9° and 9.8°; stalls at the foot of 14.8°; wheelbase pulled 237 → 340 mm |
| — peak pitch when dropping off the 9.8° ramp end (0.13 m) | Compact 30°, Extended 11°; neither tipped over |
| Full-speed step on dry sand (mu 0.425) | median wheel slip > 0.15 for 0.02 s of 5 s; mean slip 0.017 |
| Full-speed step on mud (mu 0.2975) | 0.05 s of 5 s; mean slip 0.024 |
| Full-speed step on snow (mu 0.22) | 0.09 s of 5 s; mean slip 0.018 |
| Motor torque at standstill / cruise | 0.04 Nm per wheel ≈ 2.2 N total: an emergent rolling resistance of about Crr 0.023 |

Slip is `(v_wheel − v_body) / max(|v_wheel|, |v_body|, 0.05)`, median over wheels, counted only
while |v_body| > 0.10 m/s. The lanes are not perfectly flat (±10–20 mm relief).

## 5. Topics (live census, `tools/topic_census.py`)

| Topic | Exists | Type | Rate | QoS (publisher) | Frame / stamp |
|---|---|---|---|---|---|
| `/morph/status` | yes | `std_msgs/String` (JSON) | 16.7 Hz → **20.0 Hz after fix** | reliable, depth 1 | no stamp → `sim_time_s` added |
| `/morph/perception/decision` | yes | `std_msgs/String` (JSON) | 9.8 Hz | reliable, depth 1 | `sim_time_s` field |
| `/morph/imu` | yes | `sensor_msgs/Imu` | 100 Hz | best effort, depth 5 | `base_link`, stamp age 0 |
| `/morph/perception/points` | yes | `PointCloud2` | 10 Hz | best effort, depth 5 | `perception_link`, age 38 ms |
| `/morph/camera/image` | only with `camera_enabled:=true` | `Image` | 9.6 Hz at 480 px | best effort, depth 5 | `camera_link`, age 26 ms |
| `/morph/physics/status` | yes | `String` (JSON) | 9.4 Hz | reliable, depth 10 | `sim_time_s` field |
| `/morph/ground_truth/odom` | yes | `Odometry` | 50 Hz | best effort, depth 5 | `world` |
| `/joint_states` | yes | `JointState` | 100 Hz | reliable, transient local | age 2 ms |
| `/morph/vision/telemetry` | **no** | — | — | — | — |
| `/morph/wheel_speed_mismatch` | **no** | — | — | — | — |
| the six new V2 topics (Section 9) | no (to be built) | — | — | — | — |

- `use_sim_time` is set on every launched node except `gazebo_keyboard` and `camera_gate`.
- TF: one tree, 39 frames, root `world`. `world → base_link` comes from ground-truth odometry
  (`rviz_state`), not from an estimator. `camera_link` and `perception_link` are static children of
  `base_link`. Verified by subscribing to `/tf` and `/tf_static` (70 Hz on `/tf`).
- Census caveat: ROS graph discovery arrives in stages, so the first snapshot can miss topics; the
  tool now lists late arrivals separately.

## 6. Tests and coverage

| | Before | After Phase 0 |
|---|---|---|
| pytest items | 134 passed | **157 passed** (7.4 s → 5.9 s) |
| Test files | 7 | 9 |
| Statement coverage of `morph_sim/` (coverage.py) | 38 % (1692 statements, 1050 missed) | 40 % (1742 statements, 1050 missed) |

Coverage by area before: `control.py` 97 %, `perception_core.py` 100 %, `safety.py` 100 %,
`design_physics.py` 94 %; every ROS node (`terrain_manager`, `interaction_runtime`,
`perception_adapter`, UI nodes) 0 %. The C++ plugins have no tests.

## 7. Measured vs target

| Quantity | Target / assumption in the V2 brief | Measured | Status |
|---|---|---|---|
| Headless RTF | ≈ 0.95 | 0.99–1.00 | met |
| GUI RTF | "about 0.34" | 1.00 | brief is out of date |
| Morph duration | about 2 s | 5.0 s (540↔672), 11.9 s (540↔237), 16.7 s (237↔672) | conflict S6 |
| Morph speed gate | never above 0.22 m/s | 0.20–0.25 m/s by ground truth | partly met |
| Burst speed | 2.0 m/s | 0.60 capped, 0.76 motor limit | conflict S5 |
| Ramp | 15–25° | climbs 9.8°, stalls on 14.8° | conflict S3 |
| Slip at mu 0.35 | slip expected | none sustained even at mu 0.22 | conflict S4 |
| CoM height | 340 mm | 291 mm from URDF inertials | conflict S10 |
| Peak jerk | ≤ 8.0 m/s³ × 1.05 | 8.4–10.0 m/s³ (0.1 s window), no limiter yet | baseline for A8 |
| Tests | — | 157 | baseline for J |
