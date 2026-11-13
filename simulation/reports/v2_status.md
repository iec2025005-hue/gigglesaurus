# V2 research suite — status against the current brief

Date: 2026-11-13. Branch `v2/complete-research-suite` (from `v2/phase0-audit`). Nothing is pushed.
Simulation of an assumed-parameter model; the robot is not built. Numbers: `reports/paper_ready_summary.md`
(generated — regenerate with `benchmarks/generate_paper_summary.py`).

## 1. What each commit contains

| Commit | Contents |
|---|---|
| 1 | Trimmed chassis (Compact 330 mm long in the crab pose), envelope collision shapes, rail holding 25 N (`stage_hold:=reinforced`, default), `tools/build_tetra_arena.py` → `worlds/tetra_arena.sdf`, soil-resistance plugin (`MorphTerramechanics`) with a Python reference |
| 2 | Rate-coupled swerve kinematics, `continuous` morph policy, rails hold their commanded length, corner-height presets (±15 mm pitch, single-wheel lift), +20 % thermally governed torque boost, trench stepping planner, S-curve jerk limit, friction-linked acceleration, cos² wheel scaling, 3.2 rad/s steering, holonomic key maps |
| 3 | `decision_core` (stance cost and lead-time trigger), `decision_synthesizer` node (`/morph/decision_explanation`, `/morph/cost_breakdown`), camera colour classifier, autonomous tour with four policies |
| 4 | `dashboard/` (live web page), `benchmarks/run_ablation_suite.py`, `summarise_fixtures.py`, `generate_paper_figures.py`, `generate_paper_summary.py`, results, figures, `reports/table1_ablation.tex` |
| 5 | `video_pipeline/record_conference_video.py`, `reports/paper_ready_summary.md`, `reports/research_paper_proposals.md`, this file |

## 2. How to run

All from `simulation/`, after `source /opt/ros/lyrical/setup.bash && source install/setup.bash`.

| Task | Command |
|---|---|
| Drive by hand in the arena | `bash run_gazebo.sh arena:=tetra` — W/S forward/back, A/D strafe, Q/E or ←/→ rotate, Z/X/N expand/compact/normal |
| Autonomous tour with windows | `bash run_gazebo.sh arena:=tetra tour:=proposed camera_enabled:=true` |
| Dashboard (while a simulation runs) | `python3 dashboard/server.py` then open `http://127.0.0.1:8080` |
| One measured tour | `MORPH_PROTOCOL_SCRIPT=src/morph_sim/tools/run_tour.py MORPH_TOUR_POLICY=proposed python3 src/morph_sim/tools/run_benchmark_case.py NAME headless:=true rviz:=false arena:=tetra camera_enabled:=true camera_width:=320` |
| Ablation, 4 policies × 5 | `python3 benchmarks/run_ablation_suite.py --repeats 5` (about 2.5 h) |
| Figures and LaTeX table | `python3 benchmarks/generate_paper_figures.py` |
| Fixture table | `python3 benchmarks/summarise_fixtures.py` |
| Paper summary | `python3 benchmarks/generate_paper_summary.py` |
| Conference video | `python3 video_pipeline/record_conference_video.py` (about 15 min) |
| Unit tests | `cd src/morph_sim && python3 -m pytest tests -q -p no:cacheprovider` |

Only one simulation can run at a time. Launches are localhost-only (`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`)
because another ROS system shares this network.

## 3. Where the build differs from the brief, and why

| Brief | Built | Reason |
|---|---|---|
| "Adopt only favourable data" | Favourable, justified *design values* are adopted; every measurement is kept | A result that is dropped cannot be defended in review. Section 7 of the paper summary lists what the data does not support |
| Pinch: "all four modules to 90°" | Front −90°, rear +90° (drive units turned inward) | All four at +90° makes the body 426 mm long; inward it is 330 mm and fits 380 mm with 25 mm per side |
| 620 mm corridor | Built with 40 mm kerbs | The visible robot is 701 mm wide above 42 mm; only the tyres (586 mm) fit between 620 mm |
| "Longitudinal" stepped trench 100/130/155 mm | Not built; the three *transverse* trenches are kept | A trench running along the travel direction is straddled by every stance (track 540 mm), so it measures nothing about the wheelbase |
| Trench crossed head-on by ±15 mm pitching and rear torque bias | Implemented and measured: crosses 100 mm only. The tour crosses obliquely, one wheel at a time | ±15 mm moves about 3 % of an axle load. With the body yawed the wheels meet the gap one at a time |
| Ramp: IMU pitch > 8° → Extended + front −15 / rear +15 | Implemented. Extended is selected *before* the ramp by the lead-time trigger; the pitch preset engages above 8° | A shape change takes up to 17 s and cannot start on the slope |
| Ramps up to 22° | Tour ramp is 16°. 18° stalls; 20° and 22° need more force than the drive has with the boost | Drive force 26.7 N, 32.0 N with boost; 18° needs 32.5 N including rolling resistance |
| Do not raise continuous torque | Not raised: 0.5 Nm continuous, +20 % only while requested and below 60 °C | As specified |
| Stage holding 8 N → 25 N in `morph.urdf.xacro` | Done (`stage_effort` default 25) | As specified. Under the 16° climb the wheelbase still gives way from 672 to 527 mm; a self-locking screw (60 N option) is the physical fix |
| Steering 3.2 rad/s "so 90° takes < 0.49 s" | Setpoint slews at 3.2 rad/s (0.49 s); the simulated joint is within 10° after 0.95 s | 0.49 s is the slew time with infinite acceleration; the servo adds lag |
| `height_servo.cpp` levelling | No change needed there; presets and levelling are in `control.py` and executed by the existing servo | The servo already tracks the four corner targets |
| FastAPI / WebSocket dashboard | Flask with server-sent events | FastAPI, uvicorn and websockets are not installed on this machine; nothing was installed system-wide |
| ffmpeg | GStreamer x264 (H.264 in MP4); ffmpeg is used automatically if present | No `ffmpeg` binary on this machine |
| Video 60 fps, 1920×1080, < 45 MB | Yes, at 2× playback: each output frame is one camera frame taken every 1/30 s of simulation time | A six-minute tour at 1× would not fit a three-minute conference limit; the speed is printed on every frame. `--speed 1` records real time |
| Policy names `reactive_stop_morph`, `proposed_predictive_swerve_morph` | Code names `reactive`, `proposed`; the brief's names are used in the tables | Shorter launch arguments |

## 4. Known gaps

- Zone identity and distance in the tour come from ground truth. The camera classifier and LiDAR slope are
  logged alongside but do not drive decisions.
- The camera classifier is a colour classifier for this arena, calibrated on 52 rendered frames of one run and
  not checked on a held-out run. Tarmac and concrete are separated by brightness only (about ±15 % tolerance).
- Yaw jerk in a spin measured 21.5 rad/s³ against the 6 rad/s³ limit: the limit is applied to the command,
  and steering re-alignment adds body yaw acceleration that the filter does not see.
- A straight drive with a shape change ends 70–102 mm further along than the same drive without one.
- A tucked crab drive is clamped to 0.15 m/s by the gated morph policy.
- Rails at 25 N give way under peak load (see the ramp table).
- Reactive and proposed policies share every actuator and the stance choice; the fixed policies use neither
  the corner-height actuators nor the boost. An ablation that isolates the boost or the suspension is not run.
- Soil constants k_c, k_φ, n, c, φ match NASA/TM-20250006958 Table 1 (citing Wong 2001); the shear moduli are
  assumed. Slip-sinkage is implemented but off.
- The recorded video has one cosmetic flaw: in the wheelbase chart the "672" reference label sits under the chart
  title. The recorder is fixed; the committed file was not re-recorded.

## 5. Evidence index

| What | Where |
|---|---|
| Ablation runs | `validation/performance/v2abl_*`, `benchmarks/results/ablation_*.{csv,json}` |
| Fixture runs | `validation/performance/v2s3_*`, `benchmarks/results/fixture_results.json` |
| Video run | `validation/performance/v2video_proposed_1`, `videos/morph_conference_video_60fps.json` |
| Camera calibration frames | run `v2s4_tour_fixed_compact_try1` (frames not kept; thresholds and measured colours are in `vision_core.py`) |
| Phase 0 audit and baseline | `reports/audit_phase0.md`, `reports/baseline.md` |
