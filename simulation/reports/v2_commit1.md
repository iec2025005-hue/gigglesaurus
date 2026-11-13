# V2 complete research suite — Commit 1: chassis, arena, terramechanics

> **Superseded in part (2026-11-13, later the same day).** This report was written for the previous brief. The code on
> this branch has since changed: the arena is `arena:=tetra` (`tools/build_tetra_arena.py`, `worlds/tetra_arena.sdf`,
> `config/tetra_arena.json`), the tour ramp is 16° with 15/18/20/22° test ramps beside it, sand and mud use μ 0.35 / 0.25,
> and the default rail holding force is 25 N per stage (`stage_hold:=reinforced`; 8 N and 60 N remain as options).
> Current numbers are in `reports/paper_ready_summary.md` (generated) and `reports/v2_status.md`.

Date: 2026-11-13. Branch `v2/complete-research-suite`. Simulation of an assumed-parameter model.

This commit is infrastructure only. Fixture measurements are reported with Commit 2, because the
first runs showed that the existing controller lets the telescope rails go limp (see
`reports/v2_commit2.md`), which would make any "Extended under load" number here misleading.

## What was built

| Item | Where | Notes |
|---|---|---|
| Trimmed chassis variant | `urdf/*.xacro`, `meshes/*_trim.stl`, `tools/trim_chassis_meshes.py` | `chassis:=trimmed` (default) or `chassis:=cad`. Fixed frame and telescope stages 2–5 clipped. Simulation model only; CAD and printed parts are untouched. |
| Envelope collision shapes | `urdf/*.xacro` | Deck, belly, stage-4 beams, drive units, so the physics model matches the visible robot. |
| Rail holding option | `stage_hold:=backdrivable` (default, 8 N per stage) or `self_locking` (60 N, assumed) | Speed limit 6 mm/s in both, so morph time is unchanged. |
| V2 arena | `tools/build_v2_arena.py` → `worlds/v2_arena.sdf`, `config/v2_arena.json` | `arena:=v2`. Zones in order: start, trenches 100/130/155 mm, loose sand, mud, ramps (15° on the tour line; 12/18/20/22° beside it), 620 mm corridor, 380 mm pinch, rocks, rubble, finish. Obstacle placement uses seed 42. |
| Terramechanics plugin | `src/terramechanics_system.cpp`, `src/terramechanics_model.hpp`, `morph_sim/terramechanics.py` | Bekker rigid-wheel sinkage, compaction resistance with rut memory, Janosi–Hanamoto slip estimate. Applied as a resisting torque per wheel. Telemetry on `/morph/terramechanics/status`. |
| Launch options | `launch/simulation.launch.py` | `arena:=v2`, `chassis:=`, `stage_hold:=`, `spawn_x:=`, `spawn_y:=`, `spawn_yaw:=`. |
| Network isolation | `transport_env.sh`, `tools/run_benchmark_case.py` | ROS discovery is localhost-only by default. |

## Chassis length in Compact (237 mm wheelbase)

| Configuration | CAD model | Trimmed model |
|---|---|---|
| Wheels straight | 469 mm | 387 mm (wheelbase + wheel diameter; cannot be less) |
| Crab, drive units turned inward (front −90°, rear +90°) | 469 mm | **330 mm** |
| Crab, all four modules at +90° | 469 mm | 426 mm |

The 330 mm target holds for the crab pose the pinch needs, with the drive units inward. "All four
modules at 90°" puts two drive units outward (426 mm) and does not fit a 380 mm gap; the wheel
axes are the same in both, only the sign of two steering angles differs.

## Fixture facts that follow from the geometry

- **620 mm corridor:** built with 40 mm kerbs. The drive units and frame make the visible robot
  701 mm wide from 42 mm above ground, so only the tyres (586 mm) fit; 17 mm clearance per side.
  Full-height walls would need at least 760 mm.
- **380 mm pinch:** full-height walls; 25 mm clearance per side for the 330 mm envelope. Normal
  (690 mm) and Extended (822 mm) cannot enter.
- **Rocks and rubble:** 20 mm and 12 mm high. With 0.5 Nm per wheel the single-wheel step limit is
  about 25 mm; taller obstacles would be barriers.
- **Trenches:** a 150 mm rigid wheel sits 19 / 38 mm deep in the 100 / 130 mm gaps and cannot touch
  both edges of the 155 mm gap.

## Soil model: what it is and is not

- The ground does not deform and tyres do not physically slip. Sinkage and soil slip are computed
  states; the only thing applied to the simulation is a resisting torque on each wheel.
- The project's own soil constants are placeholders (identical for sand and mud). Three literature
  sets were added under `terramechanics.soils` and the placeholders left alone. The new values
  (k_c, k_phi, n, c, phi) were later checked against Table 1 of NASA/TM-20250006958, which cites Wong (2001), and
  match; the shear deformation moduli are assumed.
- Slip sinkage and slip power loss are implemented but off by default: with slip sinkage on, the
  model drives the leading wheels to the slip limit on every soil in the table.

## Tests

`tests/test_v2_model_and_arena.py` (20 tests): chassis envelope per variant, mass unchanged, fixture
dimensions read back from the SDF, reproducible obstacle placement, friction from the parameter
table, plugin injection, rail option, soil model properties, C++ vs Python cross-check to 1e-9.
Total 177 passing; `colcon build` clean.

## Network incident

One early run was corrupted by another ROS system on the same Wi-Fi network (nodes
`sparkx_team_bridge`, a Nav2 stack) sharing the default ROS domain: its `/clock` reached this
simulation. In the other direction, that run published `/cmd_vel` at 0.3 m/s for about 6 s on the
shared domain, and the Phase 0 runs published there too. Whether that system consumed them is
unknown. All launches from this workspace are now localhost-only; set
`ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` to undo that.
