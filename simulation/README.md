# MORPH simulation workspace

ROS 2 swerve-robot simulation, articulated chassis, native suspension/drive safety plugins, Gazebo environments, RViz configuration, control console and recorded validation evidence.

## Current status

This is an **in-progress simulation research project**, with hardware parameters labeled assumed. The staged upgrade work order is not complete. Stage A is substantially implemented; Stage B has measured timing improvements but remaining stopping/rendering/RViz issues. Stages C and D remain pending. Start with [the current status ledger](reports/STAGED_UPGRADE_STATUS.md) and [manual calibration tasks](MANUAL_TODO.md). Historical reports and failed experiments are retained and must not be presented as current acceptance.

The latest source includes controller-side trajectory interpolation, measured-state activation, a transport-first bridge launcher, force-limited ride-height actuation, feedback guards and per-run parameter manifests. The combined Gazebo/RViz/console test recorded reduced real-time performance and an RViz shutdown crash; that issue is not yet resolved. HQ camera rendering also failed timing acceptance in the software-rendering test environment. Lower-friction stopping has recorded one-second misses.

## Setup and launch

The current implementation was built and tested on **Ubuntu 26.04 / ROS 2 Lyrical / Gazebo Sim 10**. Earlier Humble/Jazzy compatibility paths are retained, but the latest lifecycle and controller changes have not been verified on those versions. Install the supported ROS/Gazebo development dependencies first; the Lyrical setup path expects ROS to be installed.

From this directory:

```bash
bash setup_ubuntu.sh
bash run_gazebo.sh
```

The normal launch includes Gazebo, the combined control console and RViz. To use the separately measured camera-disabled timing configuration without RViz:

```bash
bash run_gazebo.sh rviz:=false
```

Useful options:

```bash
bash run_gazebo.sh headless:=true rviz:=false controls:=none teleop:=false
bash run_gazebo.sh arena:=adaptive
bash run_gazebo.sh arena:=warehouse
bash run_gazebo.sh camera_enabled:=true
```

Camera rendering is explicitly optional; it increases load substantially in software rendering. `position_backend:=auto` selects trajectory interpolation for the electrical-drive profile and forward controllers for the legacy profile. `LP_NUM_THREADS=2` is an optional throughput preset, not a global default: repeated tests showed improved RTF but slower GUI frame cadence.

See [arena controls](ARENA_GUIDE.md), [RViz and sensor adaptation](SMART_RVIZ_GUIDE.md), and [parameter provenance](PARAMETER_GUIDE.md).

## Files and evidence

- `src/morph_sim/`: ROS package, robot meshes/Xacro, native plugins, launch/controllers, worlds, imported models and provenance.
- `reports/`: staged work order, historical review and current implementation/evidence ledger.
- `validation/`: measured results, failures, raw experiment records and manifests. Absolute paths in historical manifests identify the original test machine; runtime generation resolves the new checkout locally.
- `output/`: historical review PDF. The Markdown status ledger is newer than the PDF.
- Root shell scripts: setup, launch and view helpers.

Build output, installed files, runtime caches, local authentication and backup archives are intentionally excluded. Run setup to generate the local build/install folders. Validation evidence is not required to execute the robot, but is included for reproducibility.

## Provenance and limitations

Project source license is recorded in `src/morph_sim/LICENSE`. Imported models retain their own license/provenance files; do not assume every third-party asset has the project source license. CAD-derived meshes and assumed component properties are documented in the package. Native rigid contact does not establish real deformable-soil behavior or a calibrated hardware model.
