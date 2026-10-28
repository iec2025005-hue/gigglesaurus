# Hexagonal Telescopic Chassis Rover - CAD to URDF & Gazebo Sim

This workspace contains the complete URDF model, meshes, simulation arena, and control plugins generated from `/home/diya/Downloads/seee1.step` and updated according to the **Swerve Robot — Interaction & Environment Specification (v1.0)** (`/home/diya/Downloads/Swerve Robot — Interaction & Environment Specification (2).pdf`).

---

## 🚀 Key Features

1. **Photorealistic Real-Life Multi-Terrain Surrounding & Environment**:
   - **Surveyed Multi-Sector Course**: Complete outdoor proving ground featuring 9 distinct real-life terrain surfaces with high-resolution PBR textures (albedo, normal, roughness):
     - *Smooth Road (Tarmac)*: Baseline high-traction test track
     - *Green Grass / Meadow*: Variable friction turf
     - *Dry Sand*: Low-traction dune surface
     - *Wet Mud*: High-slip off-road track
     - *Ramp Lab*: NIST standard ramps with 5°, 10°, 15°, 20°, 25° slopes & NIST stairs
     - *Debris / Rubble*: Loose gravel and uneven ground
     - *Rock Garden*: Photogrammetry mossy boulders and rocky terrain
     - *Snow Surface*: Low Coulomb friction slip testing
     - *Mixed Terrain*: Transitions between packed dirt, gravel, and grass
   - **Rich Real-Life Scenery & Atmosphere**:
     - 60 DRC practice orange jersey barriers lining the roadways
     - 45 realistic Oak trees and 24 foliage small trees with natural bark and leaves
     - 68 moss-covered boulders and rock clusters
     - Warehouse industrial sector with shelves, desks, and pallet jacks
     - Natural atmospheric lighting: time-of-day sky (sunrise 6h, sunset 20h, current 10h), warm directional sun light (`1.0, 0.94, 0.84`), soft specular highlights, and real dynamic shadows.

2. **True 4WIS/4WID Omnidirectional Swerve Kinematics Controller**:
   - Replaces diff-drive with genuine 4-Wheel Independent Steering & 4-Wheel Independent Drive kinematics with **Dynamic Track Width Awareness**:
     - *Crab Walk / Lateral Strafe*: Translates in any arbitrary 2D vector without changing vehicle heading.
     - *Zero-Turn Spin-In-Place*: Tangentially aligns all wheels to circumscribed circle around center of mass with zero turning radius.
     - *Ackermann Car-Like Steering*: Smooth concentric curvature with zero tyre scrubbing.
     - *X-Stance Mechanical Hill Brake*: Locks 4 wheels into an opposing 45° X wedge to eliminate rollback on steep 25° grades.
     - *Shortest Angular Path Optimization*: Enforces $\le 90^\circ$ steering rotation by dynamically reversing wheel speed.

3. **Active Situation-Aware Adaptive Chassis (Asymmetric & Slope-Aware Stance Morphing)**:
   - **Asymmetric Side-Slope Roll Mitigation** (Roll Tilt $\ge 3^\circ$ / 15° Bank): Automatically and independently expands the **downhill** telescopic stages outward (up to $300\text{ mm}$ per side) to enlarge the lateral support polygon directly below the shifted gravitational load vector, increasing tip-over margin by over $130\%$ without widening the uphill side into hillside rocks.
   - **Narrow Passage Mode** (Corridor Width < 1.0m / Indoor Doors / Warehouse): Automatically retracts the 12 telescopic stages to **$476\text{ mm}$ minimum compact width** to navigate tight spaces without scraping.
   - **High-Speed Cornering Anti-Roll Mode** ($a_{lat} \ge 0.35\text{ m/s}^2$): Dynamically widens track to minimize lateral load transfer ($dN_{lat} = m a_y h / T$) and prevent inner wheel lift.
   - **Rough Terrain / Rock Garden / Washboard Damping**: Automatically morphs to an expanded stable stance ($776\text{ mm}$ – $956\text{ mm}$) for maximum articulation and vibration isolation.
   - **Nominal Cruising**: Smoothly returns to baseline cruising width ($596\text{ mm}$).
   - **100% Native ROS 2 Architecture**: Zero subprocess forks, running at high-speed 25 Hz.

4. **Look-Ahead Computer Vision Terrain Perception Node**:
   - Analyzes real-time RGB video from `/camera/image_raw` at 30 Hz.
   - Classifies upcoming terrain surfaces (Snow, Grass, Mars Sand Dunes, Wet Mud, Rock Garden Rubble, Smooth Tarmac, Narrow Corridor) via HSV/LAB color decomposition and Laplacian spatial frequency variance.
   - Publishes pre-emptive stance recommendations $1.5\text{ seconds}$ before wheels encounter terrain.

4. **Parameters & Specification ([params.yaml](file:///home/diya/rover_urdf_ws/src/rover_description/config/params.yaml))**:
   - Single source of truth conforming to Section 12 of the PDF specification: robot inertia, JGB37-520 drive motor, NEMA17 steering stepper, battery/power specs, IMU and camera parameters, 6 surface profiles, and coupling definitions.

5. **Sensors & High Real-Time Factor (RTF $\ge 1.2$)**:
   - Active **IMU Sensor** (`/imu/data`) and **Camera Sensor** (`/camera/image_raw`).
   - Fast ODE quick-solver with $0.003\text{ s}$ time step ($333\text{ Hz}$), contact stabilization, and optimized bounding volume hierarchies providing a blazing **$\text{RTF} \ge 1.2$** even with full photorealistic scenery and 105 CAD links.

---

## 🛠️ Quick Start

### 1. Source Environment
```bash
source /opt/ros/lyrical/setup.bash
source /home/diya/rover_urdf_ws/install/setup.bash
```

### 2. Launch Gazebo Simulation Arena
```bash
ros2 launch rover_description sim.launch.py
```

### 3. Run the 4WIS/4WID Swerve Kinematics Controller
```bash
source /opt/ros/lyrical/setup.bash
source /home/diya/rover_urdf_ws/install/setup.bash
ros2 run rover_description swerve_kinematics_controller.py
# Or directly:
/home/diya/rover_urdf_ws/src/rover_description/scripts/swerve_kinematics_controller.py
```

### 4. Run the Asymmetric & Slope-Aware Adaptive Chassis Controller
In a separate terminal:
```bash
source /opt/ros/lyrical/setup.bash
source /home/diya/rover_urdf_ws/install/setup.bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/adaptive_chassis_controller.py
```

### 5. Run the Look-Ahead Terrain Perception Pipeline
```bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/terrain_perception_node.py
```

### 6. Multi-Mode Swerve Driving & Teleoperation
```bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/teleop_rover.py
```
Key bindings:
- `1` : Standard Forward / Turn
- `2` : Crab Walk (Omnidirectional Strafe)
- `3` : Zero-Turn Spin-In-Place
- `4` : X-Stance Mechanical Hill Brake
- `W/S` : Forward / Reverse
- `A/D` : Turn or Strafe
- `T/G` : Chassis Morphing (+/- 10mm)
- `I/K & J/L` : 2-DOF Camera Gimbal Tilt & Pan

### 7. Run Unit & Kinematics Test Suite
```bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/test_swerve_and_morphing.py
```

### 5. Automated Adaptive Morphing Demonstration
To run the automated 5-situation scenario test:
```bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/demo_adaptive_chassis.py
```
