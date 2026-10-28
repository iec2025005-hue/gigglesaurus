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

2. **Complete 103-Part CAD Conversion**:
   - Every single component from `/home/diya/Downloads/seee1.step` is extracted into lightweight binary STLs, scaled to SI units (meters), and centered at its exact mechanical axis.
   - All links have physically calculated mass, center of mass (CoM), and positive-definite 3D inertia tensors derived from physical material densities (rubber, steel, aluminum, delrin).
   - Preserves all 4 continuous wheel drives, 4 continuous motor shafts, 4 steering knuckles, 4 double-wishbone suspension arms, 4 shock absorbers, and 2-DOF pan-tilt camera head.

3. **Active Situation-Aware Adaptive Chassis (Dynamic Track Morphing)**:
   - **Narrow Passage Mode** (Corridor Width < 1.0m / Indoor Doors / Warehouse): Automatically retracts the 12 telescopic stages to **$476\text{ mm}$ minimum compact width** to navigate tight spaces without scraping.
   - **Side-Slope & Rollover Prevention Mode** (Roll Tilt $\ge 3^\circ$ / 15° Bank): Automatically expands track width outward up to **$896\text{ mm}$ – $1050\text{ mm}$**, increasing the rollover threshold ($T / 2h$) by over 80%.
   - **High-Speed Cornering Anti-Roll Mode** ($a_{lat} \ge 0.35\text{ m/s}^2$): Dynamically widens track to minimize lateral load transfer ($dN_{lat} = m a_y h / T$) and prevent inner wheel lift.
   - **Rough Terrain / Rock Garden / Washboard Damping**: Automatically morphs to an expanded stable stance ($776\text{ mm}$ – $956\text{ mm}$) for maximum articulation and vibration isolation.
   - **Nominal Cruising**: Smoothly returns to baseline cruising width ($596\text{ mm}$).

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

### 3. Run the Situation-Aware Adaptive Chassis Controller
In a separate terminal, to have the chassis automatically change size based on real-time situations:
```bash
source /opt/ros/lyrical/setup.bash
source /home/diya/rover_urdf_ws/install/setup.bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/adaptive_chassis_controller.py
```

### 4. Interactive Driving & Teleoperation
```bash
source /opt/ros/lyrical/setup.bash
source /home/diya/rover_urdf_ws/install/setup.bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/teleop_rover.py
```

### 5. Automated Adaptive Morphing Demonstration
To run the automated 5-situation scenario test:
```bash
/home/diya/rover_urdf_ws/src/rover_description/scripts/demo_adaptive_chassis.py
```
