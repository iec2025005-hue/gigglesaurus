# MORPH: ROS 2 / Gazebo competition demonstrator

Target: Ubuntu 22.04 / ROS 2 Humble or Ubuntu 24.04 / ROS 2 Jazzy, with Gazebo Harmonic. Code and assets are included. The root `ADAPTIVE_GUIDE.md` has the current two-command Ubuntu start, eight-zone automatic morphology, and Gazebo keyboard controls. The earlier `UPGRADE_GUIDE.md` documents the original competition arena. This is a locally checked implementation candidate; the C++ build and Gazebo physics still require Ubuntu runtime validation.

## 1. Robot model first: XACRO structure

`urdf/morph.urdf.xacro` includes `common.xacro` and `leg.xacro`.

```text
base_link
├─ telR1 → telR2 → … → telR6
│  ├─ stage 4: front-left / front-right mounts
│  └─ stage 6: two articulated perimeter visual links
└─ telL1 → telL2 → … → telL6
   ├─ stage 4: rear-left / rear-right mounts
   └─ stage 6: two articulated perimeter visual links

Each wheel module:
mount → ride-height prismatic → passive spring prismatic
      → steering revolute (−90°…+90°) → continuous wheel
```

The 17 metre-scale STL visual meshes were derived from the user's CAD BREP groups. Four additional visual STL proxies depict tread, hub, shock and steering pod. Mesh origins and CAD groups are recorded in `config/cad_mesh_provenance.json`. The front camera is a simulation sensor; its precise physical mount and several passive guide/perimeter details from the full CAD are omitted in this reduced demonstrator.

**Model reduction:** the full CAD has closed kinematic loops. This URDF is an open-chain reduced model. Twelve stage commands share one shape coordinate, and the perimeter angle is calculated to follow the telescopic heads. The original belt force distribution, slot followers and loop compliance are not physically solved. Perimeter links are visual-only collision-wise, allowing the retained slot-length adjustment to be approximated. Therefore the package is suitable for developing a jury demonstration and control experiments; it is not a lossless dynamics conversion of the entire CAD.

CAD axes are mapped to ROS x forward, y left, z up. The CAD R half becomes the front half; this naming difference is intentional. The normal wheel attachments are x/y ±0.27 m; each attachment moves ±4d in x. Every stage has relative travel d ∈ [−0.0379, +0.016511111111] m. Head movement is ±6d, not d. The reduced model's external silhouette need not reproduce the full CAD's previously measured area percentages exactly. Those percentages refer to the full CAD and neutral steering/suspension.

### Physical parameters and limitations

- Nominal total mass: **10 kg**, allocated as estimates. Base 6.4 kg, twelve stages 0.03 kg each, four perimeter links 0.02 kg each, four legs 0.79 kg each. The assumed 2 kg payload is already included in the base allocation; adding another 2 kg would double count it.
- Inertia matrices are positive-definite analytical box/cylinder estimates with stated COM offsets. They are not measured CAD inertias.
- Wheel: 80 mm diameter × 30 mm width. Smooth-cylinder collision proxy, not tread contact geometry.
- Steering: −π/2 to +π/2 = **180° total travel**. Reversing wheel rotation extends the useful velocity-direction range. This does not mean unlimited cable rotation.
- Ride height: ±15 mm position-controlled virtual actuator. Your passive shock alone cannot implement this commanded height change. Treat this feature as a simulation-only actuator until hardware is added.
- Passive suspension: −10 to +30 mm, spring 1800 N/m, damper 65 Ns/m plus native damping 2 Ns/m. Estimated sag under 10 kg shared equally is ~13.6 mm. Actual shock travel and wheel travel depend on wishbone motion ratio and must be measured.
- Spring/damper tuning changes through a Gazebo C++ plugin, with bounded gains, 0.5 s gain smoothing and a 150 N force cap. Position/velocity command interfaces elsewhere are idealized simulator actuators; URDF effort values alone do not prove physically available motor torque.
- Initial nominal speed cap is 0.60 m/s on track and lower off-road. These are demonstration limits, not measured JGB37-520 performance.
- Pinch points remain around telescopes, slots, wishbones and wheel steering. Verify wire service loops and cable-twist stops in hardware.
- For 10 kg, nominal static force per wheel is ~24.5 N. Steering scrub torque is approximately μ·Fz·scrub_radius; changing scrub radius, terrain or wheel loading changes the requirement. The NEMA17 holding rating does not provide its running-torque curve. Do not infer a real torque margin from a successful idealized Gazebo joint.

## 2. World file and visuals

`worlds/competition.sdf` contains an asphalt loop with lane markings, rocky heightmap, sand dunes, grassy knolls, an approximately 8° ramp, concrete steps from 15 to 75 mm, small rocks and arena boundaries. The taller steps are challenge/failure-test obstacles for 80 mm wheels, not promised traversable terrain.

Generated local assets include 257×257 heightmaps and 1024×1024 albedo/normal/roughness textures. Assets are deterministic and bundled; no Fuel downloads or texture-license dependencies are required. Sun shadows, ambient fill and Ogre2 rendering are configured.

At launch `morph_sim.assets` expands XACRO, calls the installed `gz sdf -p`, then adds native SDF PBR materials, friction, wheel/underbody contact sensors, IMU and ground-truth odometry. This avoids relying on Gazebo Classic-only material/plugin tags. Friction values are presets requiring calibration. Sand is a **rigid dune heightmap with low friction**, with backend-dependent contact-compliance hints; it has no grains, sinkage or terramechanics model. DART may ignore ODE-specific compliance hints. Do not report this as a granular-sand experiment.

Arena positions (metres): spawn (0,−9); rocky (−8,0); sand (0,0); grass (8,0); ramp near (−1,5); steps near (7,5). Drive off the asphalt loop toward a test zone. The layout is a manually operated arena, not an autonomous course.

## 3. Controllers and adaptation

`config/controllers.yaml` defines:

| Controller | Type | Ordered joints |
|---|---|---|
| joint_state_broadcaster | JointStateBroadcaster | All registered states |
| drive_controller | JointGroupVelocityController | fl, fr, rl, rr wheels |
| steering_controller | JointGroupPositionController | fl, fr, rl, rr steering |
| height_controller | JointGroupPositionController | fl, fr, rl, rr ride height |
| shape_controller | JointGroupPositionController | R1…R6, L1…L6, FR/BR/FL/BL perimeter |

The controller manager updates at 100 Hz; the world timestep is 1 ms. Passive spring joints have state interfaces only. `hold_joints=false` prevents the unclaimed passive springs being position-held. The spring plugin is their only commanded force source.

`morph_sim/terrain_manager.py` subscribes to IMU, wheel contacts, underbody contacts, joint states and operator requests. Track/rocky/climb/sand profiles set speed, height and spring/damper gains. Tilt assist selects climb settings above a threshold and applies a small, bounded leveling correction; it is a heuristic, not a terrain classifier or proven active-leveling controller. Wheel contacts report support; normal wheel-ground contact is not treated as a collision emergency. Underbody contact latches stop.

Shape change waits for low commanded body motion and steers the wheels to roll along the changing attachment positions. Wheel velocity uses `v_i = v_body + omega × r_i + dr_i/dt`. It is a reduced-model feedforward strategy requiring runtime tuning, not a proven anti-slip controller. Missing/stale joint feedback inhibits wheel commands; external cmd_vel timeout is 0.35 s. Commands, steering and ride height are slew-limited. Reset a stop only after inspecting its cause.

## 4. Teleoperation and HUD

Gazebo Key Publisher is now the default laptop keyboard input: focus the Gazebo scene. Key commands expire after 0.35 seconds. The former Pygame window remains available with `controls:=pygame` for simultaneous key presses; Gazebo input is selected with `controls:=gazebo`.

| Keys | Action |
|---|---|
| W/S or up/down | Forward / reverse |
| A/D or left/right | Turn left / right |
| Q/C | Strafe left / right |
| Shift | Slower drive request |
| B | Brake while held |
| 1 / 2 / 3 / 4 | Track / rocky / climb / sand profile |
| X / N / E | Compact / normal / expanded shape request |
| PageUp / PageDown | Raise/lower ride-height trim |
| Tab | Stopped steering sweep from −90° to +90° |
| T | Toggle tilt-assist heuristic |
| Space | Latch stop |
| R | Reset stop when stopped |
| Esc | Quit teleop and send zero |

HUD displays commanded speed, Gazebo ground-truth speed when available, steering angles, morphology offset, profile, ride-height target, tilt and stop state. Commanded speed is not odometry. Gazebo ground truth is provided only for simulation evaluation and is not available on the physical robot.

## 5. Directory and launch

```text
morph_sim_ws/
└── src/morph_sim/
    ├── CMakeLists.txt, package.xml, README.md
    ├── urdf/       # parameterized model and leg macros
    ├── worlds/     # competition.sdf
    ├── config/     # controllers, bridge, assumptions, CAD provenance
    ├── launch/     # simulation.launch.py
    ├── nodes/      # executable wrappers
    ├── morph_sim/  # control, adaptation, teleop, assets, smoke test
    ├── src/        # bounded spring/damper Gazebo system
    ├── meshes/     # 17 CAD-derived visual meshes, metres
    ├── models/     # bundled PBR textures and heightmaps
    ├── tests/      # offline unit/structure tests
    └── tools/      # preflight, arena generation, Linux validation
```

Use native Ubuntu 22.04 or 24.04. Extract the supplied ZIP as `~/morph_sim_ws` and run:

```bash
cd ~/morph_sim_ws && bash setup_ubuntu.sh
cd ~/morph_sim_ws && bash run_gazebo.sh
```

Launch generates temporary URDF/SDF files, starts Gazebo and robot_state_publisher, bridges sensors and Gazebo keyboard input, spawns the robot, waits for successful controller activation, then starts adaptation and the input node. `GZ_SIM_RESOURCE_PATH` and `GZ_SIM_SYSTEM_PLUGIN_PATH` are set automatically. If spawn/controller activation returns failure, launch stops.

Headless first-run check (Ogre2 sensors may still require graphics support):

```bash
ros2 launch morph_sim simulation.launch.py headless:=true teleop:=false
# In a second sourced terminal:
ros2 run morph_sim smoke_test
ros2 control list_controllers
ros2 topic echo /morph/status
```

The smoke test checks advancing clock, finite joint states, IMU data, wheel contacts and manager readiness. It does not drive the robot or certify terrain behavior. Do not run it while another teleop publisher is trying to drive, because it publishes zero cmd_vel.

To change wheel radius:

```bash
ros2 launch morph_sim simulation.launch.py wheel_radius:=0.05
```

The radius propagates into inertia, wheel geometry, spawn height and kinematics. Other hardware dimensions, mass and COM are edited in the XACRO macros and must be revalidated. Do not assume an arbitrary wheel fits the physical CAD fork.

## 6. Validation and jury demonstration

Local checks on the development host: Python syntax, XACRO expansion, asset paths and the packaged offline tests passed. Gazebo/libsdformat execution and C++ compilation were unavailable and remain unverified. `setup_ubuntu.sh` and `ros2 run morph_sim smoke_test` provide the next checks. Do not label the package production-ready until those pass on the target machine and the demonstrated behaviors are recorded.

Recommended rehearsal: spawn on flat track and settle; verify all four contact indicators; test stop/reset; drive slowly; demonstrate normal/compact/expanded morphology while stopped; show steering sweep; enter the small-rock zone at low speed; compare profiles; show IMU and actual-speed HUD; record limitations and failures. Keep the supplied challenging steps for later tests.

For sim-to-real work, replace allocated masses/inertias with measured values, calibrate motor torque-speed curves and driver voltage/current, identify shock spring/damping and motion ratios, measure tire friction/rolling resistance, model belt compliance and drive load sharing, implement actual sensor feedback and actuator limits, and compare repeatable physical trials. Never use an attractive animation as evidence of these quantities.

## References / compatibility decisions

- ROS 2 Jazzy installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Jazzy/Harmonic pairing: https://github.com/gazebosim/ros_gz
- gz_ros2_control configuration and command-interface behavior: https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html
- Harmonic sensors: https://gazebosim.org/docs/harmonic/sensors/
- Gazebo joint-force component: https://gazebosim.org/api/sim/8/jointforcecmdcomponent.html
- Bridge message mappings: https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md

Code in this package is Apache-2.0; user CAD ownership is unchanged. Textures and heightmaps are generated procedural assets. No third-party texture pack is bundled.
