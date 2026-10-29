# MORPH: RViz, quicker response and sensor-driven chassis adaptation

Start all three windows — Gazebo, RViz and the combined controls — with:

```bash
cd /home/diya/Documents/Codex/2026-10-24/ac/outputs/morph_spec_v4_ws
bash run_smart.sh
```

For less graphics load while keeping RViz, controls and the same Gazebo physics/sensors, use `bash run_smart_fast.sh`. This omits only the Gazebo 3D window. Use the regular command above when presenting both views.

Keep the **MORPH combined controls** window focused for driving. W/S drives, Q/C strafes, A/D turns; these can be held together. V or **SENSOR AUTO** toggles local sensor adaptation. E/X/N and terrain mode keys switch back to manual chassis control. B brakes, Space/K latches a controlled stop, and U resets it. The simulated controlled stop uses motor braking within the current/torque limits; it is not a modeled physical power-isolation circuit.

For manual control with the new RViz view: `bash run_realtime.sh`. To open RViz alongside an already running simulation: `bash run_rviz.sh` from a terminal using the same ROS_DOMAIN_ID. Close the existing simulation before starting a new one; the workspace launch lock rejects duplicates.

## RViz displays

- The robot model follows all measured joint states, including wheels, suspension, ride height and telescope stages.
- The fixed frame is `world`; the orbit view follows `base_link`.
- The raw camera and the measured 3D range cloud are displayed.
- The ground-truth path is explicitly labeled. It uses simulator odometry for visualization, rather than claiming encoder localization accuracy.
- The perception text marker reports the current observed geometry and shape choice. It expires when observations stop.
- Optional displays show encoder odometry and the arena's terrain mesh geometry. Enable these in the Displays panel when needed; the default keeps rendering lighter.

Use `rviz:=false` if you want Gazebo without the extra visualization window. RViz is now enabled by default in the normal simulation launch.

## How the automatic mechanism works

A proposed forward range sensor was added to the robot model: 96 horizontal by 12 vertical rays, 10 Hz, 5 m range, with assumed 3 mm range noise. Its 50 g assumed mass is included in the existing 10 kg total. It is a simulated sensor proposal, not a specification for purchased hardware.

The observer fits local ground geometry from the actual range returns and levels it using the IMU. It does not read arena sector IDs or robot ground-truth position. Suspension and height feedback estimate the sensor's ground clearance.

- Smooth ground: return toward the normal configuration.
- A sustained slope above about 8 degrees, or appreciable body tilt: request expansion for stability and reduce speed.
- Observed uneven ground: request compaction and increased ride height through the rocky profile.
- A substantial obstacle ahead: progressively limit translation and stop before the chassis reaches it.
- Missing, invalid or stale observations: hold automatic driving at zero after the 0.5 s freshness timeout.

Three stable observations are required before changing to a slope/rough-ground mode; ten smooth observations are required to return to normal. This reduces shape oscillation. Manual shape/profile commands disable sensor adaptation, and the legacy map-based automatic mode cannot simultaneously own the shape target.

**The current mechanism changes fore/aft chassis length and wheelbase. Its lateral width stays fixed.** It cannot shrink its width to pass a narrow doorway. The observer is a local geometric heuristic, not RGB object recognition, a material classifier or an autonomous navigation planner. Forward sensing does not establish rear obstacle or drop-off coverage. Use it under operator supervision while validating new terrain.

## Response changes

The console publishes immediately on key-state changes and refreshes its visual labels less often. Drive-target queues use depth one to avoid stale-command buildup. Vehicle acceleration feedforward requests motor current sooner while retaining voltage, current and torque limits. The requested translational acceleration is 1.2 m/s², below the model's ideal drive-force bound; achieved motion still depends on contact and actuator limits.

In the latest alternating 12-trial comparison with both viewers enabled, median command-to-motion response was **88.3 ms with feedforward**, versus **191.4 ms without** (six trials each; threshold 0.03 m/s, fresh 50 Hz body odometry). The longest feedforward-enabled trial was 114.8 ms. This measures command publication to observed motion, excluding human keyboard timing. It is a small local comparison, not a guaranteed latency bound.

Both viewers currently fall back to software graphics in this test session. The combined-motion run measured approximately 0.59–0.67 real-time factor, so full real-time performance with both views is **not established**. The RViz-only run measured approximately 0.65–0.72 real-time factor in this session. These short measurements are not isolated hardware benchmarks. The faster input response should not be confused with a real-time graphics/physics guarantee.

The morphology trajectory is now 4.5 mm/s per telescope stage, within its modeled 6 mm/s limit. From the normal position, full expansion takes roughly 3.7 s of stage movement and full compaction roughly 8.4 s, plus steering alignment and controller settling. Steering retains its 1 rad/s limit; a 90-degree change cannot be instantaneous with that actuator assumption.

The stop controller now holds a captured set of height/shape targets instead of repeatedly following the measured position, which could otherwise creep under load.

## Verification records

- `validation/smart_perception_integration.json`: imported NIST ramp observed at about 11.6 degrees; expansion requested and reached without a manual shape command; manual override verified.
- `validation/rviz_frames.json`: world-to-base, range-sensor and wheel transforms; a live path and perception marker.
- `validation/input_response_comparison.json`: alternating feedforward-on/off trials using fresh 50 Hz ground-truth body odometry, not human key timing.
- `validation/smart_stop.json`: moving observation-loss and controlled-stop checks.
- `validation/smart_response.json`: full two-view motion/performance recording.
- `validation/smart_fast_response.json`: RViz-only rendering motion/performance recording.
- Software regression suite: 61 checks passed, including holding captured stop targets despite measurement drift.
- Duplicate-launch prevention was verified; visualization children no longer inherit the launch lock.

This is a functional prototype of local chassis adaptation. Its shape policy, assumed sensor, actuator model and terrain behavior require broader trials and hardware measurements before real-world transfer claims.

## Remaining validation work

Sustained real-time performance needs a clean-session benchmark with working GPU acceleration. The renderer reported an unavailable iris driver and used software OpenGL. Controller logs still occasionally report target limiting at steering/stage velocity boundaries; the combined-motion checks completed, but these notices require longer trajectory tracking analysis before publication. The range policy needs obstacle, drop-off, reverse-motion and mixed-terrain trials beyond the demonstrated ramp. Chassis length adaptation does not provide lateral width adaptation. Hardware calibration remains pending.
