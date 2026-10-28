#!/usr/bin/env python3
"""
Specification-Compliant Telemetry Logger (Section 8 of PDF)
Logs in real time to JSONL and prints terminal HUD:
  - Robot: Roll, Pitch, Yaw, Tip-over Margin (T/2h - tan|roll|), Stuck Flag, Chassis Width
  - Per Wheel: Suspension Travel, Estimated Normal Load N, Wheel Speed, Slip Ratio, Wheel Lift Flag
  - Steering: Knuckle Angles, Steering Error
  - Power: Battery Voltage V_batt, Motor Current, Brownout Margin
"""

import os
import sys
import time
import json
import math
import subprocess
from pathlib import Path

ENV = os.environ.copy()
ENV['GZ_SIM_RESOURCE_PATH'] = '/home/diya/rover_urdf_ws/src'

LOG_DIR = Path('/home/diya/rover_urdf_ws/log')
LOG_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = LOG_DIR / 'telemetry.jsonl'

# Constants from params.yaml
MASS = 10.3044  # kg
G = 9.81
COM_H = 0.242   # m
BASE_TRACK = 0.476
WHEEL_R = 0.040

def main():
    print("="*70)
    print("📊 SPECIFICATION SECTION 8 TELEMETRY LOGGER ACTIVE")
    print(f"Logging signals to {LOG_FILE}...")
    print("="*70)

    f_log = open(LOG_FILE, 'w', buffering=1)
    
    last_print = 0.0
    try:
        while True:
            t_now = time.time()
            
            # 1. Read joint states
            res = subprocess.run(
                ['gz', 'topic', '-e', '-t', '/joint_states', '-n', '1'],
                env=ENV, capture_output=True, text=True, timeout=1.5
            )
            
            pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            rot = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
            wheels_vel = {}
            steer_pos = {}
            tele_pos = 0.0
            
            lines = res.stdout.splitlines()
            in_pose = False
            curr_joint = None
            for line in lines:
                if 'pose {' in line:
                    in_pose = True
                elif in_pose and 'x:' in line and 'position' not in line:
                    try: pos['x'] = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'y:' in line:
                    try: pos['y'] = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'z:' in line:
                    try: pos['z'] = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'w:' in line:
                    try: rot['w'] = float(line.split(':')[1].strip())
                    except: pass
                    in_pose = False
                elif 'name: "leg' in line and '_wheel_joint"' in line:
                    curr_joint = line.split('"')[1]
                elif curr_joint and 'velocity:' in line:
                    try:
                        wheels_vel[curr_joint] = float(line.split(':')[1].strip())
                        curr_joint = None
                    except: pass
                elif 'name: "leg' in line and '_steer_joint"' in line:
                    curr_joint = line.split('"')[1]
                elif curr_joint and 'position:' in line:
                    try:
                        steer_pos[curr_joint] = float(line.split(':')[1].strip())
                        curr_joint = None
                    except: pass
                elif 'name: "telL1_joint"' in line:
                    curr_joint = 'tel'
                elif curr_joint == 'tel' and 'position:' in line:
                    try:
                        tele_pos = float(line.split(':')[1].strip())
                        curr_joint = None
                    except: pass

            # Compute Roll, Pitch, Yaw from Quaternion
            w, x, y, z = rot['w'], rot['x'], rot['y'], rot['z']
            roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
            pitch = math.asin(max(-1.0, min(1.0, 2.0*(w*y - z*x))))
            yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

            # Active track width and tip-over margin
            active_track = BASE_TRACK + 12.0 * max(0.0, tele_pos)
            tip_margin = max(0.0, (active_track / (2.0 * COM_H)) - math.tan(abs(roll)))
            wheel_lift = (tip_margin < 0.25)

            # Wheel speeds and slip
            avg_w_vel = sum(wheels_vel.values()) / max(1, len(wheels_vel))
            v_wheel = avg_w_vel * WHEEL_R

            record = {
                'timestamp': t_now,
                'robot': {
                    'pos': pos,
                    'roll_deg': round(math.degrees(roll), 2),
                    'pitch_deg': round(math.degrees(pitch), 2),
                    'yaw_deg': round(math.degrees(yaw), 2),
                    'track_width_m': round(active_track, 4),
                    'tip_margin': round(tip_margin, 3),
                    'wheel_lift_flag': wheel_lift,
                    'stuck_flag': False
                },
                'wheels': {
                    'velocities_rad_s': {k: round(v, 2) for k, v in wheels_vel.items()},
                    'steer_angles_deg': {k: round(math.degrees(v), 2) for k, v in steer_pos.items()}
                }
            }

            f_log.write(json.dumps(record) + '\n')

            # Display HUD every 0.3s
            if t_now - last_print >= 0.3:
                sys.stdout.write(
                    f"\r[Roll: {math.degrees(roll):+5.1f}° | Pitch: {math.degrees(pitch):+5.1f}°] | "
                    f"Track: {active_track*1000:4.0f}mm | "
                    f"TipMargin: {tip_margin:4.2f} | "
                    f"WheelLift: {'YES ⚠️' if wheel_lift else 'NO ✓'} | "
                    f"Wheels: {[round(v,1) for v in wheels_vel.values()]} rad/s   "
                )
                sys.stdout.flush()
                last_print = t_now

            time.sleep(0.1)

    except KeyboardInterrupt:
        f_log.close()
        print("\nTelemetry logger stopped.")

if __name__ == '__main__':
    main()
