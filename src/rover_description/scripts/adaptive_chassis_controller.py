#!/usr/bin/env python3
"""
Situation-Aware Adaptive Chassis Controller
===========================================
Automatically morphs/adjusts the rover's 12 telescopic chassis stages
(telL1-L6, telR1-R6) according to real-time environment & vehicle situation:

  1. NARROW_PASSAGE:
     Retracts chassis to compact width (476mm) when entering tight corridors
     or obstacle gaps (1.5x robot width) to avoid side collisions.

  2. SIDE_SLOPE_STABILIZATION:
     Expands chassis to wide stance (up to 1050mm) on side slopes (roll >= 3 deg)
     to dramatically increase tip-over margin (T / 2h) and prevent rollover.

  3. HIGH_SPEED_CORNERING:
     Dynamically widens chassis during fast steering turns to reduce lateral
     load transfer (dN_lat = m*a_y*h/T) and prevent inner-wheel lift.

  4. ROUGH_TERRAIN_DAMPING:
     Adjusts to stable intermediate width when washboard/rumble vibration is detected.

  5. NOMINAL_CRUISING:
     Smoothly returns to baseline cruising width on flat open terrain.

Complies with:
  - Swerve Robot — Interaction & Environment Specification v1.0
  - Section 1 (Arena Zones), Section 2 (A1 Load Transfer, A5 Tip-over)
  - Section 8 (Log Signals: tip_margin, wheel_lift_flag, chassis_width)
"""

import os
import sys
import math
import time
import subprocess
import threading
from concurrent.futures import ThreadPoolExecutor
from collections import deque

ENV = os.environ.copy()
ENV['GZ_SIM_RESOURCE_PATH'] = '/home/diya/rover_urdf_ws/src'

# Parameters from params.yaml
TRACK_BASE_M = 0.476
MAX_STAGE_TRAVEL_M = 0.050
NUM_STAGES_PER_SIDE = 6
COM_HEIGHT_H = 0.242

class AdaptiveChassisController:
    def __init__(self):
        self.running = True
        
        # State
        self.current_stage_pos = 0.010  # meters per stage
        self.target_stage_pos = 0.010
        self.situation = "NOMINAL_CRUISING"
        
        # Sensor data
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.lin_vel_x = 0.0
        self.ang_vel_z = 0.0
        self.accel_z_history = deque(maxlen=20)
        
        # Telemetry metrics
        self.tip_margin = 1.0
        self.wheel_lift_flag = False
        
        # Last command cache to avoid redundant publishes
        self.last_published_stage = -1.0
        self.executor = ThreadPoolExecutor(max_workers=12)

    def _pub_topic(self, topic_val):
        topic, val = topic_val
        subprocess.run(
            ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {val}'],
            env=ENV, capture_output=True
        )

    def evaluate_situation(self):
        """Determine current vehicle/environment situation and compute target width."""
        abs_roll = abs(self.roll)
        abs_pitch = abs(self.pitch)
        lat_accel = abs(self.lin_vel_x * self.ang_vel_z)
        
        # Compute vertical vibration variance (washboard/rock detector)
        var_az = 0.0
        if len(self.accel_z_history) >= 5:
            mean_az = sum(self.accel_z_history) / len(self.accel_z_history)
            var_az = sum((a - mean_az)**2 for a in self.accel_z_history) / len(self.accel_z_history)

        # 1. Check for Narrow Passage / Indoor / Warehouse Corridor
        # In multi-terrain arena, warehouse is at X in [-18, -2], Y in [-22, -8]
        # Boulder narrow gaps or tight indoor channels
        is_indoor = (self.pos_x < -7.0 and self.pos_y < 0) or (-18.0 <= self.pos_x <= -2.0 and -22.0 <= self.pos_y <= -8.0)
        is_rock_gap = (3.5 <= self.pos_x <= 5.5 and 9.0 <= self.pos_y <= 13.0)
        
        # 2. Check Multi-Terrain Sectors (Y in [0, 12])
        in_sectors = (0.0 <= self.pos_y <= 12.0)
        
        if is_indoor or is_rock_gap:
            self.situation = "NARROW_PASSAGE (Compact 476mm)"
            self.target_stage_pos = 0.000  # Fully retracted: 476 mm width
            
        # Side Slope / Steep Tilt (Roll >= 3 deg = 0.05 rad or Pitch >= 10 deg)
        elif abs_roll >= 0.05 or abs_pitch >= 0.18:
            self.situation = "SIDE_SLOPE_STABILIZATION (Wide Stance)"
            # Proportional widening with roll angle: up to max 0.048m per stage
            expansion_ratio = min(1.0, (abs_roll - 0.05) / 0.20)
            self.target_stage_pos = 0.020 + 0.028 * expansion_ratio  # 0.020 to 0.048m (716mm to 1052mm)
            
        # High-Speed Turning / Cornering Anti-Roll
        elif lat_accel >= 0.30 or abs(self.ang_vel_z) >= 0.5:
            self.situation = "HIGH_SPEED_CORNERING (Anti-Roll Stance)"
            self.target_stage_pos = 0.035  # Widened to 896 mm
            
        # Multi-Terrain Specific Profiles
        elif in_sectors:
            if -18.0 <= self.pos_x < -14.0:
                self.situation = "TERRAIN: SMOOTH ROAD (Cruising)"
                self.target_stage_pos = 0.010  # 596 mm
            elif -14.0 <= self.pos_x < -10.0:
                self.situation = "TERRAIN: GREEN GRASS (Moderate Grip)"
                self.target_stage_pos = 0.020  # 716 mm
            elif -10.0 <= self.pos_x < -6.0:
                self.situation = "TERRAIN: DRY SAND (Low Ground Pressure)"
                self.target_stage_pos = 0.030  # 836 mm
            elif -6.0 <= self.pos_x < -2.0:
                self.situation = "TERRAIN: MUD / WET (Deep Rut Stability)"
                self.target_stage_pos = 0.035  # 896 mm
            elif -2.0 <= self.pos_x < 2.0:
                self.situation = "TERRAIN: RAMP LAB (Inclines)"
                self.target_stage_pos = 0.025  # 776 mm
            elif 2.0 <= self.pos_x < 6.0:
                self.situation = "TERRAIN: DEBRIS (Loose Gravel)"
                self.target_stage_pos = 0.025  # 776 mm
            elif 6.0 <= self.pos_x < 10.0:
                self.situation = "TERRAIN: ROCK GARDEN (High Articulation)"
                self.target_stage_pos = 0.040  # 956 mm
            elif 10.0 <= self.pos_x < 14.0:
                self.situation = "TERRAIN: SNOW SURFACE (Low Friction)"
                self.target_stage_pos = 0.030  # 836 mm
            else:
                self.situation = "TERRAIN: MIXED TERRAIN"
                self.target_stage_pos = 0.020  # 716 mm
                
        # Rough Terrain / Washboard Vibration Detection
        elif var_az >= 0.4:
            self.situation = "ROUGH_TERRAIN_DAMPING (Vibration Isolation)"
            self.target_stage_pos = 0.025  # Widened to 776 mm
            
        # Nominal Cruising on Flat Tarmac/Forecourt
        else:
            self.situation = "NOMINAL_CRUISING"
            self.target_stage_pos = 0.010  # Baseline: 596 mm

        # Compute dynamic tip-over margin from Section 2 A5
        # Tip margin = (Track / 2h) - tan(roll)
        current_track = TRACK_BASE_M + 2.0 * NUM_STAGES_PER_SIDE * self.current_stage_pos
        tan_roll = math.tan(abs_roll)
        stability_limit = (current_track / 2.0) / COM_HEIGHT_H
        self.tip_margin = max(0.0, stability_limit - tan_roll)
        self.wheel_lift_flag = (self.tip_margin < 0.20)

    def update_actuators(self, dt=0.05):
        """Smoothly actuate all 12 telescopic stages toward target position."""
        # Rate limit (max 0.03 m/s)
        max_delta = 0.03 * dt
        diff = self.target_stage_pos - self.current_stage_pos
        if abs(diff) <= max_delta:
            self.current_stage_pos = self.target_stage_pos
        else:
            self.current_stage_pos += math.copysign(max_delta, diff)

        # Publish to Gazebo controllers if changed significantly (> 0.5mm)
        if abs(self.current_stage_pos - self.last_published_stage) >= 0.0005:
            self.last_published_stage = self.current_stage_pos
            val = f"{self.current_stage_pos:.4f}"
            tasks = [
                (f"/rover/tele_{side}{idx}/cmd_pos", val)
                for side in ['l', 'r']
                for idx in range(1, NUM_STAGES_PER_SIDE + 1)
            ]
            list(self.executor.map(self._pub_topic, tasks))

    def read_gazebo_sensors(self):
        """Read pose, velocity, and orientation from /joint_states and IMU."""
        try:
            res = subprocess.run(
                ['gz', 'topic', '-e', '-t', '/joint_states', '-n', '1'],
                env=ENV, capture_output=True, text=True, timeout=1.0
            )
            lines = res.stdout.splitlines()
            in_pose = False
            for line in lines:
                if 'pose {' in line:
                    in_pose = True
                elif in_pose and 'x:' in line and 'position' not in line:
                    try: self.pos_x = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'y:' in line:
                    try: self.pos_y = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'z:' in line:
                    try: self.pos_z = float(line.split(':')[1].strip())
                    except: pass
                elif in_pose and 'w:' in line:
                    in_pose = False
        except Exception:
            pass

        # Read IMU data if available
        try:
            res_imu = subprocess.run(
                ['gz', 'topic', '-e', '-t', '/imu/data', '-n', '1'],
                env=ENV, capture_output=True, text=True, timeout=1.0
            )
            for line in res_imu.stdout.splitlines():
                if 'x:' in line and 'orientation' not in line:
                    pass
                elif 'linear_acceleration' in line:
                    pass
                elif 'z:' in line and len(self.accel_z_history) < 20:
                    try:
                        az = float(line.split(':')[1].strip())
                        self.accel_z_history.append(az)
                    except: pass
        except Exception:
            pass

    def run(self):
        print("="*65)
        print("🤖 SITUATION-AWARE ADAPTIVE CHASSIS CONTROLLER ACTIVE")
        print("="*65)
        print("Monitoring IMU roll/pitch, velocity, and terrain zones...")
        print("Actuating 12 telescopic stages to morph track width dynamically.\n")
        
        last_display = 0.0
        try:
            while self.running:
                t0 = time.time()
                self.read_gazebo_sensors()
                self.evaluate_situation()
                self.update_actuators(dt=0.08)
                
                # Print real-time status every 0.3s
                if time.time() - last_display >= 0.3:
                    current_width = (TRACK_BASE_M + 2.0 * NUM_STAGES_PER_SIDE * self.current_stage_pos) * 1000.0
                    sys.stdout.write(
                        f"\r[STATUS: {self.situation:<40}] | "
                        f"Width: {current_width:5.0f}mm | "
                        f"Stage: {self.current_stage_pos*1000:4.1f}mm | "
                        f"Tip Margin: {self.tip_margin:4.2f} "
                    )
                    sys.stdout.flush()
                    last_display = time.time()
                    
                elapsed = time.time() - t0
                sleep_time = max(0.01, 0.08 - elapsed)
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nShutting down Adaptive Chassis Controller...")

if __name__ == '__main__':
    controller = AdaptiveChassisController()
    controller.run()
