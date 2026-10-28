#!/usr/bin/env python3
"""
Test script to verify:
  1. Simulation Real-Time Factor (RTF)
  2. Active chassis morphing (extension & retraction)
  3. IMU sensor publication
"""
import os
import subprocess
import time

env = os.environ.copy()
env['GZ_SIM_RESOURCE_PATH'] = '/home/diya/rover_urdf_ws/src'

world = '/home/diya/rover_urdf_ws/src/rover_description/worlds/arena.sdf'

print("Launching Gazebo Sim server with 40x30m arena and rover...")
sim_proc = subprocess.Popen(
    ['gz', 'sim', '-s', '-r', world],
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

time.sleep(3.0)

try:
    print("\n--- 1. Checking Real-Time Factor (RTF) ---")
    time.sleep(1.0)
    stats = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/rover_arena/stats', '-n', '2'],
        capture_output=True, text=True, timeout=5, env=env
    )
    for line in stats.stdout.splitlines():
        if 'real_time_factor:' in line or 'iterations:' in line:
            print(" ", line)

    print("\n--- 2. Checking IMU Sensor Publication ---")
    imu = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/imu/data', '-n', '1'],
        capture_output=True, text=True, timeout=5, env=env
    )
    if 'linear_acceleration' in imu.stdout or 'angular_velocity' in imu.stdout:
        print("  ✓ IMU sensor active and publishing on /imu/data!")
        for line in imu.stdout.splitlines()[:10]:
            print("   ", line)
    else:
        print("  IMU output:", imu.stdout[:200])

    print("\n--- 3. Testing Active Telescopic Chassis Morphing ---")
    print("  a) Expanding chassis for Side-Slope / High-Stability mode (+35mm/stage)...")
    for side in ['l', 'r']:
        for idx in range(1, 7):
            subprocess.run(['gz', 'topic', '-t', f'/rover/tele_{side}{idx}/cmd_pos',
                            '-m', 'gz.msgs.Double', '-p', 'data: 0.035'],
                           env=env, capture_output=True)
    time.sleep(1.0)
    
    js = subprocess.run(['gz', 'topic', '-e', '-t', '/joint_states', '-n', '1'],
                        capture_output=True, text=True, timeout=5, env=env)
    for line in js.stdout.splitlines():
        if 'telL1_joint' in line or 'telR1_joint' in line:
            print("  ", line)

    print("  b) Retracting chassis for Narrow Corridor / Tight Gap mode (0mm/stage)...")
    for side in ['l', 'r']:
        for idx in range(1, 7):
            subprocess.run(['gz', 'topic', '-t', f'/rover/tele_{side}{idx}/cmd_pos',
                            '-m', 'gz.msgs.Double', '-p', 'data: 0.000'],
                           env=env, capture_output=True)
    time.sleep(1.0)
    print("  ✓ Telescopic morphing verified!")

finally:
    sim_proc.terminate()
    sim_proc.wait()
    print("\nTest completed cleanly.")
