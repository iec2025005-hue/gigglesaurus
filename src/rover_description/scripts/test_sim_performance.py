#!/usr/bin/env python3
"""
Test simulation performance and Real-Time Factor (RTF) in Gazebo Sim.
"""
import os
import subprocess
import time

env = os.environ.copy()
env['GZ_SIM_RESOURCE_PATH'] = '/home/diya/rover_urdf_ws/src'
env['GZ_IP'] = '127.0.0.1'

world = '/home/diya/rover_urdf_ws/src/rover_description/worlds/arena.sdf'
urdf = '/home/diya/rover_urdf_ws/src/rover_description/urdf/rover.urdf'

print("Launching Gazebo Sim server...")
sim_proc = subprocess.Popen(
    ['gz', 'sim', '-s', '-r', world],
    env=env,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE
)

time.sleep(3.0)

try:
    print("Spawning rover model into arena...")
    spawn = subprocess.run(
        ['gz', 'service', '-s', '/world/rover_arena/create',
         '--reqtype', 'gz.msgs.EntityFactory',
         '--reptype', 'gz.msgs.Boolean',
         '--timeout', '5000',
         '--req', f'sdf_filename: "{urdf}", name: "hex_tele_rover", pose: {{position: {{x: 0, y: 0, z: 0.05}}}}'],
        capture_output=True,
        text=True,
        env=env
    )
    print("Spawn result:", spawn.stdout.strip())

    print("Checking simulation stats...")
    time.sleep(2.0)
    stats = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/world/rover_arena/stats', '-n', '3'],
        capture_output=True,
        text=True,
        timeout=10,
        env=env
    )
    print("Simulation stats output:")
    print(stats.stdout)

finally:
    sim_proc.terminate()
    sim_proc.wait()
    print("Test finished.")
