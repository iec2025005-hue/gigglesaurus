#!/usr/bin/env python3
"""
Test wheel rotation, steering, and movement under /cmd_vel in Gazebo Sim.
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

time.sleep(2.5)

try:
    print("Spawning rover...")
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
    print("Spawn status:", spawn.stdout.strip())
    time.sleep(1.0)

    # Publish forward velocity command
    print("Publishing /cmd_vel forward velocity (linear.x = 0.5 m/s)...")
    cmd_pub = subprocess.Popen(
        ['gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist',
         '-p', 'linear: {x: 0.5}, angular: {z: 0.0}'],
        env=env
    )
    time.sleep(2.0)
    cmd_pub.terminate()

    # Read joint states to verify wheel and drive shaft rotation
    print("Reading /joint_states to verify wheels and motor shafts are rotating...")
    js = subprocess.run(
        ['gz', 'topic', '-e', '-t', '/joint_states', '-n', '1'],
        capture_output=True,
        text=True,
        timeout=10,
        env=env
    )
    print("Joint state sample:")
    for line in js.stdout.splitlines()[:30]:
        print(" ", line)

finally:
    sim_proc.terminate()
    sim_proc.wait()
    print("Drive test finished.")
