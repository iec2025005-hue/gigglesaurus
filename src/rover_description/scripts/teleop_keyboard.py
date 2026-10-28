#!/usr/bin/env python3
"""
Interactive Teleoperation Script for Hexagonal Telescopic Rover
Supports:
  - W / S: Drive Forward / Backward
  - A / D: Steer Left / Right
  - T / G: Extend / Retract Telescopic chassis
  - I / K / J / L: Pan and Tilt Camera
  - Space: Stop / Brake
"""
import sys
import tty
import termios
import select
import subprocess
import time

def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def publish_twist(speed, steer):
    cmd = f'linear: {{x: {speed:.2f}}}, angular: {{z: {steer:.2f}}}'
    subprocess.run(['gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist', '-p', cmd],
                   capture_output=True)

def publish_steer_angle(angle):
    for leg in ['fl', 'fr', 'bl', 'br']:
        topic = f'/rover/steer_{leg}/cmd_pos'
        subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {angle:.2f}'],
                       capture_output=True)

def publish_tele_stage(pos):
    for side in ['l', 'r']:
        for idx in range(1, 7):
            topic = f'/rover/tele_{side}{idx}/cmd_pos'
            subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {pos:.3f}'],
                           capture_output=True)

def publish_camera(pan, tilt):
    subprocess.run(['gz', 'topic', '-t', '/rover/camera/pan/cmd_pos', '-m', 'gz.msgs.Double', '-p', f'data: {pan:.2f}'],
                   capture_output=True)
    subprocess.run(['gz', 'topic', '-t', '/rover/camera/tilt/cmd_pos', '-m', 'gz.msgs.Double', '-p', f'data: {tilt:.2f}'],
                   capture_output=True)

def main():
    settings = termios.tcgetattr(sys.stdin)
    print("""
=============================================================
  Hexagonal Telescopic Chassis Rover Teleop Controller
=============================================================
Driving:
    W: Forward           S: Backward
    A: Turn Left         D: Turn Right
    SPACE: Stop

Telescopic Expansion:
    T: Extend Chassis    G: Retract Chassis

Camera Pan/Tilt:
    I: Tilt Up           K: Tilt Down
    J: Pan Left          L: Pan Right

Quit:
    Q / Ctrl-C
=============================================================
""")

    speed = 0.0
    turn = 0.0
    tele_pos = 0.0
    cam_pan = 0.0
    cam_tilt = 0.0

    try:
        while True:
            key = get_key(settings, 0.05)
            if key == 'q' or key == '\x03':
                break
            elif key == 'w':
                speed = min(1.5, speed + 0.2)
            elif key == 's':
                speed = max(-1.5, speed - 0.2)
            elif key == 'a':
                turn = min(1.0, turn + 0.2)
            elif key == 'd':
                turn = max(-1.0, turn - 0.2)
            elif key == ' ':
                speed = 0.0
                turn = 0.0
            elif key == 't':
                tele_pos = min(0.05, tele_pos + 0.01)
                publish_tele_stage(tele_pos)
                print(f"Chassis expansion: {tele_pos*1000:.1f} mm/stage")
            elif key == 'g':
                tele_pos = max(0.0, tele_pos - 0.01)
                publish_tele_stage(tele_pos)
                print(f"Chassis expansion: {tele_pos*1000:.1f} mm/stage")
            elif key == 'i':
                cam_tilt = min(0.7, cam_tilt + 0.1)
                publish_camera(cam_pan, cam_tilt)
            elif key == 'k':
                cam_tilt = max(-1.4, cam_tilt - 0.1)
                publish_camera(cam_pan, cam_tilt)
            elif key == 'j':
                cam_pan = min(3.14, cam_pan + 0.15)
                publish_camera(cam_pan, cam_tilt)
            elif key == 'l':
                cam_pan = max(-3.14, cam_pan - 0.15)
                publish_camera(cam_pan, cam_tilt)

            publish_twist(speed, turn)
            publish_steer_angle(turn * 0.45)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        publish_twist(0.0, 0.0)
        print("\nTeleop stopped.")

if __name__ == '__main__':
    main()
