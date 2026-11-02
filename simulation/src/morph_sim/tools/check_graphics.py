#!/usr/bin/env python3
"""Read-only Ubuntu graphics diagnostics; never installs or changes drivers."""
import argparse
import datetime
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='log/graphics_check.json')
    args = parser.parse_args()
    commands = {'opengl': ['glxinfo', '-B'], 'pci': ['lspci', '-nnk'],
                'nvidia': ['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader']}
    results = {}
    for key, command in commands.items():
        if not shutil.which(command[0]):
            results[key] = {'available': False}
            continue
        try:
            run = subprocess.run(command, capture_output=True, text=True, timeout=15)
            results[key] = {'available': True, 'exit_code': run.returncode,
                            'stdout': run.stdout, 'stderr': run.stderr}
        except (OSError, subprocess.TimeoutExpired) as error:
            results[key] = {'available': True, 'error': str(error)}
    gl = results['opengl']
    text = gl.get('stdout', '').lower()
    software = any(name in text for name in ('llvmpipe', 'softpipe', 'software rasterizer', 'swrast'))
    renderer_lines = [line.strip() for line in gl.get('stdout', '').splitlines() if 'renderer' in line.lower()]
    state = 'software' if software else 'hardware_candidate' if renderer_lines and gl.get('exit_code') == 0 else 'unknown'
    report = {'utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'platform': platform.platform(), 'opengl_state': state, 'renderer': renderer_lines,
              'environment': {key: os.environ.get(key) for key in
                  ('DISPLAY', 'WAYLAND_DISPLAY', 'LIBGL_ALWAYS_SOFTWARE', 'MESA_LOADER_DRIVER_OVERRIDE', 'DRI_PRIME', 'ROS_DISTRO')},
              'dri_devices': [str(path) for path in Path('/dev/dri').glob('*')],
              'commands': results,
              'scope': 'GLX diagnostic is not proof of the Gazebo sensor renderer or HQ performance; retain live run manifests and measured camera rates'}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    return 0 if state == 'hardware_candidate' else 1


if __name__ == '__main__':
    raise SystemExit(main())
