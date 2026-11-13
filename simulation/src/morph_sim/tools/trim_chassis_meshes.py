#!/usr/bin/env python3
"""Generate the trimmed-overhang chassis meshes used by the V2 `chassis:=trimmed` variant.

    python3 src/morph_sim/tools/trim_chassis_meshes.py [--half-length 0.165]

Design decision (V2 brief, Part 1.1): in Compact (237 mm wheelbase) the chassis must be 330 mm
long front to back so the rover can crab through a 380 mm pinch. The CAD meshes are 469 mm long
in Compact, so the longitudinal overhang of the fixed frame and of telescope stages 2-5 is
clipped by a plane. The CAD originals are kept untouched (`chassis:=cad`).

This is a SIMULATION MODEL CHANGE, not a CAD change: the printed parts are not modified by it.
Cuts are left open (no cap faces).
"""
import argparse, json, struct
from pathlib import Path
import numpy as np

PKG = Path(__file__).resolve().parents[1]
D_MIN = -0.0379   # Compact stage offset [m]; stage k sits at k*D_MIN along its side's axis


def read_stl(path):
    with open(path, 'rb') as f:
        header = f.read(80); n = struct.unpack('<I', f.read(4))[0]
        data = np.frombuffer(f.read(n * 50), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return header, data['v'].reshape(-1, 3, 3).astype(np.float64)


def write_stl(path, triangles, note):
    tri = np.asarray(triangles, dtype=np.float32)
    normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); length = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, length, out=np.zeros_like(normals), where=length > 0)
    out = np.zeros(len(tri), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    out['n'] = normals; out['v'] = tri.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(note.encode()[:80].ljust(80, b' ')); f.write(struct.pack('<I', len(tri))); f.write(out.tobytes())


def clip(triangles, limit, keep_below=True):
    """Keep the part of each triangle with x <= limit (keep_below) or x >= limit."""
    sign = 1. if keep_below else -1.
    dist = sign * (limit - triangles[:, :, 0])            # >= 0 means keep
    inside = dist >= 0; count = inside.sum(axis=1)
    kept = [triangles[count == 3]]
    for tri, d, keep in zip(triangles[(count > 0) & (count < 3)], dist[(count > 0) & (count < 3)], inside[(count > 0) & (count < 3)]):
        polygon = []
        for i in range(3):                                  # Sutherland-Hodgman against one plane
            a, b = tri[i], tri[(i + 1) % 3]; da, db = d[i], d[(i + 1) % 3]
            if keep[i]: polygon.append(a)
            if keep[i] != keep[(i + 1) % 3]: polygon.append(a + (b - a) * (da / (da - db)))
        for i in range(1, len(polygon) - 1): kept.append(np.array([[polygon[0], polygon[i], polygon[i + 1]]]))
    return np.vstack(kept)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--half-length', type=float, default=.165)
    half = parser.parse_args().half_length; meshes = PKG / 'meshes'; report = {'half_length_m': half, 'compact_stage_offset_m': D_MIN, 'meshes': {}}
    jobs = [('fixed', None)] + [(f'tel{side}{k}', k) for side in 'RL' for k in (2, 3, 4, 5)]
    for name, stage in jobs:
        _, tri = read_stl(meshes / f'{name}.stl'); before = [float(tri[:, :, 0].min()), float(tri[:, :, 0].max())]
        if stage is None:
            out = clip(clip(tri, half, True), -half, False); limits = [-half, half]
        else:
            # A stage link frame is displaced k*d along +x (R) or -x (L); keep |x_body| <= half in Compact.
            limit = half - stage * D_MIN
            out = clip(tri, limit, True) if name[3] == 'R' else clip(tri, -limit, False); limits = [limit] if name[3] == 'R' else [-limit]
        write_stl(meshes / f'{name}_trim.stl', out, f'{name} trimmed overhang, V2 sim model, half length {half} m')
        report['meshes'][name] = {'x_before_m': before, 'x_after_m': [float(out[:, :, 0].min()), float(out[:, :, 0].max())], 'clip_planes_link_x_m': limits,
                                  'triangles_before': int(len(tri)), 'triangles_after': int(len(out))}
        print(name, report['meshes'][name])
    (PKG / 'config/chassis_trim.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
