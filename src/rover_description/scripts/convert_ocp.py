#!/usr/bin/env python3
"""
STEP to URDF Converter for Hexagonal Telescopic Chassis Rover
Uses OCP (OpenCascade Python bindings) for proper STEP parsing,
part extraction, STL export, and inertial property computation.
"""

import os
import sys
import re
import math
import struct
import shutil
from pathlib import Path

VENV = "/home/diya/cad_env/lib/python3.14/site-packages"
if VENV not in sys.path:
    sys.path.insert(0, VENV)

import numpy as np

# ─── OCP imports ───
from OCP.STEPControl import STEPControl_Reader
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_Location
from OCP.TDF import TDF_LabelSequence, TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_SOLID, TopAbs_COMPOUND
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS
from OCP.gp import gp_Trsf, gp_XYZ, gp_Mat
from OCP.TopLoc import TopLoc_Location


# ─── Config ───
STEP_FILE   = "/home/diya/Downloads/seee1.step"
OUTPUT_BASE = "/home/diya/rover_urdf_ws/src/rover_description"
MESH_DIR    = os.path.join(OUTPUT_BASE, "meshes")
URDF_PATH   = os.path.join(OUTPUT_BASE, "urdf", "rover.urdf")

# Tessellation quality (mm)
MESH_DEFLECTION  = 1.0   # linear deflection
MESH_ANGLE       = 0.5   # angular deflection (radians)

os.makedirs(MESH_DIR, exist_ok=True)
os.makedirs(os.path.dirname(URDF_PATH), exist_ok=True)


# ═══════════════════════════════════════════════════════════
# Material density assignment (kg/m³)
# ═══════════════════════════════════════════════════════════

def density_for(name: str) -> float:
    n = name.lower()
    if 'wheel' in n or 'tire' in n:   return 1100   # rubber
    if 'shock' in n:                   return 7850   # steel
    if 'drive' in n or 'input' in n:   return 5000   # gearmotor mix
    if 'guide' in n:                   return 7850   # steel rail
    if 'carrier' in n:                 return 7850   # steel knuckle
    if 'steer' in n:                   return 4000   # aluminum+steel
    if 'vision' in n or 'camera' in n: return 1500   # plastic+electronics
    return 2700  # aluminum (structural default)


# ═══════════════════════════════════════════════════════════
# 1. Load STEP with XDE reader – preserves part names & tree
# ═══════════════════════════════════════════════════════════

print("=" * 60)
print("STEP → URDF Converter (OCP)")
print("=" * 60)

print("\n[1/4] Loading STEP with XDE reader…")
doc = TDocStd_Document(TCollection_ExtendedString("step-doc"))
reader = STEPCAFControl_Reader()
reader.SetColorMode(True)
reader.SetNameMode(True)
reader.SetLayerMode(True)

status = reader.ReadFile(STEP_FILE)
assert status == IFSelect_RetDone, f"ReadFile failed: {status}"
reader.Transfer(doc)

shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
color_tool = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())

free_shapes = TDF_LabelSequence()
shape_tool.GetFreeShapes(free_shapes)
print(f"   Top-level shapes: {free_shapes.Length()}")


# ═══════════════════════════════════════════════════════════
# 2. Walk assembly tree, export each part as STL
# ═══════════════════════════════════════════════════════════

def label_name(label) -> str:
    """Get the user-visible name attached to a TDF_Label."""
    n = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), n):
        return n.Get().ToExtString()
    return ""

def label_transform(label) -> gp_Trsf:
    """Get the placement transform on a label (component instance)."""
    loc = shape_tool.GetLocation(label)
    return loc.IsIdentity() and gp_Trsf() or loc.IsIdentity() == False and loc.Transformation() or gp_Trsf()


parts = {}   # name -> info dict
name_counter = {}

def unique_name(raw: str) -> str:
    """Sanitise and deduplicate a part name."""
    # If raw looks like a timestamp, skip it
    if re.match(r"^\d{4}-\d{2}-\d{2}", raw):
        raw = raw.split("_", 1)[-1] if "_" in raw else raw
    clean = re.sub(r"[^A-Za-z0-9_]", "_", raw).strip("_")
    if not clean or clean[0].isdigit():
        clean = "part_" + clean
    base = clean
    name_counter.setdefault(base, 0)
    if name_counter[base] > 0:
        clean = f"{base}_{name_counter[base]}"
    name_counter[base] += 1
    return clean


def export_shape(shape, name: str):
    """Tessellate a TopoDS_Shape and write binary STL."""
    stl_path = os.path.join(MESH_DIR, f"{name}.stl")

    mesh = BRepMesh_IncrementalMesh(shape, MESH_DEFLECTION, False, MESH_ANGLE, True)
    mesh.Perform()

    writer = StlAPI_Writer()
    writer.SetASCIIMode(False)
    ok = writer.Write(shape, stl_path)
    if not ok or not os.path.exists(stl_path) or os.path.getsize(stl_path) < 84:
        return None

    # ── Inertial properties ──
    vprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, vprops)
    vol_mm3 = vprops.Mass()            # mm³
    vol_m3  = abs(vol_mm3) * 1e-9      # m³
    com     = vprops.CentreOfMass()
    com_m   = (com.X()*1e-3, com.Y()*1e-3, com.Z()*1e-3)

    # Bounding box
    bbox = Bnd_Box()
    BRepBndLib.Add_s(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    size_m = ((xmax-xmin)*1e-3, (ymax-ymin)*1e-3, (zmax-zmin)*1e-3)

    # Density & mass
    rho  = density_for(name)
    mass = max(vol_m3 * rho, 0.001)

    # Inertia at CoM (OCP gives matrix in mm^5 worth of volume)
    mat = vprops.MatrixOfInertia()
    # scale = density * mm^5→m^5 = density * 1e-15
    s = rho * 1e-15
    ixx = max(abs(mat.Value(1,1)) * s, 1e-6)
    iyy = max(abs(mat.Value(2,2)) * s, 1e-6)
    izz = max(abs(mat.Value(3,3)) * s, 1e-6)
    ixy = mat.Value(1,2) * s
    ixz = mat.Value(1,3) * s
    iyz = mat.Value(2,3) * s

    parts[name] = dict(
        stl      = stl_path,
        mesh_uri = f"package://rover_description/meshes/{name}.stl",
        mass     = mass,
        com      = com_m,
        inertia  = dict(ixx=ixx, iyy=iyy, izz=izz, ixy=ixy, ixz=ixz, iyz=iyz),
        bbox_min = (xmin*1e-3, ymin*1e-3, zmin*1e-3),
        bbox_max = (xmax*1e-3, ymax*1e-3, zmax*1e-3),
        size     = size_m,
    )
    return name


def walk(label, depth=0):
    """Recursively walk the XDE assembly tree."""
    name_raw = label_name(label)

    if shape_tool.IsSimpleShape(label):
        shape = shape_tool.GetShape(label)
        if shape.IsNull():
            return
        uname = unique_name(name_raw or f"anon_{label.Tag()}")
        result = export_shape(shape, uname)
        if result:
            print(f"{'  '*depth}✓ {uname}  mass={parts[uname]['mass']:.4f} kg  "
                  f"size={parts[uname]['size'][0]*100:.1f}×{parts[uname]['size'][1]*100:.1f}×{parts[uname]['size'][2]*100:.1f} cm")
        else:
            print(f"{'  '*depth}✗ {uname} (empty mesh)")
        return

    if shape_tool.IsAssembly(label):
        print(f"{'  '*depth}▸ Assembly: {name_raw}")
        comps = TDF_LabelSequence()
        shape_tool.GetComponents(label, comps)
        for i in range(comps.Length()):
            comp = comps.Value(i + 1)
            ref = TDF_Label()
            if shape_tool.GetReferredShape(comp, ref):
                walk(ref, depth + 1)
            else:
                walk(comp, depth + 1)


print("\n[2/4] Extracting parts → STL …")
for i in range(free_shapes.Length()):
    walk(free_shapes.Value(i + 1))

print(f"\n   Exported {len(parts)} parts to {MESH_DIR}")


# ═══════════════════════════════════════════════════════════
# 2b. Scale all STL files from mm → m
# ═══════════════════════════════════════════════════════════

print("\n   Scaling STL meshes mm → m …")
for name, info in parts.items():
    path = info['stl']
    try:
        with open(path, 'rb') as f:
            header = f.read(80)
            n_tri  = struct.unpack('<I', f.read(4))[0]
            raw    = f.read()
        with open(path, 'wb') as f:
            f.write(header)
            f.write(struct.pack('<I', n_tri))
            off = 0
            for _ in range(n_tri):
                nx, ny, nz = struct.unpack_from('<3f', raw, off); off += 12
                f.write(struct.pack('<3f', nx, ny, nz))
                for _ in range(3):
                    vx, vy, vz = struct.unpack_from('<3f', raw, off); off += 12
                    f.write(struct.pack('<3f', vx*1e-3, vy*1e-3, vz*1e-3))
                f.write(raw[off:off+2]); off += 2
    except Exception as e:
        print(f"      Warning scaling {name}: {e}")


# ═══════════════════════════════════════════════════════════
# 3. Build URDF
# ═══════════════════════════════════════════════════════════

print("\n[3/4] Building URDF …")

L = []   # output lines
def w(s=""): L.append(s)

w('<?xml version="1.0" encoding="UTF-8"?>')
w('<robot name="hex_tele_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">')
w()

# ── Materials ──
mats = dict(
    aluminum    = "0.7 0.7 0.75 1",
    dark_alu    = "0.3 0.3 0.35 1",
    steel       = "0.5 0.5 0.55 1",
    rubber      = "0.15 0.15 0.15 1",
    red_anod    = "0.8 0.1 0.1 1",
    blue_anod   = "0.1 0.2 0.8 1",
    black       = "0.1 0.1 0.1 1",
    orange      = "1 0.5 0 1",
)
for mn, rgba in mats.items():
    w(f'  <material name="{mn}"><color rgba="{rgba}"/></material>')
w()

def mat_for(n):
    nl = n.lower()
    if 'wheel' in nl:                   return 'rubber'
    if 'shock' in nl:                   return 'steel'
    if 'drive' in nl:                   return 'dark_alu'
    if 'steer' in nl:                   return 'blue_anod'
    if 'hex' in nl or 'follower' in nl: return 'red_anod'
    if 'vision' in nl or 'camera' in nl:return 'black'
    if 'tip' in nl:                     return 'orange'
    return 'aluminum'


def link(name, p=None):
    """Emit a <link>."""
    w(f'  <link name="{name}">')
    if p and name in parts:
        info = parts[name]
        m = info['mass']
        I = info['inertia']
        w(f'    <inertial>')
        w(f'      <mass value="{m:.6f}"/>')
        w(f'      <inertia ixx="{I["ixx"]:.8e}" ixy="{I["ixy"]:.8e}" ixz="{I["ixz"]:.8e}"')
        w(f'               iyy="{I["iyy"]:.8e}" iyz="{I["iyz"]:.8e}" izz="{I["izz"]:.8e}"/>')
        w(f'    </inertial>')
        w(f'    <visual>')
        w(f'      <geometry><mesh filename="{info["mesh_uri"]}"/></geometry>')
        w(f'      <material name="{mat_for(name)}"/>')
        w(f'    </visual>')
        w(f'    <collision>')
        w(f'      <geometry><mesh filename="{info["mesh_uri"]}"/></geometry>')
        w(f'    </collision>')
    else:
        w(f'    <inertial><mass value="0.001"/>')
        w(f'      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>')
        w(f'    </inertial>')
    w(f'  </link>')
    w()


def joint(name, jtype, parent, child,
          xyz="0 0 0", rpy="0 0 0", axis="0 0 1",
          lo=None, hi=None, eff=100, vel=1.0,
          damp=0.1, fric=0.05):
    w(f'  <joint name="{name}" type="{jtype}">')
    w(f'    <parent link="{parent}"/>')
    w(f'    <child link="{child}"/>')
    w(f'    <origin xyz="{xyz}" rpy="{rpy}"/>')
    if jtype != 'fixed':
        w(f'    <axis xyz="{axis}"/>')
        w(f'    <dynamics damping="{damp}" friction="{fric}"/>')
    if jtype in ('revolute', 'prismatic') and lo is not None:
        w(f'    <limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="{vel}"/>')
    w(f'  </joint>')
    w()

def gazebo_link(name, mu1=0.8, mu2=0.8, kp=1e6, kd=100):
    w(f'  <gazebo reference="{name}">')
    w(f'    <mu1>{mu1}</mu1><mu2>{mu2}</mu2>')
    w(f'    <kp>{kp}</kp><kd>{kd}</kd>')
    w(f'  </gazebo>')
    w()

def transmission(jname, hw="hardware_interface/EffortJointInterface"):
    w(f'  <transmission name="trans_{jname}">')
    w(f'    <type>transmission_interface/SimpleTransmission</type>')
    w(f'    <joint name="{jname}"><hardwareInterface>{hw}</hardwareInterface></joint>')
    w(f'    <actuator name="motor_{jname}"><mechanicalReduction>1</mechanicalReduction></actuator>')
    w(f'  </transmission>')
    w()


# ── Gazebo ros_control plugin ──
w('  <gazebo>')
w('    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">')
w('      <robotNamespace>/rover</robotNamespace>')
w('    </plugin>')
w('  </gazebo>')
w()

# ═══════════ BASE ═══════════
w('  <!-- ═══ BASE ═══ -->')
link('base_footprint')
joint('footprint_joint', 'fixed', 'base_footprint', 'base_link', xyz='0 0 0.15')
link('base_link', 'fixed')

used = {'fixed'}

# ═══════════ LEG CHAINS ═══════════
# Approximate leg mount positions (derived from bounding boxes of actual parts)
# We'll compute these from the CAD data if available
def get_part_center(name):
    """Get the center of a part's bounding box in meters."""
    if name in parts:
        bmin = parts[name]['bbox_min']
        bmax = parts[name]['bbox_max']
        return ((bmin[0]+bmax[0])/2, (bmin[1]+bmax[1])/2, (bmin[2]+bmax[2])/2)
    return None

# Try to compute actual positions from part bounding boxes
leg_mounts = {}
for leg in ('FL', 'FR', 'BL', 'BR'):
    input_name = f'leg{leg}_input'
    ctr = get_part_center(input_name)
    if ctr:
        leg_mounts[leg] = ctr
        print(f"   Leg {leg} input center: ({ctr[0]:.3f}, {ctr[1]:.3f}, {ctr[2]:.3f}) m")
    else:
        # Fallback approximate positions
        sx = 1 if 'F' in leg else -1
        sy = 1 if 'L' in leg else -1
        leg_mounts[leg] = (sx * 0.25, sy * 0.20, 0)
        print(f"   Leg {leg} using fallback position")

for leg in ('FL', 'FR', 'BL', 'BR'):
    w(f'  <!-- ═══ LEG {leg} ═══ -->')
    pf = f'leg{leg}'
    sx = 1 if 'F' in leg else -1
    sy = 1 if 'L' in leg else -1
    mnt = leg_mounts[leg]
    mxyz = f'{mnt[0]:.4f} {mnt[1]:.4f} {mnt[2]:.4f}'

    # input (steering actuator mount)
    link(f'{pf}_input', True); used.add(f'{pf}_input')
    joint(f'{pf}_input_jnt', 'revolute', 'base_link', f'{pf}_input',
          xyz=mxyz, axis='0 0 1', lo=-0.5, hi=0.5, eff=50, vel=2, damp=0.5, fric=0.1)
    transmission(f'{pf}_input_jnt')

    # upper suspension arm
    link(f'{pf}_upper', True); used.add(f'{pf}_upper')
    joint(f'{pf}_upper_jnt', 'revolute', f'{pf}_input', f'{pf}_upper',
          axis='0 1 0', lo=-0.3, hi=0.3, eff=200, vel=1, damp=5, fric=0.5)

    # shock body (mounted on upper arm, pivots about y)
    link(f'{pf}_shockbody', True); used.add(f'{pf}_shockbody')
    joint(f'{pf}_shockbody_jnt', 'revolute', f'{pf}_upper', f'{pf}_shockbody',
          xyz='0 0 -0.02', axis='0 1 0', lo=-0.2, hi=0.2, eff=500, vel=0.5, damp=50, fric=1)

    # shock rod (prismatic inside shock body – the actual spring/damper)
    link(f'{pf}_shockrod', True); used.add(f'{pf}_shockrod')
    joint(f'{pf}_shock_jnt', 'prismatic', f'{pf}_shockbody', f'{pf}_shockrod',
          axis='0 0 1', lo=-0.05, hi=0.05, eff=1000, vel=0.3, damp=500, fric=10)
    # Gazebo implicit spring for shock absorber
    w(f'  <gazebo reference="{pf}_shock_jnt">')
    w(f'    <implicitSpringDamper>true</implicitSpringDamper>')
    w(f'    <springStiffness>5000</springStiffness>')
    w(f'    <springReference>0</springReference>')
    w(f'  </gazebo>')
    w()

    # lower suspension arm
    link(f'{pf}_lower', True); used.add(f'{pf}_lower')
    joint(f'{pf}_lower_jnt', 'revolute', f'{pf}_input', f'{pf}_lower',
          xyz='0 0 -0.05', axis='0 1 0', lo=-0.3, hi=0.3, eff=200, vel=1, damp=5, fric=0.5)

    # carrier / knuckle
    link(f'{pf}_carrier', True); used.add(f'{pf}_carrier')
    joint(f'{pf}_carrier_jnt', 'revolute', f'{pf}_lower', f'{pf}_carrier',
          xyz=f'0 {0.05*sy:.3f} -0.08', axis='0 0 1',
          lo=-0.1, hi=0.1, eff=100, vel=1, damp=2, fric=0.3)

    # steering
    link(f'{pf}_steer', True); used.add(f'{pf}_steer')
    joint(f'{pf}_steer_jnt', 'revolute', f'{pf}_carrier', f'{pf}_steer',
          axis='0 0 1', lo=-0.6, hi=0.6, eff=30, vel=3, damp=0.3, fric=0.05)
    transmission(f'{pf}_steer_jnt')

    # drive (motor hub – continuous)
    link(f'{pf}_drive', True); used.add(f'{pf}_drive')
    joint(f'{pf}_drive_jnt', 'continuous', f'{pf}_steer', f'{pf}_drive',
          axis='0 1 0', damp=0.01, fric=0.005)
    transmission(f'{pf}_drive_jnt', 'hardware_interface/VelocityJointInterface')

    # wheel (continuous, high-friction Gazebo contact)
    link(f'{pf}_wheel', True); used.add(f'{pf}_wheel')
    joint(f'{pf}_wheel_jnt', 'continuous', f'{pf}_drive', f'{pf}_wheel',
          axis='0 1 0', damp=0.005, fric=0.001)
    gazebo_link(f'{pf}_wheel', mu1=1.5, mu2=1.2, kp=1e7, kd=100)
    transmission(f'{pf}_wheel_jnt', 'hardware_interface/VelocityJointInterface')


# ═══════════ HEX EXPANSION ═══════════
w('  <!-- ═══ HEX EXPANSION MECHANISM ═══ -->')
for hid in ('FL', 'FR', 'BL', 'BR'):
    ha = f'hexarm{hid}'
    if ha in parts:
        c = get_part_center(ha)
        xyz = f'{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}' if c else '0 0 0'
        link(ha, True); used.add(ha)
        joint(f'{ha}_jnt', 'revolute', 'base_link', ha,
              xyz=xyz, axis='0 0 1', lo=-1.0, hi=1.0, eff=50, vel=1, damp=1, fric=0.2)
        transmission(f'{ha}_jnt')

    arm = f'arm{hid}'
    if arm in parts:
        link(arm, True); used.add(arm)
        parent = ha if ha in parts else 'base_link'
        joint(f'{arm}_jnt', 'revolute', parent, arm,
              xyz='0.05 0 0', axis='0 0 1', lo=-1.5, hi=1.5, eff=50, vel=1, damp=1, fric=0.2)

    hf = f'hex_follower_{hid}'
    if hf in parts:
        link(hf, True); used.add(hf)
        parent = arm if arm in parts else 'base_link'
        joint(f'{hf}_jnt', 'revolute', parent, hf,
              xyz='0.03 0 0', axis='0 0 1', lo=-1.0, hi=1.0, eff=20, vel=1, damp=0.5, fric=0.1)

    tip = f'tip{hid}'
    if tip in parts:
        link(tip, True); used.add(tip)
        parent = arm if arm in parts else 'base_link'
        joint(f'{tip}_jnt', 'fixed', parent, tip, xyz='0.08 0 0')


# Hex rotors
for hr in ('hexrotL240', 'hexrotL350', 'hexrotR240', 'hexrotR350'):
    if hr in parts:
        c = get_part_center(hr)
        xyz = f'{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}' if c else '0 0 0'
        link(hr, True); used.add(hr)
        joint(f'{hr}_jnt', 'revolute', 'base_link', hr,
              xyz=xyz, axis='0 0 1', lo=-3.14, hi=3.14, eff=30, vel=2, damp=0.3, fric=0.05)
        transmission(f'{hr}_jnt')

# ═══════════ TELESCOPIC MECHANISM ═══════════
w('  <!-- ═══ TELESCOPIC SEGMENTS ═══ -->')
for side in ('L', 'R'):
    prev = 'base_link'
    for seg in range(1, 7):
        tn = f'tel{side}{seg}'
        if tn in parts:
            c = get_part_center(tn)
            xyz = f'{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}' if (c and seg == 1) else '0 0 0'
            link(tn, True); used.add(tn)
            joint(f'{tn}_jnt', 'prismatic', prev, tn,
                  xyz=xyz, axis='1 0 0', lo=-0.1, hi=0.1, eff=200, vel=0.5, damp=10, fric=2)
            if seg == 1:
                transmission(f'{tn}_jnt')
            prev = tn

    # Rotary elements
    for ri in range(5):
        for rs in ('145', '15'):
            rn = f'rot{side}{ri}_{rs}'
            if rn in parts:
                c = get_part_center(rn)
                xyz = f'{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}' if c else '0 0 0'
                link(rn, True); used.add(rn)
                joint(f'{rn}_jnt', 'continuous', 'base_link', rn,
                      xyz=xyz, axis='1 0 0', damp=0.1, fric=0.02)

# ═══════════ GUIDE RAILS ═══════════
w('  <!-- ═══ GUIDE RAILS (fixed) ═══ -->')
for side in ('L', 'R'):
    for angle in ('-240', '240'):
        for idx in range(1, 4):
            gn = f'guide{side}_{angle}_{idx}'
            if gn in parts:
                link(gn, True); used.add(gn)
                joint(f'{gn}_jnt', 'fixed', 'base_link', gn)

# ═══════════ VISION SYSTEM ═══════════
w('  <!-- ═══ VISION (pan/tilt camera) ═══ -->')
if 'VISION_camera_yoke' in parts:
    c = get_part_center('VISION_camera_yoke')
    xyz = f'{c[0]:.4f} {c[1]:.4f} {c[2]:.4f}' if c else '0.05 0 0.10'
    link('VISION_camera_yoke', True); used.add('VISION_camera_yoke')
    joint('cam_pan_jnt', 'revolute', 'base_link', 'VISION_camera_yoke',
          xyz=xyz, axis='0 0 1', lo=-3.14, hi=3.14, eff=5, vel=3, damp=0.05, fric=0.01)
    transmission('cam_pan_jnt')

if 'VISION_tilt_cradle' in parts:
    link('VISION_tilt_cradle', True); used.add('VISION_tilt_cradle')
    joint('cam_tilt_jnt', 'revolute', 'VISION_camera_yoke', 'VISION_tilt_cradle',
          xyz='0 0 0.03', axis='0 1 0', lo=-1.57, hi=0.5, eff=5, vel=3, damp=0.05, fric=0.01)
    transmission('cam_tilt_jnt')
    # Camera sensor
    w('  <gazebo reference="VISION_tilt_cradle">')
    w('    <sensor type="camera" name="rover_camera">')
    w('      <update_rate>30</update_rate>')
    w('      <camera><horizontal_fov>1.3963</horizontal_fov>')
    w('        <image><width>1280</width><height>720</height><format>R8G8B8</format></image>')
    w('        <clip><near>0.02</near><far>300</far></clip>')
    w('      </camera>')
    w('      <plugin name="camera_ctrl" filename="libgazebo_ros_camera.so">')
    w('        <cameraName>rover/camera</cameraName>')
    w('        <imageTopicName>image_raw</imageTopicName>')
    w('        <frameName>VISION_tilt_cradle</frameName>')
    w('      </plugin>')
    w('    </sensor>')
    w('  </gazebo>')
    w()


# ═══════════ REMAINING PARTS ═══════════
w('  <!-- ═══ REMAINING PARTS (fixed to base) ═══ -->')
for pn in sorted(parts.keys()):
    if pn not in used:
        link(pn, True)
        joint(f'{pn}_fixed_jnt', 'fixed', 'base_link', pn)
        used.add(pn)
        print(f"   ℹ {pn} → fixed to base_link")


w('</robot>')

with open(URDF_PATH, 'w') as f:
    f.write('\n'.join(L))

print(f"\n   URDF → {URDF_PATH}")


# ═══════════════════════════════════════════════════════════
# 4. Summary
# ═══════════════════════════════════════════════════════════

print("\n[4/4] Summary")
total_mass = sum(p['mass'] for p in parts.values())
n_stl = len([f for f in os.listdir(MESH_DIR) if f.endswith('.stl')])
print(f"   Parts exported : {len(parts)}")
print(f"   STL files      : {n_stl}")
print(f"   Total mass     : {total_mass:.2f} kg")
print(f"   URDF           : {URDF_PATH}")
print(f"   Meshes         : {MESH_DIR}")
print("   Done ✓")
