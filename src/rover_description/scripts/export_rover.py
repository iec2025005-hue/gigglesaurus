#!/usr/bin/env python3
"""
Full CAD-to-URDF Exporter for Hexagonal Telescopic Rover
Exports all 103 components to STL, computes exact inertial tensors,
and builds a physics-ready, high-RTF URDF for Gazebo Sim (ROS 2).
"""

import os
import sys
import math
import struct
import json
import trimesh
from pathlib import Path

# OpenCascade Python bindings
sys.path.insert(0, '/home/diya/cad_env/lib/python3.14/site-packages')
from OCP.XCAFDoc import XCAFDoc_ShapeTool, XCAFDoc_DocumentTool
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.TopoDS import TopoDS

STEP_FILE = "/home/diya/Downloads/seee1.step"
OUTPUT_DIR = "/home/diya/rover_urdf_ws/src/rover_description"
MESH_DIR = os.path.join(OUTPUT_DIR, "meshes")
URDF_PATH = os.path.join(OUTPUT_DIR, "urdf", "rover.urdf")

os.makedirs(MESH_DIR, exist_ok=True)
os.makedirs(os.path.dirname(URDF_PATH), exist_ok=True)

print("="*60)
print("1. Loading STEP document...")
print("="*60)

doc = TDocStd_Document(TCollection_ExtendedString('step-doc'))
reader = STEPCAFControl_Reader()
reader.ReadFile(STEP_FILE)
reader.Transfer(doc)

shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
free_shapes = TDF_LabelSequence()
shape_tool.GetFreeShapes(free_shapes)
root = free_shapes.Value(1)
comps = TDF_LabelSequence()
XCAFDoc_ShapeTool.GetComponents_s(root, comps)

print(f"Total components found: {comps.Length()}")

# Densities in kg/m^3
DENSITIES = {
    'wheel': 1100.0,       # Rubber tire
    'shockbody': 7850.0,   # Steel damper cylinder
    'shockrod': 7850.0,    # Steel piston rod
    'drive': 7850.0,       # Steel / brass motor shaft
    'steer': 2700.0,       # Aluminum steering knuckle
    'carrier': 2700.0,     # Aluminum upright
    'upper': 2700.0,       # Aluminum wishbone arm
    'lower': 2700.0,       # Aluminum wishbone arm
    'input': 2700.0,       # Aluminum chassis bracket
    'camera': 1500.0,      # Sensor / plastic / composite
    'cradle': 1500.0,      # Sensor bracket
    'tel': 2700.0,         # Aluminum extrusion
    'guide': 7850.0,       # Steel guide rails
    'rot': 1400.0,         # Delrin / POM rollers
    'hex': 2700.0,         # Aluminum mechanism
    'tip': 2700.0,         # Aluminum
    'arm': 2700.0,         # Aluminum
    'default': 2700.0      # Aluminum default
}

def get_density(name):
    nl = name.lower()
    for k, rho in DENSITIES.items():
        if k in nl:
            return rho
    return DENSITIES['default']

# Materials / colors for visual rendering in Gazebo and RViz
COLORS = {
    'wheel': ('rubber_black', '0.15 0.15 0.15 1.0'),
    'shockbody': ('steel_silver', '0.65 0.65 0.70 1.0'),
    'shockrod': ('chrome_gold', '0.85 0.75 0.30 1.0'),
    'drive': ('dark_steel', '0.25 0.25 0.30 1.0'),
    'steer': ('anodized_blue', '0.10 0.35 0.85 1.0'),
    'carrier': ('anodized_blue', '0.15 0.40 0.90 1.0'),
    'upper': ('bright_alu', '0.75 0.75 0.80 1.0'),
    'lower': ('bright_alu', '0.75 0.75 0.80 1.0'),
    'input': ('anodized_black', '0.20 0.20 0.22 1.0'),
    'camera': ('matte_black', '0.10 0.10 0.12 1.0'),
    'cradle': ('matte_black', '0.10 0.10 0.12 1.0'),
    'tel': ('anodized_orange', '0.90 0.45 0.10 1.0'),
    'guide': ('polished_steel', '0.70 0.70 0.75 1.0'),
    'rot': ('nylon_white', '0.90 0.90 0.92 1.0'),
    'hex': ('anodized_red', '0.85 0.15 0.15 1.0'),
    'tip': ('anodized_orange', '0.90 0.50 0.10 1.0'),
    'arm': ('anodized_silver', '0.75 0.75 0.78 1.0'),
    'fixed': ('dark_chassis', '0.30 0.32 0.35 1.0'),
}

def get_material(name):
    nl = name.lower()
    for k, mat in COLORS.items():
        if k in nl:
            return mat
    return ('default_metal', '0.65 0.65 0.70 1.0')

# Extract all component metadata first
components = {}
print("="*60)
print("2. Extracting component geometries and physical properties...")
print("="*60)

for i in range(comps.Length()):
    comp = comps.Value(i+1)
    n = TDataStd_Name()
    raw_name = n.Get().ToExtString() if comp.FindAttribute(TDataStd_Name.GetID_s(), n) else f'comp_{i}'
    base_name = raw_name.split(':')[0]
    
    shape = XCAFDoc_ShapeTool.GetShape_s(comp)
    
    # Inertial properties
    gprops = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, gprops)
    
    vol_mm3 = abs(gprops.Mass())
    vol_m3 = vol_mm3 * 1e-9
    rho = get_density(base_name)
    mass_kg = max(0.005, vol_m3 * rho)
    
    com_ocp = gprops.CentreOfMass()
    com_assembly_m = (com_ocp.X() * 1e-3, com_ocp.Y() * 1e-3, com_ocp.Z() * 1e-3)
    
    # Bounding box
    bbox = Bnd_Box()
    BRepBndLib.Add_s(shape, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    
    # Inertia tensor about CoM
    ocp_mat = gprops.MatrixOfInertia()
    # Scale inertia from mm^5 * density to kg * m^2
    # In OCP with density=1, matrix is mm^5. Multiply by density (kg/m^3) * 1e-15 -> kg*m^2
    scale_I = rho * 1e-15
    ixx = max(1e-6, ocp_mat.Value(1, 1) * scale_I)
    iyy = max(1e-6, ocp_mat.Value(2, 2) * scale_I)
    izz = max(1e-6, ocp_mat.Value(3, 3) * scale_I)
    ixy = ocp_mat.Value(1, 2) * scale_I
    ixz = ocp_mat.Value(1, 3) * scale_I
    iyz = ocp_mat.Value(2, 3) * scale_I
    
    components[base_name] = {
        'comp_label': comp,
        'shape': shape,
        'mass': mass_kg,
        'com_assembly': com_assembly_m,
        'inertia': (ixx, iyy, izz, ixy, ixz, iyz),
        'bbox_m': (xmin*1e-3, ymin*1e-3, zmin*1e-3, xmax*1e-3, ymax*1e-3, zmax*1e-3),
        'size_m': ((xmax-xmin)*1e-3, (ymax-ymin)*1e-3, (zmax-zmin)*1e-3)
    }

print(f"Extracted metadata for {len(components)} components.")

# Define Link Origins (P_joint_assembly in meters)
# For parts that rotate or slide, link origin is at the mechanical joint axis center!
# For parts fixed to base, origin is (0, 0, 0).

link_origins = {}
for name, cinfo in components.items():
    com = cinfo['com_assembly']
    bbox = cinfo['bbox_m']
    
    if name == 'fixed':
        link_origins[name] = (0.0, 0.0, 0.0)
    elif 'wheel' in name:
        # Wheel axle center is at X=com[0], Y=com[1], Z=0.040
        link_origins[name] = (com[0], com[1], 0.040)
    elif 'steer' in name:
        # Steering kingpin center at X=sgn(X)*0.238, Y=sgn(Y)*0.270, Z=0.123
        sx = 1.0 if com[0] > 0 else -1.0
        sy = 1.0 if com[1] > 0 else -1.0
        link_origins[name] = (sx * 0.238, sy * 0.270, 0.123)
    elif 'drive' in name:
        # Motor drive shaft at com
        link_origins[name] = (com[0], com[1], com[2])
    elif 'carrier' in name:
        # Carrier hub attaches to steer knuckle
        sx = 1.0 if com[0] > 0 else -1.0
        sy = 1.0 if com[1] > 0 else -1.0
        link_origins[name] = (sx * 0.238, sy * 0.270, 0.123)
    elif 'upper' in name:
        # Upper wishbone pivot at Y=sgn(Y)*0.325, Z=0.085
        sx = 1.0 if com[0] > 0 else -1.0
        sy = 1.0 if com[1] > 0 else -1.0
        link_origins[name] = (sx * 0.238, sy * 0.325, 0.085)
    elif 'lower' in name:
        # Lower wishbone pivot at Y=sgn(Y)*0.325, Z=0.060
        sx = 1.0 if com[0] > 0 else -1.0
        sy = 1.0 if com[1] > 0 else -1.0
        link_origins[name] = (sx * 0.238, sy * 0.325, 0.060)
    elif 'shockbody' in name:
        # Shock body upper mount at Y=sgn(Y)*0.335, Z=0.091
        link_origins[name] = (com[0], (0.335 if com[1]>0 else -0.335), 0.091)
    elif 'shockrod' in name:
        # Shock rod lower mount at Y=sgn(Y)*0.270, Z=0.060
        link_origins[name] = (com[0], (0.270 if com[1]>0 else -0.270), 0.060)
    elif 'input' in name:
        # Leg input bracket
        sx = 1.0 if com[0] > 0 else -1.0
        sy = 1.0 if com[1] > 0 else -1.0
        link_origins[name] = (sx * 0.238, sy * 0.310, 0.129)
    elif name == 'VISION_camera_yoke':
        link_origins[name] = (0.0, 0.160, 0.454)
    elif name == 'VISION_tilt_cradle':
        link_origins[name] = (0.0, 0.160, 0.482)
    elif name.startswith('telL') or name.startswith('telR'):
        # Telescopic stage origin
        link_origins[name] = (com[0], 0.0, 0.230)
    else:
        # Default fixed components attached to base_link
        link_origins[name] = (0.0, 0.0, 0.0)

print("="*60)
print("3. Exporting STL meshes relative to link origins (in meters)...")
print("="*60)

for name, cinfo in components.items():
    shape = cinfo['shape']
    stl_temp = os.path.join(MESH_DIR, f"{name}_temp.stl")
    stl_final = os.path.join(MESH_DIR, f"{name}.stl")
    
    # Tessellate shape
    BRepMesh_IncrementalMesh(shape, 0.6, False, 0.4, True)
    writer = StlAPI_Writer()
    writer.Write(shape, stl_temp)
    
    P_orig = link_origins[name]  # (X, Y, Z) in meters
    ox_mm = P_orig[0] * 1000.0
    oy_mm = P_orig[1] * 1000.0
    oz_mm = P_orig[2] * 1000.0
    
    try:
        tmesh = trimesh.load(stl_temp)
        if isinstance(tmesh, trimesh.Scene):
            tmesh = tmesh.dump(concatenate=True)
        if hasattr(tmesh, 'faces') and len(tmesh.faces) > 0:
            tmesh.apply_translation([-ox_mm, -oy_mm, -oz_mm])
            tmesh.apply_scale(1e-3)
            tmesh.export(stl_final)
        else:
            s = cinfo['size_m']
            tmesh = trimesh.creation.box(extents=[max(0.002, s[0]), max(0.002, s[1]), max(0.002, s[2])])
            tmesh.export(stl_final)
    except Exception as e:
        print(f"Fallback box for {name}: {e}")
        s = cinfo['size_m']
        tmesh = trimesh.creation.box(extents=[max(0.002, s[0]), max(0.002, s[1]), max(0.002, s[2])])
        tmesh.export(stl_final)
        
    if os.path.exists(stl_temp):
        os.remove(stl_temp)
        
    # Update local CoM relative to link origin
    cinfo['com_local'] = (
        cinfo['com_assembly'][0] - P_orig[0],
        cinfo['com_assembly'][1] - P_orig[1],
        cinfo['com_assembly'][2] - P_orig[2]
    )

print(f"Exported all {len(components)} STL meshes to {MESH_DIR}")

# ============================================================
# 4. Generate URDF
# ============================================================
print("="*60)
print("4. Generating complete URDF with kinematic tree and Gazebo Sim plugins...")
print("="*60)

lines = []
def w(s=""): lines.append(s)

w('<?xml version="1.0" encoding="UTF-8"?>')
w('<robot name="hex_tele_rover">')
w()

# Materials
unique_mats = {}
for name in components.keys():
    mname, rgba = get_material(name)
    unique_mats[mname] = rgba

for mname, rgba in sorted(unique_mats.items()):
    w(f'  <material name="{mname}">')
    w(f'    <color rgba="{rgba}"/>')
    w(f'  </material>')
w()

# base_footprint (ground projection at Z=0)
w('  <!-- ═════════ BASE FOOTPRINT & BASE LINK ═════════ -->')
w('  <link name="base_footprint"/>')
w()
w('  <joint name="base_footprint_joint" type="fixed">')
w('    <parent link="base_footprint"/>')
w('    <child link="base_link"/>')
w('    <origin xyz="0 0 0" rpy="0 0 0"/>')
w('  </joint>')
w()

# base_link includes fixed chassis body
c_fixed = components['fixed']
com_f = c_fixed['com_local']
I_f = c_fixed['inertia']
m_f = c_fixed['mass']
bbox_f = c_fixed['bbox_m']
size_f = c_fixed['size_m']

w('  <link name="base_link">')
w('    <inertial>')
w(f'      <origin xyz="{com_f[0]:.4f} {com_f[1]:.4f} {com_f[2]:.4f}" rpy="0 0 0"/>')
w(f'      <mass value="{m_f:.4f}"/>')
w(f'      <inertia ixx="{I_f[0]:.6e}" ixy="{I_f[3]:.6e}" ixz="{I_f[4]:.6e}"')
w(f'               iyy="{I_f[1]:.6e}" iyz="{I_f[5]:.6e}" izz="{I_f[2]:.6e}"/>')
w('    </inertial>')
w('    <visual>')
w('      <origin xyz="0 0 0" rpy="0 0 0"/>')
w('      <geometry>')
w('        <mesh filename="package://rover_description/meshes/fixed.stl"/>')
w('      </geometry>')
w(f'      <material name="{get_material("fixed")[0]}"/>')
w('    </visual>')
w('    <collision>')
w(f'      <origin xyz="{c_fixed["com_assembly"][0]:.4f} {c_fixed["com_assembly"][1]:.4f} {c_fixed["com_assembly"][2]:.4f}" rpy="0 0 0"/>')
w(f'      <geometry>')
w(f'        <box size="{size_f[0]:.4f} {size_f[1]:.4f} {size_f[2]:.4f}"/>')
w(f'      </geometry>')
w('    </collision>')
w('  </link>')
w()

# Helper to write links
def write_link(name, collision_type='none'):
    c = components[name]
    com = c['com_local']
    I = c['inertia']
    m = c['mass']
    mat_name = get_material(name)[0]
    
    w(f'  <link name="{name}">')
    w('    <inertial>')
    w(f'      <origin xyz="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" rpy="0 0 0"/>')
    w(f'      <mass value="{m:.4f}"/>')
    w(f'      <inertia ixx="{I[0]:.6e}" ixy="{I[3]:.6e}" ixz="{I[4]:.6e}"')
    w(f'               iyy="{I[1]:.6e}" iyz="{I[5]:.6e}" izz="{I[2]:.6e}"/>')
    w('    </inertial>')
    w('    <visual>')
    w('      <origin xyz="0 0 0" rpy="0 0 0"/>')
    w('      <geometry>')
    w(f'        <mesh filename="package://rover_description/meshes/{name}.stl"/>')
    w('      </geometry>')
    w(f'      <material name="{mat_name}"/>')
    w('    </visual>')
    if collision_type == 'cylinder_x':
        # Wheel cylinder aligned along X axis (rpy="0 1.5707963 0")
        w('    <collision>')
        w('      <origin xyz="0 0 0" rpy="0 1.5707963 0"/>')
        w('      <geometry>')
        w('        <cylinder radius="0.040" length="0.080"/>')
        w('      </geometry>')
        w('    </collision>')
    elif collision_type == 'box':
        s = c['size_m']
        w('    <collision>')
        w(f'      <origin xyz="{com[0]:.4f} {com[1]:.4f} {com[2]:.4f}" rpy="0 0 0"/>')
        w('      <geometry>')
        w(f'        <box size="{s[0]:.4f} {s[1]:.4f} {s[2]:.4f}"/>')
        w('      </geometry>')
        w('    </collision>')
    w('  </link>')
    w()

def write_joint(name, jtype, parent, child, axis=None, lo=None, hi=None, eff=100.0, vel=10.0, damp=0.1, fric=0.05):
    # Origin of child relative to parent
    p_child = link_origins[child]
    p_parent = link_origins.get(parent, (0.0, 0.0, 0.0))
    rel_x = p_child[0] - p_parent[0]
    rel_y = p_child[1] - p_parent[1]
    rel_z = p_child[2] - p_parent[2]
    
    w(f'  <joint name="{name}" type="{jtype}">')
    w(f'    <parent link="{parent}"/>')
    w(f'    <child link="{child}"/>')
    w(f'    <origin xyz="{rel_x:.4f} {rel_y:.4f} {rel_z:.4f}" rpy="0 0 0"/>')
    if axis:
        w(f'    <axis xyz="{axis}"/>')
    if jtype in ('revolute', 'prismatic', 'continuous'):
        w(f'    <dynamics damping="{damp}" friction="{fric}"/>')
    if jtype in ('revolute', 'prismatic'):
        w(f'    <limit lower="{lo}" upper="{hi}" effort="{eff}" velocity="{vel}"/>')
    w('  </joint>')
    w()

# ═════════ 4 LEG SYSTEMS (FL, FR, BL, BR) ═════════
for leg in ['FL', 'FR', 'BL', 'BR']:
    w(f'  <!-- ═════════ LEG {leg} ═════════ -->')
    is_front = 'F' in leg
    is_left = 'L' in leg
    sign_x = -1.0 if is_left else 1.0
    sign_y = 1.0 if is_front else -1.0
    
    # 1. input bracket
    input_name = f'leg{leg}_input'
    write_link(input_name)
    write_joint(f'{input_name}_joint', 'fixed', 'base_link', input_name)
    
    # 2. upper A-arm (revolute suspension pivot about X)
    upper_name = f'leg{leg}_upper'
    write_link(upper_name)
    write_joint(f'{upper_name}_joint', 'revolute', input_name, upper_name,
                axis='1 0 0', lo=-0.25, hi=0.25, eff=250.0, vel=2.0, damp=5.0, fric=0.5)
    
    # 3. lower A-arm (revolute suspension pivot about X)
    lower_name = f'leg{leg}_lower'
    write_link(lower_name)
    write_joint(f'{lower_name}_joint', 'revolute', input_name, lower_name,
                axis='1 0 0', lo=-0.25, hi=0.25, eff=250.0, vel=2.0, damp=5.0, fric=0.5)
    
    # 4. shock body (damper top mount on upper arm)
    shockbody_name = f'leg{leg}_shockbody'
    write_link(shockbody_name)
    write_joint(f'{shockbody_name}_joint', 'revolute', upper_name, shockbody_name,
                axis='1 0 0', lo=-0.2, hi=0.2, eff=200.0, vel=2.0, damp=5.0, fric=0.2)
    
    # 5. shock rod (prismatic shock absorber piston with spring/damping)
    shockrod_name = f'leg{leg}_shockrod'
    shock_dir_y = -0.9027 if is_front else 0.9027
    write_link(shockrod_name)
    write_joint(f'{shockrod_name}_joint', 'prismatic', shockbody_name, shockrod_name,
                axis=f'0 {shock_dir_y:.4f} -0.4303', lo=-0.020, hi=0.020, eff=1000.0, vel=1.0, damp=50.0, fric=2.0)
    
    # 6. steering knuckle (revolute about Z)
    steer_name = f'leg{leg}_steer'
    write_link(steer_name)
    write_joint(f'{steer_name}_joint', 'revolute', input_name, steer_name,
                axis='0 0 1', lo=-0.65, hi=0.65, eff=50.0, vel=3.0, damp=1.0, fric=0.1)
    
    # 7. wheel carrier hub (fixed to steering knuckle)
    carrier_name = f'leg{leg}_carrier'
    write_link(carrier_name)
    write_joint(f'{carrier_name}_joint', 'fixed', steer_name, carrier_name)
    
    # 8. motor drive shaft (continuous rotating shaft about X)
    drive_name = f'leg{leg}_drive'
    write_link(drive_name)
    write_joint(f'{drive_name}_joint', 'continuous', steer_name, drive_name,
                axis='-1 0 0', damp=0.02, fric=0.01)
    
    # 9. driven wheel (continuous rotating wheel about X)
    wheel_name = f'leg{leg}_wheel'
    write_link(wheel_name, collision_type='cylinder_x')
    write_joint(f'{wheel_name}_joint', 'continuous', steer_name, wheel_name,
                axis='-1 0 0', damp=0.05, fric=0.01)

# ═════════ TELESCOPIC CHASSIS (LEFT & RIGHT) ═════════
w('  <!-- ═════════ TELESCOPIC CHASSIS ═════════ -->')
# Left side stages
prev_left = 'base_link'
for i in range(1, 7):
    stage_name = f'telL{i}'
    write_link(stage_name)
    write_joint(f'{stage_name}_joint', 'prismatic', prev_left, stage_name,
                axis='-1 0 0', lo=0.0, hi=0.05, eff=150.0, vel=0.2, damp=10.0, fric=2.0)
    prev_left = stage_name

# Right side stages
prev_right = 'base_link'
for i in range(1, 7):
    stage_name = f'telR{i}'
    write_link(stage_name)
    write_joint(f'{stage_name}_joint', 'prismatic', prev_right, stage_name,
                axis='1 0 0', lo=0.0, hi=0.05, eff=150.0, vel=0.2, damp=10.0, fric=2.0)
    prev_right = stage_name

# ═════════ VISION PAN/TILT SYSTEM ═════════
w('  <!-- ═════════ PAN/TILT CAMERA ═════════ -->')
write_link('VISION_camera_yoke')
write_joint('camera_pan_joint', 'revolute', 'base_link', 'VISION_camera_yoke',
            axis='0 0 1', lo=-3.1415, hi=3.1415, eff=10.0, vel=3.0, damp=0.1, fric=0.02)

write_link('VISION_tilt_cradle')
write_joint('camera_tilt_joint', 'revolute', 'VISION_camera_yoke', 'VISION_tilt_cradle',
            axis='1 0 0', lo=-1.5707, hi=0.7854, eff=10.0, vel=3.0, damp=0.1, fric=0.02)

# Camera optical frame & sensor
w('  <link name="camera_optical_frame"/>')
w('  <joint name="camera_optical_joint" type="fixed">')
w('    <parent link="VISION_tilt_cradle"/>')
w('    <child link="camera_optical_frame"/>')
w('    <origin xyz="0 0.04 0" rpy="-1.5707963 0 -1.5707963"/>')
w('  </joint>')
w()

# IMU Sensor Link & Joint (Section 0 & Section 4 D3 of specification)
w('  <!-- ═════════ IMU SENSOR ═════════ -->')
w('  <link name="imu_link"/>')
w('  <joint name="imu_joint" type="fixed">')
w('    <parent link="base_link"/>')
w('    <child link="imu_link"/>')
w('    <origin xyz="0 0 0.15" rpy="0 0 0"/>')
w('  </joint>')
w()

# ═════════ GUIDE RAILS, ROLLERS & REMAINING PARTS ═════════
w('  <!-- ═════════ GUIDE RAILS, PULLEYS & REMAINING COMPONENTS ═════════ -->')
already_written = set(['fixed', 'base_link', 'base_footprint', 'camera_optical_frame', 'VISION_camera_yoke', 'VISION_tilt_cradle'])
for leg in ['FL', 'FR', 'BL', 'BR']:
    for p in ['input', 'upper', 'lower', 'shockbody', 'shockrod', 'steer', 'carrier', 'drive', 'wheel']:
        already_written.add(f'leg{leg}_{p}')
for i in range(1, 7):
    already_written.add(f'telL{i}')
    already_written.add(f'telR{i}')

for name in sorted(components.keys()):
    if name not in already_written:
        # Determine parent:
        # If it's a left pulley or guide, attach to telL or base_link
        parent = 'base_link'
        if name.startswith('rotL') or name.startswith('guideL'):
            parent = 'telL2' if '2' in name else 'telL1'
        elif name.startswith('rotR') or name.startswith('guideR'):
            parent = 'telR2' if '2' in name else 'telR1'
        elif 'armBL' in name or 'hex_follower_BL' in name or 'tipBL' in name:
            parent = 'telL4'
        elif 'armFL' in name or 'hex_follower_FL' in name or 'tipFL' in name:
            parent = 'telL4'
        elif 'armBR' in name or 'hex_follower_BR' in name or 'tipBR' in name:
            parent = 'telR4'
        elif 'armFR' in name or 'hex_follower_FR' in name or 'tipFR' in name:
            parent = 'telR4'
            
        write_link(name)
        write_joint(f'{name}_joint', 'fixed', parent, name)
        already_written.add(name)

# ═════════ GAZEBO SIM PLUGINS & CONTACT PHYSICS ═════════
w('  <!-- ═════════ GAZEBO SIMULATION CONFIGURATION ═════════ -->')
# Wheels friction and contact properties
for leg in ['FL', 'FR', 'BL', 'BR']:
    wn = f'leg{leg}_wheel'
    w(f'  <gazebo reference="{wn}">')
    w('    <mu1>1.2</mu1>')
    w('    <mu2>1.2</mu2>')
    w('    <kp>1000000.0</kp>')
    w('    <kd>100.0</kd>')
    w('    <minDepth>0.001</minDepth>')
    w('    <maxVel>1.0</maxVel>')
    w('    <material>Gazebo/DarkGrey</material>')
    w('  </gazebo>')
w()

# Camera sensor
w('  <gazebo reference="camera_optical_frame">')
w('    <sensor name="rover_camera" type="camera">')
w('      <camera>')
w('        <horizontal_fov>1.3962634</horizontal_fov>')
w('        <image>')
w('          <width>1280</width>')
w('          <height>720</height>')
w('          <format>R8G8B8</format>')
w('        </image>')
w('        <clip>')
w('          <near>0.05</near>')
w('          <far>100.0</far>')
w('        </clip>')
w('      </camera>')
w('      <always_on>1</always_on>')
w('      <update_rate>30</update_rate>')
w('      <visualize>true</visualize>')
w('      <topic>/camera/image_raw</topic>')
w('      <gz_frame_id>camera_optical_frame</gz_frame_id>')
w('    </sensor>')
w('  </gazebo>')
w()

# IMU Sensor Plugin (Section 4 D3 of specification)
w('  <gazebo reference="imu_link">')
w('    <sensor name="rover_imu" type="imu">')
w('      <always_on>1</always_on>')
w('      <update_rate>100</update_rate>')
w('      <visualize>false</visualize>')
w('      <topic>/imu/data</topic>')
w('    </sensor>')
w('  </gazebo>')
w()

# Gazebo Sim System Plugins
w('  <gazebo>')
# 1. Joint state publisher system
w('    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">')
w('      <topic>/joint_states</topic>')
w('      <update_rate>50</update_rate>')
w('    </plugin>')

# 2. Diff drive / skid steer driving plugin for the 4 wheels!
w('    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">')
w('      <left_joint>legFL_wheel_joint</left_joint>')
w('      <left_joint>legBL_wheel_joint</left_joint>')
w('      <right_joint>legFR_wheel_joint</right_joint>')
w('      <right_joint>legBR_wheel_joint</right_joint>')
w('      <wheel_separation>0.476</wheel_separation>')
w('      <wheel_radius>0.040</wheel_radius>')
w('      <max_wheel_torque>100.0</max_wheel_torque>')
w('      <max_wheel_acceleration>25.0</max_wheel_acceleration>')
w('      <odom_publish_frequency>50</odom_publish_frequency>')
w('      <cmd_vel_timeout>0.5</cmd_vel_timeout>')
w('      <topic>/cmd_vel</topic>')
w('      <odom_topic>/odom</odom_topic>')
w('      <tf_topic>/tf</tf_topic>')
w('      <frame_id>odom</frame_id>')
w('      <child_frame_id>base_footprint</child_frame_id>')
w('    </plugin>')

# 3. Steering joint position controller plugins for all 4 steer knuckles
for leg in ['FL', 'FR', 'BL', 'BR']:
    sj = f'leg{leg}_steer_joint'
    w('    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">')
    w(f'      <joint_name>{sj}</joint_name>')
    w(f'      <topic>/rover/steer_{leg.lower()}/cmd_pos</topic>')
    w('      <p_gain>500.0</p_gain>')
    w('      <i_gain>10.0</i_gain>')
    w('      <d_gain>5.0</d_gain>')
    w('      <i_max>50.0</i_max>')
    w('      <i_min>-50.0</i_min>')
    w('      <cmd_max>100.0</cmd_max>')
    w('      <cmd_min>-100.0</cmd_min>')
    w('    </plugin>')

# 4. Drive motor shafts continuous velocity controller
for leg in ['FL', 'FR', 'BL', 'BR']:
    dj = f'leg{leg}_drive_joint'
    w('    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">')
    w(f'      <joint_name>{dj}</joint_name>')
    w(f'      <topic>/rover/motor_{leg.lower()}/cmd_vel</topic>')
    w('      <use_velocity_commands>true</use_velocity_commands>')
    w('    </plugin>')

# 5. Telescopic prismatic position controllers
for side in ['L', 'R']:
    for idx in range(1, 7):
        tj = f'tel{side}{idx}_joint'
        w('    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">')
        w(f'      <joint_name>{tj}</joint_name>')
        w(f'      <topic>/rover/tele_{side.lower()}{idx}/cmd_pos</topic>')
        w('      <p_gain>2000.0</p_gain>')
        w('      <d_gain>50.0</d_gain>')
        w('    </plugin>')

# 6. Camera Pan/Tilt position controllers
w('    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">')
w('      <joint_name>camera_pan_joint</joint_name>')
w('      <topic>/rover/camera/pan/cmd_pos</topic>')
w('      <p_gain>100.0</p_gain>')
w('      <d_gain>2.0</d_gain>')
w('    </plugin>')
w('    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">')
w('      <joint_name>camera_tilt_joint</joint_name>')
w('      <topic>/rover/camera/tilt/cmd_pos</topic>')
w('      <p_gain>100.0</p_gain>')
w('      <d_gain>2.0</d_gain>')
w('    </plugin>')

w('  </gazebo>')
w()

w('</robot>')

with open(URDF_PATH, 'w') as f:
    f.write('\n'.join(lines))

print(f"\nURDF generation complete! Saved to {URDF_PATH}")
print("="*60)
