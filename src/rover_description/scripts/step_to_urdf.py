#!/usr/bin/env python3
"""
STEP to URDF Converter for Hexagonal Telescopic Chassis Rover
=============================================================
Parses the STEP assembly, extracts each part as an STL mesh,
computes inertial properties (assuming realistic material densities),
and generates a complete URDF with proper kinematic chains.

Author: Auto-generated for Gazebo simulation
"""

import os
import sys
import re
import math
import struct
import hashlib
from pathlib import Path
from collections import defaultdict

# Add venv to path
VENV_SITE = "/home/diya/cad_env/lib/python3.14/site-packages"
if VENV_SITE not in sys.path:
    sys.path.insert(0, VENV_SITE)

import numpy as np

# ============================================================
# STEP file parser: extract component names and transforms
# ============================================================

def parse_step_products(step_file):
    """Extract product names from STEP file."""
    products = {}
    with open(step_file, 'r', errors='replace') as f:
        content = f.read()

    # Find PRODUCT entries
    pattern = r"#(\d+)=PRODUCT\('([^']*?)','([^']*?)'"
    for match in re.finditer(pattern, content):
        entity_id = int(match.group(1))
        name = match.group(2)
        desc = match.group(3)
        # Use descriptive name if first name looks like a timestamp
        if name.startswith('20') and '-' in name and len(name) > 15:
            name = desc
        products[entity_id] = name
    return products


def parse_step_assembly(step_file):
    """Extract NAUO (assembly parent-child) relations from STEP file."""
    relations = []
    with open(step_file, 'r', errors='replace') as f:
        content = f.read()

    # NEXT_ASSEMBLY_USAGE_OCCURRENCE
    pattern = r"#(\d+)=NEXT_ASSEMBLY_USAGE_OCCURRENCE\('([^']*?)','[^']*?','[^']*?',#(\d+),\s*#(\d+)"
    for match in re.finditer(pattern, content):
        nauo_id = int(match.group(1))
        name = match.group(2)
        parent_pds = int(match.group(3))
        child_pds = int(match.group(4))
        relations.append({
            'id': nauo_id,
            'name': name,
            'parent': parent_pds,
            'child': child_pds
        })
    return relations


# ============================================================
# Use CadQuery / OCP to load STEP & export individual STL parts
# ============================================================

def extract_parts_with_cadquery(step_file, output_dir):
    """
    Load STEP assembly with OCP/CadQuery, tessellate each part into STL.
    Returns dict of {part_name: {'stl_path': ..., 'mass': ..., 'inertia': ..., 'com': ..., 'bbox': ...}}
    """
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_SOLID, TopAbs_FACE, TopAbs_COMPOUND
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.StlAPI import StlAPI_Writer
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.BRepGProp import brepgprop
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    from OCC.Core.TopoDS import topods
    from OCC.Core.TDocStd import TDocStd_Document
    from OCC.Core.XCAFDoc import XCAFDoc_ShapeTool, XCAFDoc_DocumentTool, XCAFDoc_ColorTool
    from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
    from OCC.Core.TDF import TDF_LabelSequence, TDF_Label
    from OCC.Core.TCollection import TCollection_ExtendedString
    from OCC.Core.XCAFDoc import XCAFDoc_Location
    from OCC.Core.TopLoc import TopLoc_Location
    from OCC.Core.gp import gp_Trsf, gp_XYZ

    os.makedirs(output_dir, exist_ok=True)

    # Use XDE reader to preserve assembly structure and names
    doc = TDocStd_Document(TCollection_ExtendedString("pythonocc-doc"))
    reader = STEPCAFControl_Reader()
    reader.SetColorMode(True)
    reader.SetNameMode(True)
    reader.SetLayerMode(True)

    status = reader.ReadFile(step_file)
    if status != IFSelect_RetDone:
        raise RuntimeError(f"Failed to read STEP file: {step_file}")

    reader.Transfer(doc)

    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    color_tool = XCAFDoc_DocumentTool.ColorTool(doc.Main())

    parts = {}

    def get_label_name(label):
        """Get the name of a label."""
        from OCC.Core.TDataStd import TDataStd_Name
        name_attr = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID(), name_attr):
            return name_attr.Get().ToExtString()
        return None

    def process_label(label, prefix="", depth=0):
        """Recursively process assembly tree labels."""
        name = get_label_name(label)
        if name is None:
            name = f"part_{label.Tag()}"

        # Clean name
        clean_name = re.sub(r'[^a-zA-Z0-9_]', '_', name)
        if clean_name and clean_name[0].isdigit():
            clean_name = 'p_' + clean_name

        shape = shape_tool.GetShape(label)
        if shape.IsNull():
            return

        if shape_tool.IsSimpleShape(label):
            # This is a leaf shape - export as STL
            stl_name = clean_name
            # Ensure unique names
            counter = 1
            base_name = stl_name
            while stl_name in parts:
                stl_name = f"{base_name}_{counter}"
                counter += 1

            stl_path = os.path.join(output_dir, f"{stl_name}.stl")

            # Tessellate
            mesh = BRepMesh_IncrementalMesh(shape, 0.5, False, 0.5, True)
            mesh.Perform()

            # Write STL
            writer = StlAPI_Writer()
            writer.SetASCIIMode(False)  # binary STL for smaller files
            success = writer.Write(shape, stl_path)

            if success and os.path.exists(stl_path) and os.path.getsize(stl_path) > 84:
                # Compute volume and inertia
                props = GProp_GProps()
                brepgprop.VolumeProperties(shape, props)
                volume_mm3 = props.Mass()  # in mm^3 since STEP is typically in mm
                volume_m3 = volume_mm3 * 1e-9  # convert to m^3

                # Compute surface area
                sprops = GProp_GProps()
                brepgprop.SurfaceProperties(shape, sprops)

                # Center of mass
                com = props.CentreOfMass()
                com_m = (com.X() * 0.001, com.Y() * 0.001, com.Z() * 0.001)

                # Bounding box
                bbox = Bnd_Box()
                brepbndlib.Add(shape, bbox)
                xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
                bbox_m = {
                    'min': (xmin * 0.001, ymin * 0.001, zmin * 0.001),
                    'max': (xmax * 0.001, ymax * 0.001, zmax * 0.001),
                    'size': ((xmax - xmin) * 0.001, (ymax - ymin) * 0.001, (zmax - zmin) * 0.001)
                }

                # Assign material density based on part name
                density = assign_density(name)
                mass = max(abs(volume_m3) * density, 0.001)

                # Inertia tensor at center of mass
                mat = props.MatrixOfInertia()
                # Scale from mm^5 * density_equiv to kg*m^2
                scale = density * 1e-9  # mm^3 -> m^3 for volume, mm^2 -> m^2 for radius^2
                # OCC returns moments in volume units, need to multiply by density and convert
                ixx = abs(mat.Value(1, 1)) * density * 1e-15
                iyy = abs(mat.Value(2, 2)) * density * 1e-15
                izz = abs(mat.Value(3, 3)) * density * 1e-15
                ixy = mat.Value(1, 2) * density * 1e-15
                ixz = mat.Value(1, 3) * density * 1e-15
                iyz = mat.Value(2, 3) * density * 1e-15

                # Ensure minimum inertia
                min_inertia = 1e-6
                ixx = max(ixx, min_inertia)
                iyy = max(iyy, min_inertia)
                izz = max(izz, min_inertia)

                parts[stl_name] = {
                    'stl_path': stl_path,
                    'original_name': name,
                    'mass': mass,
                    'volume_m3': volume_m3,
                    'com': com_m,
                    'inertia': {
                        'ixx': ixx, 'iyy': iyy, 'izz': izz,
                        'ixy': ixy, 'ixz': ixz, 'iyz': iyz
                    },
                    'bbox': bbox_m
                }
                print(f"  Exported: {stl_name} (mass={mass:.4f} kg, vol={volume_m3*1e6:.2f} cm³)")
            else:
                print(f"  SKIP (empty): {stl_name}")

        # Process children
        if shape_tool.IsAssembly(label):
            children = TDF_LabelSequence()
            shape_tool.GetComponents(label, children)
            for i in range(children.Length()):
                child = children.Value(i + 1)
                ref_label = TDF_Label()
                if shape_tool.GetReferredShape(child, ref_label):
                    process_label(ref_label, prefix=f"{clean_name}/", depth=depth + 1)

    # Get free shapes (top-level)
    free_shapes = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free_shapes)

    print(f"\nFound {free_shapes.Length()} top-level shape(s)")

    for i in range(free_shapes.Length()):
        label = free_shapes.Value(i + 1)
        process_label(label)

    return parts


def assign_density(part_name):
    """Assign material density (kg/m³) based on part naming conventions."""
    name_lower = part_name.lower()

    # Wheels - rubber/composite
    if 'wheel' in name_lower or 'tire' in name_lower:
        return 1100  # rubber ~1100 kg/m³

    # Shock absorbers - steel
    if 'shock' in name_lower:
        return 7850  # steel

    # Drive components - steel/aluminum mix
    if 'drive' in name_lower or 'motor' in name_lower or 'input' in name_lower:
        return 5000  # mix of steel gears + aluminum housing

    # Structural arms - aluminum
    if 'arm' in name_lower or 'upper' in name_lower or 'lower' in name_lower:
        return 2700  # aluminum

    # Hexagonal mechanism - aluminum
    if 'hex' in name_lower or 'rot' in name_lower:
        return 2700  # aluminum

    # Telescopic tubes - aluminum
    if 'tel' in name_lower:
        return 2700  # aluminum

    # Guide rails - steel
    if 'guide' in name_lower:
        return 7850  # steel

    # Steering - aluminum/steel
    if 'steer' in name_lower:
        return 4000  # aluminum with steel bearings

    # Carrier/knuckle - steel
    if 'carrier' in name_lower:
        return 7850  # steel

    # Camera/vision - mixed (plastic + electronics)
    if 'vision' in name_lower or 'camera' in name_lower:
        return 1500  # plastic + electronics

    # Fixed frame / chassis
    if 'fixed' in name_lower or 'chassis' in name_lower or 'frame' in name_lower:
        return 2700  # aluminum

    # Tips
    if 'tip' in name_lower:
        return 2700  # aluminum

    # Follower links
    if 'follower' in name_lower:
        return 2700  # aluminum

    # Default - aluminum
    return 2700


# ============================================================
# Alternative: use gmsh for STEP -> STL conversion
# ============================================================

def extract_parts_with_gmsh(step_file, output_dir):
    """
    Use gmsh to load STEP and export all solids as individual STLs.
    Fallback approach if CadQuery/OCP is problematic.
    """
    import gmsh

    os.makedirs(output_dir, exist_ok=True)

    gmsh.initialize()
    gmsh.option.setNumber("General.Verbosity", 1)
    gmsh.option.setNumber("Mesh.MeshSizeMax", 3.0)  # mm
    gmsh.option.setNumber("Mesh.MeshSizeMin", 0.5)  # mm

    print(f"Loading STEP file: {step_file}")
    gmsh.open(step_file)

    # Get all volumes (3D entities)
    volumes = gmsh.model.getEntities(3)
    surfaces = gmsh.model.getEntities(2)
    print(f"Found {len(volumes)} volumes and {len(surfaces)} surfaces")

    # Get entity names
    parts = {}
    for dim, tag in volumes:
        name = gmsh.model.getEntityName(dim, tag)
        if not name:
            name = f"solid_{tag}"
        # Clean the name
        # Names from STEP often include hierarchy like "Assembly/Part"
        clean = name.split('/')[-1] if '/' in name else name
        clean = re.sub(r'[^a-zA-Z0-9_]', '_', clean)
        if clean and clean[0].isdigit():
            clean = 'p_' + clean

        print(f"  Volume {tag}: {name} -> {clean}")
        parts[tag] = {
            'name': clean,
            'original_name': name,
            'dim': dim,
            'tag': tag
        }

    gmsh.finalize()

    return parts


# ============================================================
# Direct STEP parser for transforms (without OCP XDE)
# ============================================================

def parse_step_transforms(step_file):
    """
    Parse AXIS2_PLACEMENT_3D and related entities from STEP
    to extract placement transforms for each component.
    """
    with open(step_file, 'r', errors='replace') as f:
        content = f.read()

    # Parse CARTESIAN_POINT
    points = {}
    for m in re.finditer(r"#(\d+)=CARTESIAN_POINT\('([^']*)',\(([^)]*)\)\)", content):
        eid = int(m.group(1))
        coords = [float(x.strip()) for x in m.group(3).split(',')]
        points[eid] = coords

    # Parse DIRECTION
    directions = {}
    for m in re.finditer(r"#(\d+)=DIRECTION\('([^']*)',\(([^)]*)\)\)", content):
        eid = int(m.group(1))
        coords = [float(x.strip()) for x in m.group(3).split(',')]
        directions[eid] = coords

    # Parse AXIS2_PLACEMENT_3D
    placements = {}
    for m in re.finditer(r"#(\d+)=AXIS2_PLACEMENT_3D\('([^']*)',#(\d+),#(\d+),#(\d+)\)", content):
        eid = int(m.group(1))
        pt_id = int(m.group(3))
        dir_z_id = int(m.group(4))
        dir_x_id = int(m.group(5))
        if pt_id in points and dir_z_id in directions and dir_x_id in directions:
            placements[eid] = {
                'origin': points[pt_id],
                'z_axis': directions[dir_z_id],
                'x_axis': directions[dir_x_id]
            }

    return placements


# ============================================================
# Full pipeline: parse STEP, extract STLs, build URDF
# ============================================================

def convert_stl_mm_to_m(stl_path):
    """Convert an STL file from mm to meters (scale by 0.001)."""
    try:
        from stl import mesh as stl_mesh
        m = stl_mesh.Mesh.from_file(stl_path)
        m.vectors *= 0.001  # mm -> m
        m.save(stl_path)
        return True
    except:
        # Manual binary STL scaling
        try:
            with open(stl_path, 'rb') as f:
                header = f.read(80)
                num_triangles = struct.unpack('<I', f.read(4))[0]
                data = f.read()

            with open(stl_path, 'wb') as f:
                f.write(header)
                f.write(struct.pack('<I', num_triangles))
                offset = 0
                for i in range(num_triangles):
                    # Normal (3 floats) + 3 vertices (9 floats) + attribute (2 bytes)
                    normal = struct.unpack_from('<3f', data, offset)
                    offset += 12
                    vertices = []
                    for v in range(3):
                        vx, vy, vz = struct.unpack_from('<3f', data, offset)
                        vertices.append((vx * 0.001, vy * 0.001, vz * 0.001))
                        offset += 12
                    attr = struct.unpack_from('<H', data, offset)[0]
                    offset += 2

                    f.write(struct.pack('<3f', *normal))
                    for vx, vy, vz in vertices:
                        f.write(struct.pack('<3f', vx, vy, vz))
                    f.write(struct.pack('<H', attr))
            return True
        except Exception as e:
            print(f"Warning: Could not scale STL {stl_path}: {e}")
            return False


def build_urdf(parts, output_dir, urdf_path):
    """
    Build a complete URDF file for the hexagonal telescopic chassis rover.

    Kinematic chain per leg (FL, FR, BL, BR):
      chassis (fixed) -> arm_XX (revolute, hex expansion)
                       -> hexarm_XX (revolute)
      chassis -> legXX_input (revolute, steering input)
             -> legXX_upper (revolute, upper suspension)
             -> legXX_shockbody (prismatic, shock absorber)
             -> legXX_shockrod (prismatic, shock extension)
             -> legXX_lower (revolute, lower suspension)
             -> legXX_carrier (revolute, knuckle)
             -> legXX_steer (revolute, steering)
             -> legXX_drive (continuous, drive motor)
             -> legXX_wheel (continuous, wheel spin)

    Telescopic mechanism:
      chassis -> telL/R segments (prismatic joints)

    Vision system:
      chassis -> VISION_camera_yoke (revolute, pan)
             -> VISION_tilt_cradle (revolute, tilt)
    """

    mesh_dir = os.path.join(output_dir, '..', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)

    # Copy/move STL files to meshes directory and scale to meters
    for name, info in parts.items():
        src = info['stl_path']
        dst = os.path.join(mesh_dir, f"{name}.stl")
        if src != dst:
            import shutil
            shutil.copy2(src, dst)
        # Scale STL from mm to meters
        convert_stl_mm_to_m(dst)
        info['mesh_path'] = f"package://rover_description/meshes/{name}.stl"
        info['mesh_path_abs'] = dst

    # ================================================================
    # URDF Generation
    # ================================================================

    xml_lines = []
    xml_lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    xml_lines.append('<robot name="hexagonal_telescopic_rover" xmlns:xacro="http://www.ros.org/wiki/xacro">')
    xml_lines.append('')
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- Hexagonal Telescopic Chassis Rover URDF      -->')
    xml_lines.append('  <!-- Generated from seee1.step CAD model          -->')
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('')

    # Gazebo plugin for differential drive / control
    xml_lines.append('  <!-- Gazebo ROS Control Plugin -->')
    xml_lines.append('  <gazebo>')
    xml_lines.append('    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">')
    xml_lines.append('      <robotNamespace>/rover</robotNamespace>')
    xml_lines.append('    </plugin>')
    xml_lines.append('  </gazebo>')
    xml_lines.append('')

    # Material definitions
    xml_lines.append('  <!-- Material Definitions -->')
    materials = {
        'aluminum': (0.7, 0.7, 0.75, 1.0),
        'dark_aluminum': (0.3, 0.3, 0.35, 1.0),
        'steel': (0.5, 0.5, 0.55, 1.0),
        'rubber': (0.15, 0.15, 0.15, 1.0),
        'red_anodized': (0.8, 0.1, 0.1, 1.0),
        'blue_anodized': (0.1, 0.2, 0.8, 1.0),
        'black_plastic': (0.1, 0.1, 0.1, 1.0),
        'orange': (1.0, 0.5, 0.0, 1.0),
    }
    for mat_name, rgba in materials.items():
        xml_lines.append(f'  <material name="{mat_name}">')
        xml_lines.append(f'    <color rgba="{rgba[0]} {rgba[1]} {rgba[2]} {rgba[3]}"/>')
        xml_lines.append(f'  </material>')
    xml_lines.append('')

    def get_material(part_name):
        n = part_name.lower()
        if 'wheel' in n: return 'rubber'
        if 'shock' in n: return 'steel'
        if 'drive' in n or 'motor' in n: return 'dark_aluminum'
        if 'steer' in n: return 'blue_anodized'
        if 'hex' in n: return 'red_anodized'
        if 'vision' in n or 'camera' in n: return 'black_plastic'
        if 'tip' in n: return 'orange'
        if 'follower' in n: return 'red_anodized'
        return 'aluminum'

    def add_link(name, part_info=None, material='aluminum'):
        """Add a URDF link element."""
        xml_lines.append(f'  <link name="{name}">')

        if part_info and 'mesh_path' in part_info:
            mass = part_info.get('mass', 0.1)
            inertia = part_info.get('inertia', {})
            ixx = inertia.get('ixx', 1e-4)
            iyy = inertia.get('iyy', 1e-4)
            izz = inertia.get('izz', 1e-4)
            ixy = inertia.get('ixy', 0)
            ixz = inertia.get('ixz', 0)
            iyz = inertia.get('iyz', 0)

            xml_lines.append(f'    <inertial>')
            xml_lines.append(f'      <mass value="{mass:.6f}"/>')
            xml_lines.append(f'      <inertia ixx="{ixx:.8e}" ixy="{ixy:.8e}" ixz="{ixz:.8e}"')
            xml_lines.append(f'               iyy="{iyy:.8e}" iyz="{iyz:.8e}" izz="{izz:.8e}"/>')
            xml_lines.append(f'    </inertial>')

            xml_lines.append(f'    <visual>')
            xml_lines.append(f'      <geometry>')
            xml_lines.append(f'        <mesh filename="{part_info["mesh_path"]}"/>')
            xml_lines.append(f'      </geometry>')
            xml_lines.append(f'      <material name="{material}"/>')
            xml_lines.append(f'    </visual>')

            xml_lines.append(f'    <collision>')
            xml_lines.append(f'      <geometry>')
            xml_lines.append(f'        <mesh filename="{part_info["mesh_path"]}"/>')
            xml_lines.append(f'      </geometry>')
            xml_lines.append(f'    </collision>')
        else:
            # Dummy link
            xml_lines.append(f'    <inertial>')
            xml_lines.append(f'      <mass value="0.001"/>')
            xml_lines.append(f'      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>')
            xml_lines.append(f'    </inertial>')

        xml_lines.append(f'  </link>')
        xml_lines.append('')

    def add_joint(name, joint_type, parent, child,
                  xyz="0 0 0", rpy="0 0 0",
                  axis="0 0 1", limits=None,
                  damping=0.1, friction=0.05):
        """Add a URDF joint element."""
        xml_lines.append(f'  <joint name="{name}" type="{joint_type}">')
        xml_lines.append(f'    <parent link="{parent}"/>')
        xml_lines.append(f'    <child link="{child}"/>')
        xml_lines.append(f'    <origin xyz="{xyz}" rpy="{rpy}"/>')

        if joint_type != 'fixed':
            xml_lines.append(f'    <axis xyz="{axis}"/>')
            xml_lines.append(f'    <dynamics damping="{damping}" friction="{friction}"/>')

        if limits and joint_type in ('revolute', 'prismatic'):
            lower = limits.get('lower', -1.57)
            upper = limits.get('upper', 1.57)
            effort = limits.get('effort', 100)
            velocity = limits.get('velocity', 1.0)
            xml_lines.append(f'    <limit lower="{lower}" upper="{upper}" effort="{effort}" velocity="{velocity}"/>')

        xml_lines.append(f'  </joint>')
        xml_lines.append('')

    def add_gazebo_link(name, mu1=0.8, mu2=0.8, kp=1e6, kd=100):
        """Add Gazebo-specific link properties."""
        xml_lines.append(f'  <gazebo reference="{name}">')
        xml_lines.append(f'    <mu1>{mu1}</mu1>')
        xml_lines.append(f'    <mu2>{mu2}</mu2>')
        xml_lines.append(f'    <kp>{kp}</kp>')
        xml_lines.append(f'    <kd>{kd}</kd>')
        xml_lines.append(f'  </gazebo>')
        xml_lines.append('')

    def add_transmission(joint_name, hw_interface="hardware_interface/EffortJointInterface"):
        """Add a transmission for ros_control."""
        xml_lines.append(f'  <transmission name="trans_{joint_name}">')
        xml_lines.append(f'    <type>transmission_interface/SimpleTransmission</type>')
        xml_lines.append(f'    <joint name="{joint_name}">')
        xml_lines.append(f'      <hardwareInterface>{hw_interface}</hardwareInterface>')
        xml_lines.append(f'    </joint>')
        xml_lines.append(f'    <actuator name="motor_{joint_name}">')
        xml_lines.append(f'      <hardwareInterface>{hw_interface}</hardwareInterface>')
        xml_lines.append(f'      <mechanicalReduction>1</mechanicalReduction>')
        xml_lines.append(f'    </actuator>')
        xml_lines.append(f'  </transmission>')
        xml_lines.append('')

    # ================================================================
    # BASE LINK (fixed chassis)
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- BASE CHASSIS (fixed frame)                   -->')
    xml_lines.append('  <!-- ============================================ -->')

    # base_footprint for ground plane reference
    add_link('base_footprint')
    add_joint('base_footprint_joint', 'fixed', 'base_footprint', 'base_link',
              xyz='0 0 0.15')

    fixed_part = parts.get('fixed', None)
    add_link('base_link', fixed_part, 'aluminum')

    # ================================================================
    # SUSPENSION LEGS (FL, FR, BL, BR)
    # ================================================================
    # Leg positions (approximate from assembly - in meters)
    # Based on a typical rover with ~0.4m wheelbase
    leg_positions = {
        'FL': {'xyz': '0.25 0.20 0', 'sign_x': 1, 'sign_y': 1},
        'FR': {'xyz': '0.25 -0.20 0', 'sign_x': 1, 'sign_y': -1},
        'BL': {'xyz': '-0.25 0.20 0', 'sign_x': -1, 'sign_y': 1},
        'BR': {'xyz': '-0.25 -0.20 0', 'sign_x': -1, 'sign_y': -1},
    }

    for leg_id, pos in leg_positions.items():
        xml_lines.append(f'  <!-- ============================================ -->')
        xml_lines.append(f'  <!-- LEG {leg_id} - Suspension & Drive             -->')
        xml_lines.append(f'  <!-- ============================================ -->')

        prefix = f"leg{leg_id}"
        sx = pos['sign_x']
        sy = pos['sign_y']

        # --- Input link (steering input / actuator mount) ---
        input_part = parts.get(f'{prefix}_input', None)
        add_link(f'{prefix}_input', input_part, 'dark_aluminum')
        add_joint(f'{prefix}_input_joint', 'revolute',
                  'base_link', f'{prefix}_input',
                  xyz=pos['xyz'], rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -0.5, 'upper': 0.5, 'effort': 50, 'velocity': 2.0},
                  damping=0.5, friction=0.1)
        add_transmission(f'{prefix}_input_joint')

        # --- Upper suspension arm ---
        upper_part = parts.get(f'{prefix}_upper', None)
        add_link(f'{prefix}_upper', upper_part, 'aluminum')
        add_joint(f'{prefix}_upper_joint', 'revolute',
                  f'{prefix}_input', f'{prefix}_upper',
                  xyz='0 0 0', rpy='0 0 0',
                  axis='0 1 0',
                  limits={'lower': -0.3, 'upper': 0.3, 'effort': 200, 'velocity': 1.0},
                  damping=5.0, friction=0.5)

        # --- Shock absorber body (attached to upper arm) ---
        shockbody_part = parts.get(f'{prefix}_shockbody', None)
        add_link(f'{prefix}_shockbody', shockbody_part, 'steel')
        add_joint(f'{prefix}_shock_mount_joint', 'revolute',
                  f'{prefix}_upper', f'{prefix}_shockbody',
                  xyz='0 0 -0.02', rpy='0 0 0',
                  axis='0 1 0',
                  limits={'lower': -0.2, 'upper': 0.2, 'effort': 500, 'velocity': 0.5},
                  damping=50.0, friction=1.0)

        # --- Shock absorber rod (prismatic - the actual damping) ---
        shockrod_part = parts.get(f'{prefix}_shockrod', None)
        add_link(f'{prefix}_shockrod', shockrod_part, 'steel')
        add_joint(f'{prefix}_shock_joint', 'prismatic',
                  f'{prefix}_shockbody', f'{prefix}_shockrod',
                  xyz='0 0 0', rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -0.05, 'upper': 0.05, 'effort': 1000, 'velocity': 0.3},
                  damping=500.0, friction=10.0)

        # Add spring-like Gazebo joint properties for shock
        xml_lines.append(f'  <gazebo reference="{prefix}_shock_joint">')
        xml_lines.append(f'    <implicitSpringDamper>true</implicitSpringDamper>')
        xml_lines.append(f'    <springStiffness>5000</springStiffness>')
        xml_lines.append(f'    <springReference>0</springReference>')
        xml_lines.append(f'  </gazebo>')
        xml_lines.append('')

        # --- Lower suspension arm ---
        lower_part = parts.get(f'{prefix}_lower', None)
        add_link(f'{prefix}_lower', lower_part, 'aluminum')
        add_joint(f'{prefix}_lower_joint', 'revolute',
                  f'{prefix}_input', f'{prefix}_lower',
                  xyz='0 0 -0.05', rpy='0 0 0',
                  axis='0 1 0',
                  limits={'lower': -0.3, 'upper': 0.3, 'effort': 200, 'velocity': 1.0},
                  damping=5.0, friction=0.5)

        # --- Carrier / Knuckle ---
        carrier_part = parts.get(f'{prefix}_carrier', None)
        add_link(f'{prefix}_carrier', carrier_part, 'steel')
        add_joint(f'{prefix}_carrier_joint', 'revolute',
                  f'{prefix}_lower', f'{prefix}_carrier',
                  xyz=f'0 {0.05 * sy} -0.08', rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -0.1, 'upper': 0.1, 'effort': 100, 'velocity': 1.0},
                  damping=2.0, friction=0.3)

        # --- Steering link ---
        steer_part = parts.get(f'{prefix}_steer', None)
        add_link(f'{prefix}_steer', steer_part, 'blue_anodized')
        add_joint(f'{prefix}_steer_joint', 'revolute',
                  f'{prefix}_carrier', f'{prefix}_steer',
                  xyz='0 0 0', rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -0.6, 'upper': 0.6, 'effort': 30, 'velocity': 3.0},
                  damping=0.3, friction=0.05)
        add_transmission(f'{prefix}_steer_joint')

        # --- Drive (motor/hub) ---
        drive_part = parts.get(f'{prefix}_drive', None)
        add_link(f'{prefix}_drive', drive_part, 'dark_aluminum')
        add_joint(f'{prefix}_drive_joint', 'continuous',
                  f'{prefix}_steer', f'{prefix}_drive',
                  xyz='0 0 0', rpy='0 0 0',
                  axis='0 1 0',
                  damping=0.01, friction=0.005)
        add_transmission(f'{prefix}_drive_joint',
                        "hardware_interface/VelocityJointInterface")

        # --- Wheel ---
        wheel_part = parts.get(f'{prefix}_wheel', None)
        add_link(f'{prefix}_wheel', wheel_part, 'rubber')
        add_joint(f'{prefix}_wheel_joint', 'continuous',
                  f'{prefix}_drive', f'{prefix}_wheel',
                  xyz='0 0 0', rpy='0 0 0',
                  axis='0 1 0',
                  damping=0.005, friction=0.001)

        # Wheel Gazebo properties - high friction for ground contact
        add_gazebo_link(f'{prefix}_wheel', mu1=1.5, mu2=1.2, kp=1e7, kd=100)
        add_transmission(f'{prefix}_wheel_joint',
                        "hardware_interface/VelocityJointInterface")

    # ================================================================
    # HEXAGONAL EXPANSION MECHANISM
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- HEXAGONAL EXPANSION MECHANISM                -->')
    xml_lines.append('  <!-- ============================================ -->')

    # Hex arms (FL, FR, BL, BR)
    hex_arm_positions = {
        'FL': '0.15 0.12 0',
        'FR': '0.15 -0.12 0',
        'BL': '-0.15 0.12 0',
        'BR': '-0.15 -0.12 0',
    }
    for ha_id, ha_xyz in hex_arm_positions.items():
        ha_name = f'hexarm{ha_id}'
        ha_part = parts.get(ha_name, None)
        add_link(ha_name, ha_part, 'red_anodized')
        add_joint(f'{ha_name}_joint', 'revolute',
                  'base_link', ha_name,
                  xyz=ha_xyz, rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -1.0, 'upper': 1.0, 'effort': 50, 'velocity': 1.0},
                  damping=1.0, friction=0.2)
        add_transmission(f'{ha_name}_joint')

    # Arms (connecting links)
    arm_positions = {
        'FL': '0.20 0.15 0',
        'FR': '0.20 -0.15 0',
        'BL': '-0.20 0.15 0',
        'BR': '-0.20 -0.15 0',
    }
    for arm_id, arm_xyz in arm_positions.items():
        arm_name = f'arm{arm_id}'
        arm_part = parts.get(arm_name, None)
        add_link(arm_name, arm_part, 'aluminum')
        add_joint(f'{arm_name}_joint', 'revolute',
                  f'hexarm{arm_id}', arm_name,
                  xyz='0.05 0 0', rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -1.5, 'upper': 1.5, 'effort': 50, 'velocity': 1.0},
                  damping=1.0, friction=0.2)

    # Hex followers
    for hf_id in ['FL', 'FR', 'BL', 'BR']:
        hf_name = f'hex_follower_{hf_id}'
        hf_part = parts.get(hf_name, None)
        if hf_part:
            add_link(hf_name, hf_part, 'red_anodized')
            add_joint(f'{hf_name}_joint', 'revolute',
                      f'arm{hf_id}', hf_name,
                      xyz='0.03 0 0', rpy='0 0 0',
                      axis='0 0 1',
                      limits={'lower': -1.0, 'upper': 1.0, 'effort': 20, 'velocity': 1.0},
                      damping=0.5, friction=0.1)

    # Hex rotors
    hex_rotors = {
        'hexrotL240': {'parent': 'base_link', 'xyz': '0 0.10 0'},
        'hexrotL350': {'parent': 'base_link', 'xyz': '0 0.08 0'},
        'hexrotR240': {'parent': 'base_link', 'xyz': '0 -0.10 0'},
        'hexrotR350': {'parent': 'base_link', 'xyz': '0 -0.08 0'},
    }
    for hr_name, hr_info in hex_rotors.items():
        hr_part = parts.get(hr_name, None)
        add_link(hr_name, hr_part, 'red_anodized')
        add_joint(f'{hr_name}_joint', 'revolute',
                  hr_info['parent'], hr_name,
                  xyz=hr_info['xyz'], rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -3.14, 'upper': 3.14, 'effort': 30, 'velocity': 2.0},
                  damping=0.3, friction=0.05)
        add_transmission(f'{hr_name}_joint')

    # ================================================================
    # TELESCOPIC MECHANISM
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- TELESCOPIC MECHANISM (L & R sides)           -->')
    xml_lines.append('  <!-- ============================================ -->')

    # Left side telescopic segments
    for side in ['L', 'R']:
        y_offset = 0.15 if side == 'L' else -0.15
        prev_link = 'base_link'
        for seg in range(1, 7):
            tel_name = f'tel{side}{seg}'
            tel_part = parts.get(tel_name, None)
            add_link(tel_name, tel_part, 'aluminum')
            add_joint(f'{tel_name}_joint', 'prismatic',
                      prev_link, tel_name,
                      xyz=f'0 {y_offset if seg == 1 else 0} 0', rpy='0 0 0',
                      axis='1 0 0',
                      limits={'lower': -0.1, 'upper': 0.1, 'effort': 200, 'velocity': 0.5},
                      damping=10.0, friction=2.0)
            if seg == 1:
                add_transmission(f'{tel_name}_joint')
            prev_link = tel_name

        # Rotary elements for each side
        for rot_idx in range(5):
            for rot_size in ['145', '15']:
                rot_name = f'rot{side}{rot_idx}_{rot_size}'
                rot_part = parts.get(rot_name, None)
                if rot_part:
                    add_link(rot_name, rot_part, 'dark_aluminum')
                    add_joint(f'{rot_name}_joint', 'continuous',
                              'base_link', rot_name,
                              xyz=f'0 {y_offset} 0', rpy='0 0 0',
                              axis='1 0 0',
                              damping=0.1, friction=0.02)

    # ================================================================
    # GUIDE RAILS
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- GUIDE RAILS                                  -->')
    xml_lines.append('  <!-- ============================================ -->')

    for side in ['L', 'R']:
        for angle in ['-240', '240']:
            for idx in range(1, 4):
                guide_name = f'guide{side}_{angle}_{idx}'
                guide_part = parts.get(guide_name, None)
                if guide_part:
                    add_link(guide_name, guide_part, 'steel')
                    add_joint(f'{guide_name}_joint', 'fixed',
                              'base_link', guide_name)

    # ================================================================
    # TIPS (end effectors / bumpers)
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- TIPS                                         -->')
    xml_lines.append('  <!-- ============================================ -->')

    tip_positions = {
        'tipFL': {'parent': 'armFL', 'xyz': '0.08 0 0'},
        'tipFR': {'parent': 'armFR', 'xyz': '0.08 0 0'},
        'tipBL': {'parent': 'armBL', 'xyz': '-0.08 0 0'},
        'tipBR': {'parent': 'armBR', 'xyz': '-0.08 0 0'},
    }
    for tip_name, tip_info in tip_positions.items():
        tip_part = parts.get(tip_name, None)
        if tip_part:
            add_link(tip_name, tip_part, 'orange')
            add_joint(f'{tip_name}_joint', 'fixed',
                      tip_info['parent'], tip_name,
                      xyz=tip_info['xyz'])

    # ================================================================
    # VISION SYSTEM (Camera Pan/Tilt)
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- VISION SYSTEM (Pan/Tilt Camera)              -->')
    xml_lines.append('  <!-- ============================================ -->')

    yoke_part = parts.get('VISION_camera_yoke', None)
    if yoke_part:
        add_link('VISION_camera_yoke', yoke_part, 'black_plastic')
        add_joint('camera_pan_joint', 'revolute',
                  'base_link', 'VISION_camera_yoke',
                  xyz='0.05 0 0.10', rpy='0 0 0',
                  axis='0 0 1',
                  limits={'lower': -3.14, 'upper': 3.14, 'effort': 5, 'velocity': 3.0},
                  damping=0.05, friction=0.01)
        add_transmission('camera_pan_joint')

    tilt_part = parts.get('VISION_tilt_cradle', None)
    if tilt_part:
        add_link('VISION_tilt_cradle', tilt_part, 'black_plastic')
        add_joint('camera_tilt_joint', 'revolute',
                  'VISION_camera_yoke', 'VISION_tilt_cradle',
                  xyz='0 0 0.03', rpy='0 0 0',
                  axis='0 1 0',
                  limits={'lower': -1.57, 'upper': 0.5, 'effort': 5, 'velocity': 3.0},
                  damping=0.05, friction=0.01)
        add_transmission('camera_tilt_joint')

        # Add camera sensor
        xml_lines.append('  <gazebo reference="VISION_tilt_cradle">')
        xml_lines.append('    <sensor type="camera" name="rover_camera">')
        xml_lines.append('      <update_rate>30.0</update_rate>')
        xml_lines.append('      <camera name="head">')
        xml_lines.append('        <horizontal_fov>1.3962634</horizontal_fov>')
        xml_lines.append('        <image>')
        xml_lines.append('          <width>1280</width>')
        xml_lines.append('          <height>720</height>')
        xml_lines.append('          <format>R8G8B8</format>')
        xml_lines.append('        </image>')
        xml_lines.append('        <clip>')
        xml_lines.append('          <near>0.02</near>')
        xml_lines.append('          <far>300</far>')
        xml_lines.append('        </clip>')
        xml_lines.append('      </camera>')
        xml_lines.append('      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">')
        xml_lines.append('        <alwaysOn>true</alwaysOn>')
        xml_lines.append('        <updateRate>0.0</updateRate>')
        xml_lines.append('        <cameraName>rover/camera</cameraName>')
        xml_lines.append('        <imageTopicName>image_raw</imageTopicName>')
        xml_lines.append('        <cameraInfoTopicName>camera_info</cameraInfoTopicName>')
        xml_lines.append('        <frameName>VISION_tilt_cradle</frameName>')
        xml_lines.append('      </plugin>')
        xml_lines.append('    </sensor>')
        xml_lines.append('  </gazebo>')
        xml_lines.append('')

    # ================================================================
    # Handle any remaining parts not yet included
    # ================================================================
    xml_lines.append('  <!-- ============================================ -->')
    xml_lines.append('  <!-- REMAINING PARTS (fixed to base)              -->')
    xml_lines.append('  <!-- ============================================ -->')

    # Track which parts we've already used
    used_parts = set()
    used_parts.add('fixed')
    for leg_id in ['FL', 'FR', 'BL', 'BR']:
        prefix = f'leg{leg_id}'
        for suffix in ['input', 'upper', 'lower', 'shockbody', 'shockrod',
                       'carrier', 'steer', 'drive', 'wheel']:
            used_parts.add(f'{prefix}_{suffix}')
        used_parts.add(f'hexarm{leg_id}')
        used_parts.add(f'arm{leg_id}')
        used_parts.add(f'hex_follower_{leg_id}')
        used_parts.add(f'tip{leg_id}')
    for hr in ['hexrotL240', 'hexrotL350', 'hexrotR240', 'hexrotR350']:
        used_parts.add(hr)
    for side in ['L', 'R']:
        for seg in range(1, 7):
            used_parts.add(f'tel{side}{seg}')
        for rot_idx in range(5):
            for rot_size in ['145', '15']:
                used_parts.add(f'rot{side}{rot_idx}_{rot_size}')
        for angle in ['-240', '240']:
            for idx in range(1, 4):
                used_parts.add(f'guide{side}_{angle}_{idx}')
    used_parts.add('VISION_camera_yoke')
    used_parts.add('VISION_tilt_cradle')

    for part_name, part_info in parts.items():
        if part_name not in used_parts:
            mat = get_material(part_name)
            add_link(part_name, part_info, mat)
            add_joint(f'{part_name}_fixed_joint', 'fixed',
                      'base_link', part_name)
            print(f"  Note: Part '{part_name}' attached as fixed to base_link")

    xml_lines.append('</robot>')

    # Write URDF
    with open(urdf_path, 'w') as f:
        f.write('\n'.join(xml_lines))

    print(f"\nURDF written to: {urdf_path}")
    return urdf_path


# ============================================================
# Main execution
# ============================================================

def main():
    step_file = "/home/diya/Downloads/seee1.step"
    output_base = "/home/diya/rover_urdf_ws/src/rover_description"
    stl_dir = os.path.join(output_base, "meshes")
    urdf_path = os.path.join(output_base, "urdf", "rover.urdf")

    print("=" * 60)
    print("STEP to URDF Converter")
    print("Hexagonal Telescopic Chassis Rover")
    print("=" * 60)

    # Step 1: Parse STEP assembly for product names
    print("\n[1/4] Parsing STEP assembly structure...")
    products = parse_step_products(step_file)
    print(f"  Found {len(products)} products")
    for pid, pname in sorted(products.items()):
        print(f"    #{pid}: {pname}")

    # Step 2: Extract parts and export STLs
    print("\n[2/4] Extracting parts and exporting STL meshes...")
    try:
        parts = extract_parts_with_cadquery(step_file, stl_dir)
        print(f"  Successfully extracted {len(parts)} parts with CadQuery/OCP")
    except Exception as e:
        print(f"  CadQuery/OCP extraction failed: {e}")
        print("  Falling back to gmsh-based extraction...")
        parts = extract_parts_with_gmsh_full(step_file, stl_dir)

    # Step 3: Build URDF
    print("\n[3/4] Building URDF...")
    build_urdf(parts, stl_dir, urdf_path)

    # Step 4: Validate
    print("\n[4/4] Validation summary:")
    print(f"  Total parts extracted: {len(parts)}")
    total_mass = sum(p.get('mass', 0) for p in parts.values())
    print(f"  Total robot mass: {total_mass:.2f} kg")
    print(f"  URDF file: {urdf_path}")
    print(f"  Mesh directory: {stl_dir}")
    print(f"  Number of STL files: {len([f for f in os.listdir(stl_dir) if f.endswith('.stl')])}")

    return parts, urdf_path


def extract_parts_with_gmsh_full(step_file, output_dir):
    """Full gmsh-based fallback: load STEP, mesh each solid, export STL."""
    import gmsh

    os.makedirs(output_dir, exist_ok=True)

    gmsh.initialize()
    gmsh.option.setNumber("General.Verbosity", 2)

    print(f"  Loading STEP file with gmsh...")
    gmsh.open(step_file)

    volumes = gmsh.model.getEntities(3)
    print(f"  Found {len(volumes)} volumes")

    parts = {}

    for dim, tag in volumes:
        name = gmsh.model.getEntityName(dim, tag)
        if not name:
            name = f"solid_{tag}"
        clean = name.split('/')[-1] if '/' in name else name
        clean = re.sub(r'[^a-zA-Z0-9_]', '_', clean)
        if clean and clean[0].isdigit():
            clean = 'p_' + clean

        # Ensure unique
        base_clean = clean
        counter = 1
        while clean in parts:
            clean = f"{base_clean}_{counter}"
            counter += 1

        stl_path = os.path.join(output_dir, f"{clean}.stl")

        try:
            # Mesh this volume
            gmsh.model.mesh.generate(2)

            # Get mesh data for this entity
            node_tags, coords, _ = gmsh.model.mesh.getNodes(dim, tag)
            if len(node_tags) == 0:
                print(f"    SKIP (no nodes): {clean}")
                continue

            # Write individual STL using gmsh
            gmsh.write(stl_path)

            # Estimate mass from bounding box
            bbox = gmsh.model.getBoundingBox(dim, tag)
            xmin, ymin, zmin, xmax, ymax, zmax = bbox
            size_x = (xmax - xmin) * 0.001  # mm to m
            size_y = (ymax - ymin) * 0.001
            size_z = (zmax - zmin) * 0.001
            approx_vol = size_x * size_y * size_z * 0.3  # ~30% fill ratio
            density = assign_density(name)
            mass = max(approx_vol * density, 0.01)

            com = ((xmin + xmax) / 2 * 0.001,
                   (ymin + ymax) / 2 * 0.001,
                   (zmin + zmax) / 2 * 0.001)

            # Approximate inertia as cuboid
            ixx = mass / 12 * (size_y**2 + size_z**2)
            iyy = mass / 12 * (size_x**2 + size_z**2)
            izz = mass / 12 * (size_x**2 + size_y**2)

            parts[clean] = {
                'stl_path': stl_path,
                'original_name': name,
                'mass': mass,
                'com': com,
                'inertia': {
                    'ixx': max(ixx, 1e-6),
                    'iyy': max(iyy, 1e-6),
                    'izz': max(izz, 1e-6),
                    'ixy': 0, 'ixz': 0, 'iyz': 0
                },
                'bbox': {
                    'min': (xmin * 0.001, ymin * 0.001, zmin * 0.001),
                    'max': (xmax * 0.001, ymax * 0.001, zmax * 0.001),
                    'size': (size_x, size_y, size_z)
                },
                'mesh_path': f"package://rover_description/meshes/{clean}.stl"
            }
            print(f"    Exported: {clean} ({size_x*100:.1f}x{size_y*100:.1f}x{size_z*100:.1f} cm, {mass:.3f} kg)")

        except Exception as e:
            print(f"    ERROR exporting {clean}: {e}")

    gmsh.finalize()
    return parts


if __name__ == '__main__':
    parts, urdf = main()
