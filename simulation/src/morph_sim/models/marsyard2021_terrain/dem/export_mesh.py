"""Export the upstream DEM without altering its elevation samples.

Using the same triangles for Gazebo visual and collision avoids different
heightfield sampling in the renderer and physics backend. Units are metres.
Run this file to regenerate meshes/terrain.obj from the upstream TIFF.
"""
from pathlib import Path
import numpy as np
from PIL import Image
import xml.etree.ElementTree as ET


def main():
    model = Path(__file__).resolve().parents[1]
    height = np.asarray(Image.open(model / 'dem/marsyard_terrain_hm.tif'), dtype=float)
    rows, cols = height.shape
    dx, dy = 47.0 / (cols - 1), 47.0 / (rows - 1)
    dz_row, dz_col = np.gradient(height, dy, dx)
    normals = np.stack((-dz_col, dz_row, np.ones_like(height)), axis=-1)
    normals /= np.linalg.norm(normals, axis=-1)[..., None]
    tree = ET.parse(model / 'model.sdf')
    link = tree.find("./model/link[@name='marsyard_terrain']")
    for element in list(link):
        link.remove(element)
    tile_count = 0
    for start_row in range(0, rows-1, 64):
        for start_col in range(0, cols-1, 64):
            stop_row, stop_col = min(start_row+64, rows-1), min(start_col+64, cols-1)
            tile_rows, tile_cols = stop_row-start_row+1, stop_col-start_col+1
            name = f'terrain_{start_row:03d}_{start_col:03d}'
            with (model / 'meshes' / (name+'.obj')).open('w') as out:
                out.write('# Leo Rover ERC2021 DEM tile, metres\nmtllib terrain.mtl\no '+name+'\n')
                for row in range(start_row, stop_row+1):
                    for col in range(start_col, stop_col+1):
                        out.write(f'v {-23.5 + col*dx:.8f} {23.5-row*dy:.8f} {height[row,col]:.8f}\n')
                for row in range(start_row, stop_row+1):
                    for col in range(start_col, stop_col+1):
                        out.write(f'vt {col/(cols-1):.8f} {1-row/(rows-1):.8f}\n')
                for normal in normals[start_row:stop_row+1, start_col:stop_col+1].reshape(-1, 3):
                    out.write('vn %.8f %.8f %.8f\n' % tuple(normal))
                out.write('usemtl Terrain\ns 1\n')
                for row in range(tile_rows-1):
                    for col in range(tile_cols-1):
                        a = row*tile_cols+col+1
                        for face in ((a,a+tile_cols,a+1),(a+1,a+tile_cols,a+tile_cols+1)):
                            out.write('f ' + ' '.join(f'{i}/{i}/{i}' for i in face) + '\n')
            for kind in ('collision','visual'):
                element = ET.SubElement(link, kind, name=name+'_'+kind)
                geometry = ET.SubElement(element, 'geometry')
                mesh = ET.SubElement(geometry, 'mesh')
                ET.SubElement(mesh,'uri').text = 'model://marsyard2021_terrain/meshes/'+name+'.obj'
                if kind == 'collision':
                    ode = ET.SubElement(ET.SubElement(ET.SubElement(element,'surface'),'friction'),'ode')
                    for key in ('mu','mu2'):
                        ET.SubElement(ode,key).text = '0.85'
                else:
                    material = ET.SubElement(element,'material')
                    ET.SubElement(material,'ambient').text = '1 1 1 1'
                    ET.SubElement(material,'diffuse').text = '1 1 1 1'
                    ET.SubElement(material,'specular').text = '0 0 0 1'
                    metal = ET.SubElement(ET.SubElement(material,'pbr'),'metal')
                    ET.SubElement(metal,'albedo_map').text = 'model://marsyard2021_terrain/dem/marsyard_terrain_texture.png'
                    ET.SubElement(metal,'roughness').text = '0.95'
                    ET.SubElement(metal,'metalness').text = '0'
            tile_count += 1
    (model / 'meshes/terrain.mtl').write_text(
        'newmtl Terrain\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nNs 1\nd 1\n'
        'illum 1\nmap_Kd ../dem/marsyard_terrain_texture.png\n')
    ET.indent(tree, space='  ')
    tree.write(model/'model.sdf', encoding='utf-8', xml_declaration=True)
    (model/'meshes/terrain.obj').unlink(missing_ok=True)
    print(f'{tile_count} tiles, {2*(rows-1)*(cols-1)} triangles; identical visual/collision meshes')


if __name__ == '__main__':
    main()
