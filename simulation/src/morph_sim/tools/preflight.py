#!/usr/bin/env python3
"""Portable source checks; on Ubuntu also expands/converts and validates SDF."""
import ast,json,pathlib,shutil,sys,tempfile,xml.etree.ElementTree as ET
P=pathlib.Path(__file__).resolve().parents[1];sys.path.insert(0,str(P))
def main():
 import xacro
 checks={};errors=[]
 for f in list(P.rglob('*.py'))+list((P/'nodes').iterdir()):
  if '__pycache__' not in str(f) and f.is_file():
   try:ast.parse(f.read_text(encoding='utf-8'),filename=str(f))
   except Exception as e:errors.append(str(e))
 checks['python_syntax']=not errors
 for f in [P/'package.xml',P/'worlds/competition.sdf',P/'worlds/adaptive_arena.sdf']:
  try:ET.parse(f)
  except Exception as e:errors.append(str(e))
 doc=xacro.process_file(str(P/'urdf/morph.urdf.xacro'),mappings={'controllers_file':str(P/'config/controllers.yaml')});root=ET.fromstring(doc.toxml());checks['xacro_expansion']=True
 missing=[]
 for mesh in root.findall('.//mesh'):
  path=P/mesh.attrib['filename'].split('package://morph_sim/')[1]
  if not path.exists():missing.append(str(path))
 for world in [P/'worlds/competition.sdf',P/'worlds/adaptive_arena.sdf']:
  for elem in ET.parse(world).iter():
   if elem.text and elem.text.startswith('model://morph_sim/') and not (P/elem.text.split('model://morph_sim/')[1]).exists():missing.append(elem.text)
 checks['resource_paths']=not missing;errors+=missing
 checks['gz_sdf_validation']='NOT RUN: gz executable unavailable'
 if shutil.which('gz'):
  import subprocess
  from morph_sim.assets import prepare_robot
  with tempfile.TemporaryDirectory(prefix='morph_check_') as tmp:
   _,sdf=prepare_robot(P,tmp)
   for f in [P/'worlds/competition.sdf',P/'worlds/adaptive_arena.sdf',sdf]:
    result=subprocess.run(['gz','sdf','-k',str(f)],capture_output=True,text=True,timeout=60)
    if result.returncode:errors.append(result.stdout+result.stderr)
  checks['gz_sdf_validation']='PASSED' if not errors else 'FAILED'
 checks['errors']=errors;checks['passed_local_checks']=not errors
 print(json.dumps(checks,indent=2));return 1 if errors else 0
if __name__=='__main__':sys.exit(main())
