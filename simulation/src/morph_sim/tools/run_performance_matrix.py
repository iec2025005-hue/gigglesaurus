#!/usr/bin/env python3
"""Five repeats per preset, preserving per-run manifests and failed results."""
import argparse,json,os,subprocess
from pathlib import Path
from experiment_batch import ExperimentBatch
parser=argparse.ArgumentParser();parser.add_argument('--prefix',required=True);parser.add_argument('--repeats',type=int,default=5);parser.add_argument('--position-backend',choices=('forward','trajectory'),default='forward');args=parser.parse_args()
if not args.prefix.replace('_','').replace('-','').isalnum():raise ValueError('Invalid output prefix')
ws=Path.cwd();records=[]
presets={
 'headless':['headless:=true','rviz:=false'],
 'visible':['headless:=false','rviz:=false'],
 'hq_camera':['headless:=false','rviz:=false','camera_enabled:=true','camera_width:=640','camera_rate:=30','render_shadows:=true'],
 'visible_lp2':['headless:=false','rviz:=false'],
}
with ExperimentBatch() as batch:
 for repeat in range(args.repeats):
  for preset,launch in presets.items():
   name=f'{args.prefix}_{preset}_{repeat+1}';directory=ws/'validation/performance'/name
   if directory.exists():raise RuntimeError('Refusing to overwrite evidence: '+str(directory))
   env=dict(os.environ,MORPH_PROTOCOL_SCRIPT='src/morph_sim/tools/benchmark_response_trials.py',MORPH_RESPONSE_TRIALS='20')
   # Compare identical presets except the explicitly declared render worker count.
   env.pop('LP_NUM_THREADS',None)
   if preset=='visible_lp2':env['LP_NUM_THREADS']='2'
   with (ws/'log'/(name+'.log')).open('w') as log:
    try:code=batch.run(['python3','src/morph_sim/tools/run_benchmark_case.py',name]+launch+['position_backend:='+args.position_backend],env=env,output=log,timeout=1000)
    except TimeoutError:raise RuntimeError('Owned runner timeout: inspect descendants and lock before continuing')
   report=json.loads((directory/'summary.json').read_text()) if (directory/'summary.json').exists() else {'passed':False,'error':'Missing summary'}
   cleanup=json.loads((directory/'cleanup.json').read_text()) if (directory/'cleanup.json').exists() else None
   if code!=0 and not (directory/'summary.json').exists():raise SystemExit('Batch stopped after launch setup failure; inspect '+str(directory))
   records.append(dict(name=name,preset=preset,repeat=repeat+1,process_exit_code=code,summary=report,cleanup=cleanup))
   output={'scope':'Interleaved repeated application benchmarks; inspect manifests for rendering backend; actual GUI frame time remains separately measured','repeats_requested':args.repeats,'records':records,'complete':len(records)==4*args.repeats,'passed':len(records)==4*args.repeats and all(r['summary'].get('passed') and r['process_exit_code']==0 and r['cleanup'] and r['cleanup'].get('clean_exit',False) and r['cleanup'].get('native_limit_warning_count',0)==0 for r in records)}
   (ws/'validation'/f'{args.prefix}_matrix.json').write_text(json.dumps(output,indent=2))
   print(json.dumps(dict(name=name,passed=report.get('passed'),mean_rtf=report.get('mean_rtf'),response_p95_ms=report.get('body_response_wall_p95_ms'))),flush=True)
