#!/usr/bin/env python3
"""Sequential owned launches: six faults on high/low friction, retaining failures."""
from pathlib import Path
from experiment_batch import ExperimentBatch
import argparse,json,os,subprocess
parser=argparse.ArgumentParser();parser.add_argument("--prefix",required=True);parser.add_argument("--position-backend",choices=("forward","trajectory"),default="forward");parser.add_argument("--low-speed",type=float,default=.25);parser.add_argument("--low-mode",default="");args=parser.parse_args()
if not args.prefix.replace("_", "").replace("-", "").isalnum():raise ValueError("Invalid evidence prefix")
ws=Path.cwd();results=[]
with ExperimentBatch() as batch:
 for surface,arguments in [('high',['headless:=true','rviz:=false']),('low',['headless:=true','rviz:=false','sector:=snow'])]:
  for case in ('estop','release','stale_command','stale_state','manager_crash','actuator_crash'):
   name=args.prefix+'_'+surface+'_'+case;env=dict(os.environ,MORPH_PROTOCOL_SCRIPT='src/morph_sim/tools/check_moving_stop.py',MORPH_STOP_CASE=case)
   if surface=='low':env.update(MORPH_STOP_SPEED=str(args.low_speed),MORPH_STOP_MODE=args.low_mode)
   directory=ws/'validation/performance'/name
   if directory.exists():raise RuntimeError('Refusing to overwrite previous evidence: '+str(directory))
   with (ws/'log'/(name+'.log')).open('w') as log:
    code=batch.run(['python3','src/morph_sim/tools/run_benchmark_case.py',name]+arguments+['position_backend:='+args.position_backend],env=env,output=log,timeout=160)
   summary=directory/'summary.json'
   report=json.loads(summary.read_text()) if summary.exists() else {'case':case,'passed':False,'error':'No summary; inspect '+str(directory/'benchmark.log')}
   cleanup=directory/'cleanup.json';report['cleanup']=json.loads(cleanup.read_text()) if cleanup.exists() else None
   report['surface_case']=surface;report['process_exit_code']=code;results.append(report)
   print(json.dumps({k:v for k,v in report.items() if k in ('case','surface_case','stop_wall_s','stop_distance_path_m','passed','cleanup','error')}),flush=True)
   if code!=0 and not summary.exists():raise SystemExit('Batch stopped after launch/protocol setup failure; inspect '+str(directory))
   Path('validation/'+args.prefix+'_matrix.json').write_text(json.dumps({'scope':'Six injected failure modes on two main-arena surfaces; all design parameters assumed; release is command-level, not GUI key/focus test','cases':results,'passed':len(results)==12 and all(r.get('passed') and r['process_exit_code']==0 and r.get('cleanup',{}).get('clean_exit',False) and r['cleanup'].get('native_limit_warning_count',0)==0 for r in results)},indent=2))
