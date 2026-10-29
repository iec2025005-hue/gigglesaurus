#!/usr/bin/env python3
"""Launch one benchmark with isolated logs and guaranteed supervisor shutdown."""
import argparse,json,os,shutil,signal,subprocess,time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('name');parser.add_argument('launch_args',nargs='*');args=parser.parse_args()
    ws=Path.cwd()
    cancelled=ws/'log/benchmark_cancel_prefixes.json'
    if cancelled.exists() and any(args.name.startswith(prefix) for prefix in json.loads(cancelled.read_text())):
        raise SystemExit('Benchmark batch cancelled before launch: '+args.name)
    directory=ws/'validation/performance'/args.name
    if directory.exists():raise RuntimeError('Refusing to overwrite previous experiment evidence: '+str(directory))
    directory.mkdir(parents=True)
    previous=set((ws/'log').glob('run_*'));log=(directory/'launch.log').open('w')
    process=subprocess.Popen(['bash','run_gazebo.sh','controls:=none','teleop:=false']+args.launch_args,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ,MORPH_SUPERVISOR_DIAGNOSTIC_FILE=str(directory/'shutdown.json')))
    try:
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Launch exited; inspect '+str(directory/'launch.log'))
            text=(directory/'launch.log').read_text()
            if 'Traceback' in text:raise RuntimeError('Node startup failed')
            if 'ASSUMED design profile' in text:break
            time.sleep(.2)
        else:raise RuntimeError('Timed out waiting for controllers')
        runs=set((ws/'log').glob('run_*'))-previous
        if len(runs)!=1:raise RuntimeError('Run manifest identification is ambiguous')
        run=next(iter(runs));shutil.copyfile(run/'resolved_manifest.json',directory/'resolved_manifest.json')
        generated=directory/'generated';generated.mkdir(exist_ok=True)
        for file in run.iterdir():
            if file.is_file():shutil.copyfile(file,generated/file.name)
        env=dict(os.environ,MORPH_BENCH_OUTPUT=str(directory/'raw.json'),ROS_LOG_DIR=str(ws/'log/benchmark_ros'),MORPH_RUN_MANIFEST_FILE=str(directory/'resolved_manifest.json'))
        Path(env['ROS_LOG_DIR']).mkdir(parents=True,exist_ok=True)
        with (directory/'benchmark.log').open('w') as output:
            protocol=subprocess.run(['python3',os.environ.get('MORPH_PROTOCOL_SCRIPT','src/morph_sim/tools/benchmark_realtime.py')],env=env,stdout=output,stderr=subprocess.STDOUT,check=False,timeout=max(900,float(os.environ.get('MORPH_LOADED_SOAK_SECONDS','0'))*5+120,float(os.environ.get('MORPH_LOADED_SOAK_WALL_SECONDS','0'))+300))
        if not (directory/'raw.json').exists():
            failure={'passed':False,'protocol_exit_code':protocol.returncode,'error':'Protocol produced no raw data; inspect benchmark.log','name':args.name,'launch_args':args.launch_args}
            (directory/'summary.json').write_text(json.dumps(failure,indent=2));raise SystemExit(protocol.returncode or 1)
        data=json.loads((directory/'raw.json').read_text())
        if isinstance(data,dict):
            report={k:v for k,v in data.items() if k not in ('trajectory','manager_status','physics_status','samples','native_samples','sensor_samples','trials','loaded_soak_samples')};report['name']=args.name;report['launch_args']=args.launch_args
            (directory/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));
            if protocol.returncode:raise SystemExit(protocol.returncode)
            return
        sim=sum(x['duration_sim_s'] for x in data);wall=sum(x['duration_sim_s']/x['rtf'] for x in data)
        report=dict(name=args.name,launch_args=args.launch_args,sim_s=sim,wall_s=wall,mean_rtf=sim/wall,forward_response_wall_ms=data[1]['response']['wall_s']*1000,scope='One short application-level benchmark; not p95, moving emergency stop, or loaded soak')
        (directory/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    finally:
        if process.poll() is None:
            os.kill(process.pid,signal.SIGINT)
            try:process.wait(timeout=35)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        log.close()
        text=(directory/'launch.log').read_text()
        shutdown=json.loads((directory/'shutdown.json').read_text()) if (directory/'shutdown.json').exists() else {}
        (directory/'cleanup.json').write_text(json.dumps({'launch_exit_code':process.returncode,'traceback':'Traceback' in text,'native_crash':'exit code -11' in text,'forced_termination':'escalating to' in text or shutdown.get('forced',False),'shutdown_diagnostics':shutdown,'clean_exit':process.returncode==0 and not ('Traceback' in text or 'exit code -11' in text or 'escalating to' in text or shutdown.get('forced',False)),'native_limit_warning_count':text.count('Command of at least one joint is out of limits')},indent=2))


if __name__=='__main__':main()
