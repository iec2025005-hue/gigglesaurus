#!/usr/bin/env python3
"""Exercise normal startup, duplicate launch rejection and bounded shutdown."""
import fcntl,json,os,signal,subprocess,time
from pathlib import Path
ws=Path.cwd();log=ws/'log/shutdown_check.log';handle=log.open('w')
process=subprocess.Popen(['bash','run_gazebo.sh','headless:=true','rviz:=false','controls:=none','teleop:=false'],stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
try:
    deadline=time.monotonic()+120
    while time.monotonic()<deadline:
        if process.poll() is not None:raise RuntimeError('Launch exited early; inspect '+str(log))
        if 'Traceback' in log.read_text():raise RuntimeError('Node failed during startup; inspect '+str(log))
        if 'ASSUMED design profile' in log.read_text():break
        time.sleep(.2)
    else:raise RuntimeError('Controllers did not become ready')
    duplicate=subprocess.run(['bash','run_gazebo.sh','headless:=true','rviz:=false','controls:=none','teleop:=false'],capture_output=True,text=True,timeout=15)
    guard=duplicate.returncode==75 and 'already running' in duplicate.stdout
    start=time.monotonic();os.kill(process.pid,signal.SIGINT);code=process.wait(timeout=35);elapsed=time.monotonic()-start
    with (ws/'log/simulation-launch.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    text=log.read_text();report=dict(duplicate_guard=guard,exit_code=code,shutdown_wall_s=elapsed,lock_released=True,tracebacks='Traceback' in text,forced_termination='escalating to' in text,native_crash='exit code -11' in text)
    report['passed']=guard and code==0 and not report['tracebacks'] and not report['forced_termination'] and not report['native_crash']
    (ws/'validation/shutdown_check.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)
finally:
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGTERM)
        try:process.wait(timeout=30)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    handle.close()
