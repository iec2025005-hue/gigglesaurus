#!/usr/bin/env python3
"""Own the launch lock and forward shutdown to a dedicated process group."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import shutil
import time


def owned_bridge_pids(root):
    parents={};bridges=[]
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():continue
        try:
            parents[int(path.name)]=int((path/'stat').read_text().rpartition(')')[2].split()[1])
            if any(Path(arg.decode()).name in ('parameter_bridge','lifecycle_bridge') for arg in (path/'cmdline').read_bytes().split(b'\0') if arg):bridges.append(int(path.name))
        except (OSError,ValueError,UnicodeDecodeError):pass
    owned=[]
    for bridge in bridges:
        ancestor=bridge;seen=set()
        while ancestor in parents and ancestor not in seen:
            if ancestor==root:owned.append(bridge);break
            seen.add(ancestor);ancestor=parents[ancestor]
    return owned


def main():
    if len(sys.argv)<3:
        raise SystemExit('Usage: launch_supervisor.py LOCK COMMAND [ARGS...]')
    lock=Path(sys.argv[1]);lock.parent.mkdir(parents=True,exist_ok=True)
    with lock.open('a') as owner:
        try:fcntl.flock(owner,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('A MORPH simulation from this workspace is already running. Close its launch terminal with Ctrl+C before starting another.',flush=True)
            return 75
        child=subprocess.Popen(sys.argv[2:],start_new_session=True,close_fds=True)
        diagnostic={'launch_pid':child.pid,'supervisor_pid':os.getpid(),'phases':[],'forced':False,'requested':False}
        diagnostic_path=Path(os.environ.get('MORPH_SUPERVISOR_DIAGNOSTIC_FILE',str(lock.parent/('shutdown_'+str(os.getpid())+'.json'))))
        def record(phase,**values):diagnostic['phases'].append(dict(phase=phase,wall_ns=time.monotonic_ns(),**values))
        gui_enabled=not any(arg in ('headless:=true','gazebo_gui:=false') for arg in sys.argv)
        interrupted=False
        stop_sent=False
        def forward(signum,frame):
            nonlocal interrupted
            interrupted=True
            diagnostic["requested"]=True

        signal.signal(signal.SIGINT,forward);signal.signal(signal.SIGTERM,forward)
        try:
            while True:
                try:returncode=child.wait(timeout=.2);break
                except subprocess.TimeoutExpired:
                    if interrupted:
                        if not stop_sent:
                            stop_sent=True
                            stopped=False
                            record('shutdown_requested')
                            if os.environ.get('MORPH_SUPERVISED_GZ')=='1' and shutil.which('gz'):
                                try:
                                    world='morph_arena' if 'arena:=competition' in sys.argv else 'leo_marsyard'
                                    subprocess.run(['gz','service','-s','/world/'+world+'/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: true'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=3)
                                    record('pause_service_complete')
                                    time.sleep(.15)  # Drain sensor deliveries before transport teardown.
                                    # Stop our receiver while its Gazebo provider is still alive.
                                    for bridge in owned_bridge_pids(child.pid):
                                        try:os.kill(bridge,signal.SIGINT)
                                        except ProcessLookupError:continue
                                        deadline=time.monotonic()+2
                                        while Path('/proc',str(bridge)).exists() and time.monotonic()<deadline:time.sleep(.02)
                                    if gui_enabled:
                                        gui_result=subprocess.run(['gz','service','-s','/morph/gui/shutdown','--reqtype','gz.msgs.Boolean','--reptype','gz.msgs.Boolean','--timeout','2000','--req','data: true'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=3)
                                        record('gui_quit_service',acknowledged=gui_result.returncode==0 and 'true' in gui_result.stdout)
                                    result=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true'],stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,timeout=5)
                                    stopped=result.returncode==0 and 'true' in result.stdout
                                    record('server_stop_service',acknowledged=stopped)
                                except (subprocess.TimeoutExpired,OSError):pass
                            if not stopped:
                                try:os.kill(child.pid,signal.SIGINT)
                                except ProcessLookupError:pass
                        try:returncode=child.wait(timeout=20);break
                        except subprocess.TimeoutExpired:
                            diagnostic['forced']=True;record('escalating_to_sigterm');print('MORPH: escalating to SIGTERM after graceful shutdown timeout',flush=True)
                            os.killpg(child.pid,signal.SIGTERM)
                            try:returncode=child.wait(timeout=5);break
                            except subprocess.TimeoutExpired:
                                diagnostic['forced']=True;record('escalating_to_sigkill');print('MORPH: escalating to SIGKILL after SIGTERM timeout',flush=True)
                                os.killpg(child.pid,signal.SIGKILL);returncode=child.wait();break
            return 0 if interrupted and returncode in (0,-signal.SIGINT,130) else returncode
        finally:
            diagnostic['launch_return_code']=child.poll();record('supervisor_cleanup')
            diagnostic_path.parent.mkdir(parents=True,exist_ok=True);diagnostic_path.write_text(json.dumps(diagnostic,indent=2))
            # A launch failure must not leave orphaned descendants holding resources.
            try:os.killpg(child.pid,signal.SIGTERM)
            except ProcessLookupError:pass


if __name__=='__main__':raise SystemExit(main())
