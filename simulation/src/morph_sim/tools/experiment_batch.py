"""Serialize experiment batches; cancellation forwards to the owned runner."""
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import time

class ExperimentBatch:
    def __enter__(self):
        self.lock=Path('log/experiment-batch.lock').open('a')
        try:fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:self.lock.close();raise SystemExit('Another experiment batch is running; stop its owning terminal first')
        self.cancelled=False;self.child=None;self.interrupted_pid=None;self.previous={}
        for sig in (signal.SIGINT,signal.SIGTERM):
            self.previous[sig]=signal.getsignal(sig);signal.signal(sig,self.cancel)
        return self
    def cancel(self,signum,frame):
        self.cancelled=True
        if self.child and self.child.poll() is None and self.interrupted_pid!=self.child.pid:
            self.interrupted_pid=self.child.pid
            try:os.kill(self.child.pid,signal.SIGINT)
            except ProcessLookupError:pass
    def run(self,command,env,output,timeout):
        if self.cancelled:raise KeyboardInterrupt
        self.child=subprocess.Popen(command,env=env,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
        start=time.monotonic()
        try:
            while self.child.poll() is None:
                if self.cancelled:self.cancel(signal.SIGINT,None)
                if time.monotonic()-start>timeout:self.cancel(signal.SIGTERM,None);raise TimeoutError('Experiment timeout; runner cleanup requested')
                time.sleep(.1)
            code=self.child.returncode
            if self.cancelled:raise KeyboardInterrupt
            return code
        finally:
            if self.child.poll() is None:
                self.cancel(signal.SIGINT,None)
                try:self.child.wait(timeout=40)
                except subprocess.TimeoutExpired:os.killpg(self.child.pid,signal.SIGKILL);self.child.wait()
    def __exit__(self,*args):
        for sig,handler in self.previous.items():signal.signal(sig,handler)
        self.lock.close()
