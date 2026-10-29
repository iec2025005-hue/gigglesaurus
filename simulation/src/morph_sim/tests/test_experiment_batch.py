"""Batch locking must prevent overlapping simulation experiments."""
from pathlib import Path
import subprocess,sys
import pytest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from experiment_batch import ExperimentBatch


def test_second_batch_cannot_acquire_held_experiment_lock(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path);(tmp_path/'log').mkdir()
    with ExperimentBatch():
        with pytest.raises(SystemExit,match='Another experiment batch'):
            with ExperimentBatch():pass
    with ExperimentBatch():pass


def test_cancelled_batch_does_not_start_a_new_process(tmp_path,monkeypatch):
    monkeypatch.chdir(tmp_path);(tmp_path/'log').mkdir()
    with ExperimentBatch() as batch:
        batch.cancelled=True
        with pytest.raises(KeyboardInterrupt):batch.run([sys.executable,'-c','raise RuntimeError("must not run")'],None,subprocess.DEVNULL,1)


def test_interruption_allows_owned_runner_to_finish_cleanup(tmp_path,monkeypatch):
    import threading,time
    monkeypatch.chdir(tmp_path);(tmp_path/'log').mkdir()
    script='''import signal,time
from pathlib import Path
stop=False
def handle(sig,frame):
 global stop
 stop=True
signal.signal(signal.SIGINT,handle)
Path('ready').touch()
while not stop:time.sleep(.01)
time.sleep(.3)
Path('cleaned').touch()
'''
    with ExperimentBatch() as batch:
        def interrupt():
            deadline=time.monotonic()+5
            while not (tmp_path/'ready').exists() and time.monotonic()<deadline:time.sleep(.01)
            batch.cancel(signal.SIGINT,None)
        import signal
        thread=threading.Thread(target=interrupt);thread.start()
        try:
            with pytest.raises(KeyboardInterrupt):batch.run([sys.executable,'-c',script],None,subprocess.DEVNULL,5)
        finally:thread.join(timeout=6)
    assert (tmp_path/'cleaned').exists()
