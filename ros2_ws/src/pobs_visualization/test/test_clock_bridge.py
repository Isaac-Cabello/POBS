"""Exercise child ownership and repeated shutdown signals with a real process."""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from pobs_visualization.clock_bridge import run_managed
import pytest


def test_missing_child_returns_failure():
    assert run_managed(['/does/not/exist/clock-bridge']) == 1


def test_child_exit_status_propagates():
    assert run_managed([sys.executable, '-c', 'raise SystemExit(3)']) == 3


@pytest.mark.skipif(os.name != 'posix', reason='POSIX process group signals')
def test_repeated_group_sigint_reaps_isolated_child(tmp_path):
    pid_path = tmp_path / 'child.pid'
    code = (
        'import os,time; from pathlib import Path; '
        f'Path({str(pid_path)!r}).write_text(str(os.getpid())); time.sleep(60)'
    )
    wrapper_code = (
        'from pobs_visualization.clock_bridge import run_managed; '
        f'raise SystemExit(run_managed({[sys.executable, "-c", code]!r}))'
    )
    wrapper = subprocess.Popen(
        [sys.executable, '-c', wrapper_code], start_new_session=True)
    child_pid = None
    try:
        deadline = time.monotonic() + 5
        while not pid_path.exists() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert pid_path.exists(), 'Child did not start'
        child_pid = int(Path(pid_path).read_text())
        os.killpg(wrapper.pid, signal.SIGINT)
        os.killpg(wrapper.pid, signal.SIGINT)
        assert wrapper.wait(timeout=7) == 0
        with pytest.raises(ProcessLookupError):
            os.kill(child_pid, 0)
    finally:
        if wrapper.poll() is None:
            wrapper.kill()
            wrapper.wait()
        if child_pid is not None:
            try:
                os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
