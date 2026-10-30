"""Smoke test visualization command tags and clean headless launch shutdown."""
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['POBS_WORKSPACE'] = '/tmp/mapping_ws'
command = ['bash', '/repo/ros2_ws/scripts/pobs']
subprocess.run(['bash', '-n', command[1]], check=True)
invalid = subprocess.run(command + ['rviz', 'invalid_tag'], capture_output=True, text=True)
assert invalid.returncode == 2 and 'Unknown RViz tag' in invalid.stderr
log_path = Path('/tmp/mapping_cli.log')
with log_path.open('w') as log:
    process = subprocess.Popen(
        command + ['rviz', 'point_cloud_mapping', 'rviz:=false'],
        start_new_session=True, stdout=log, stderr=log)
    try:
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            assert process.poll() is None, log_path.read_text()
            if 'Loaded four SDF sensor transforms' in log_path.read_text():
                break
            time.sleep(0.1)
        output = log_path.read_text()
        assert 'Loaded four SDF sensor transforms' in output, output
        assert 'point_cloud_mapper' in output and 'clock_bridge' in output, output
        assert 'rviz2-' not in output and 'state_estimator-' not in output, output
    finally:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=12)
assert process.returncode in (0, 130), process.returncode
output = log_path.read_text()
assert 'process has died' not in output and 'Traceback' not in output, output
print('PASS: pobs rviz tag selects mapping-only headless launch, '
      'starts mapper and clock bridge, rejects unknown tags, exits cleanly')
