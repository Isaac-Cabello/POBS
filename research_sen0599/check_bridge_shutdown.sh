#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/lyrical/setup.bash
python3 - <<'PY'
import signal
import subprocess
import time

process = subprocess.Popen(['bash', '/scripts/bridge_pobs.sh'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, start_new_session=True)
try:
    time.sleep(3)
    assert process.poll() is None
    process.send_signal(signal.SIGINT)
    output, _ = process.communicate(timeout=10)
    print(output)
    assert process.returncode == 130, process.returncode
    assert 'Traceback' not in output and 'Segmentation fault' not in output
    remaining = subprocess.check_output(['ps', '-eo', 'args'], text=True)
    assert '/lib/ros_gz_bridge/parameter_bridge' not in remaining, remaining
    assert 'python3 /scripts/pobs_depth_sensor.py' not in remaining, remaining
    print('PASS: bridge launcher stops both isolated children without orphans')
finally:
    if process.poll() is None:
        process.kill()
        process.wait()
PY
