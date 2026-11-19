"""Show the expected ESP32 LED state from the live Pi simulation."""

import argparse
import shlex
import subprocess


MONITOR = '''
import math
import time

import rclpy
from std_msgs.msg import Float64

rclpy.init()
node = rclpy.create_node('watch_thruster_leds')
pins = (15, 16, 18, 21)
samples = [None] * len(pins)

def receive(message, index):
    value = message.data
    samples[index] = (abs(value), time.monotonic()) if math.isfinite(value) else None

for index in range(len(pins)):
    node.create_subscription(
        Float64, f'/model/pobs/joint/thruster_{index}_joint/cmd_thrust',
        lambda message, i=index: receive(message, i), 1,
    )

previous = None

def report():
    global previous
    now = time.monotonic()
    if any(sample is None or now - sample[1] > 0.5 for sample in samples):
        state = 'All OFF (waiting for four fresh thrust values)'
    else:
        maximum = max(sample[0] for sample in samples)
        winners = [f'LED {i + 1} (GPIO {pin}, thruster {i})'
                   for i, pin in enumerate(pins)
                   if maximum > 0.01 and math.isclose(samples[i][0], maximum,
                                                       rel_tol=1e-6, abs_tol=1e-9)]
        state = 'ON: ' + ', '.join(winners) if winners else 'All OFF (below or at deadband)'
    if state != previous:
        print(state, flush=True)
        previous = state

node.create_timer(0.1, report)
print('Watching thrusters 0-3. Press Ctrl+C to stop.', flush=True)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='pobs-pi-ethernet')
    parser.add_argument('--container', default='pobs-pi')
    args = parser.parse_args()
    shell = 'source /opt/ros/lyrical/setup.bash && exec python3 -u -c ' + shlex.quote(MONITOR)
    remote = shlex.join(['sudo', 'docker', 'exec', '-it', '-u', 'ros',
                         args.container, 'bash', '-c', shell])
    try:
        return subprocess.call(['ssh', '-t', '-o', 'ConnectTimeout=8', '--', args.host, remote])
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
