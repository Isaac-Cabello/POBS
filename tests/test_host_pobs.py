"""Exercise host command routing without starting Docker or a simulator."""

import json
import os
from pathlib import Path
import subprocess

import pytest


LAUNCHER = Path(__file__).parents[1] / 'scripts' / 'pobs'


@pytest.fixture
def host(tmp_path):
    docker = tmp_path / 'docker'
    docker.write_text('''#!/usr/bin/env python3
import json
import os
import sys
args = sys.argv[1:]
with open(os.environ['DOCKER_LOG'], 'a') as stream:
    stream.write(json.dumps(args) + '\\n')
if args[0] == 'inspect':
    print(os.environ.get('SIM_RUNNING', 'true'))
if args[0] == 'exec' and 'pgrep' in args:
    found = (os.environ.get('EXISTING_BRIDGE') == 'true' and 'pobs/imu' in args[-1])
    found |= (os.environ.get('EXISTING_ESTIMATOR') == 'true'
              and 'pobs_state_estimator/state_estimator' in args[-1])
    sys.exit(0 if found else 1)
''')
    docker.chmod(0o755)
    log = tmp_path / 'docker.jsonl'
    env = dict(os.environ, PATH=f'{tmp_path}:{os.environ["PATH"]}', DOCKER_LOG=str(log),
               POBS_REPO=str(LAUNCHER.parent.parent))

    def run(*args, existing_bridge=False, existing_estimator=False, sim_running=True):
        env['EXISTING_BRIDGE'] = 'true' if existing_bridge else 'false'
        env['EXISTING_ESTIMATOR'] = 'true' if existing_estimator else 'false'
        env['SIM_RUNNING'] = 'true' if sim_running else 'false'
        subprocess.run(['bash', str(LAUNCHER), *args], env=env, check=True,
                       capture_output=True, text=True)
        return [json.loads(line) for line in log.read_text().splitlines()]
    return run


def test_sensors_routes_to_source_command_with_simulation_default(host):
    commands = host('sensors')
    assert commands[-1][-3:] == [
        '/home/ros/ros2_ws/scripts/pobs', 'sensors', 'source:=simulation']
    assert commands[-1][0] == 'exec'


def test_sensors_skips_existing_bridge_and_preserves_explicit_source(host):
    command = host('sensors', 'source:=hardware', existing_bridge=True)[-1]
    assert command[-3:] == ['sensors', 'source:=hardware', 'existing_sensors:=false']


def test_rviz_tags_and_arguments_are_forwarded_as_separate_arguments(host):
    command = host('rvis', 'point_cloud_mapping', 'world_frame:=lake',
                   'model_path:=/a directory/model.sdf')[-1]
    assert command[-4:] == ['rviz', 'point_cloud_mapping', 'world_frame:=lake',
                            'model_path:=/a directory/model.sdf']
    assert 'exec "$@"' in command[command.index('-lc') + 1]


def test_legacy_sonar_command_reads_downward_range_topic(host):
    command = host('sonar', '--watch')[-1]
    assert '/pobs/sonar/sonar_depth/range' in command
    assert '--once' not in command


def test_compare_routes_to_restored_state_view_and_forwards_overrides(host):
    command = host('compare', 'start_estimator:=false')[-1]
    assert command[-4:] == ['/home/ros/ros2_ws/scripts/pobs', 'rviz',
                            'state_estimation', 'start_estimator:=false']


def test_compare_reuses_the_existing_estimator(host):
    command = host('compare', existing_estimator=True)[-1]
    assert command[-1] == 'start_estimator:=false'


@pytest.mark.parametrize('options, flags', [((), 'gz sim  -r'),
                                           (('--headless',), 'gz sim -s --headless-rendering -r')])
def test_sim_defaults_to_gui_with_explicit_headless_option(host, options, flags):
    commands = host('sim', *options, sim_running=False)
    launch = next(command for command in commands if command[0] == 'compose')
    assert flags in launch[-1]


@pytest.mark.parametrize('options', [(), ('--paused',)])
def test_soft_reset_routes_to_existing_helper_without_stopping_container(host, options):
    commands = host('reset', *options)
    assert (LAUNCHER.parent.parent / 'ros2_ws/scripts/reset_pobs.py').is_file()
    command = commands[-1]
    assert '/home/ros/ros2_ws/scripts/reset_pobs.py' in command[-1]
    assert ('--paused' in command[-1]) == bool(options)
    assert not any(item[0] in ('stop', 'rm') for item in commands)
