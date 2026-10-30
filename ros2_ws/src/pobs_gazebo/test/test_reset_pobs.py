"""Exercise reset ordering and buoyancy-only restoration with Gazebo messages."""

from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
from gz.msgs.serialized_map_pb2 import SerializedStepMap

spec = importlib.util.spec_from_file_location(
    'reset_pobs', Path(__file__).resolve().parents[3] / 'scripts/reset_pobs.py')
reset_pobs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reset_pobs)


class FakeGazebo(reset_pobs.GazeboReset):
    def __init__(self, fail_restore=False):
        self.snapshot = SerializedStepMap()
        self.snapshot.stats.sim_time.sec = 10
        entity = self.snapshot.state.entities[5]
        entity.id = 5
        for type_id in reset_pobs.BUOYANCY_TYPES | {123}:
            component = entity.components[type_id]
            component.type = type_id
            component.component = b'original'
        self.calls = []
        self.fail_restore = fail_restore

    def state(self):
        return deepcopy(self.snapshot)

    def request(self, service, message, response_type):
        self.calls.append((service, deepcopy(message)))
        if service == 'control':
            self.snapshot.stats.paused = message.pause
            if message.reset.all:
                self.snapshot.stats.sim_time.Clear()
                for type_id in reset_pobs.BUOYANCY_TYPES:
                    del self.snapshot.state.entities[5].components[type_id]
        elif service == 'control/state':
            if self.fail_restore:
                raise RuntimeError('restore failed')
            assert self.snapshot.stats.paused
            for entity in message.state.entities:
                for component in entity.components:
                    self.snapshot.state.entities[entity.id].components[
                        component.type].CopyFrom(component)


@pytest.mark.parametrize('paused', [True, False])
def test_reset_restores_only_buoyancy_before_resuming(paused):
    gazebo = FakeGazebo()
    original = reset_pobs.buoyancy_components(gazebo.state())
    assert gazebo.reset(paused=paused) == 1
    assert gazebo.snapshot.stats.paused == paused
    assert reset_pobs.buoyancy_components(gazebo.state()) == original
    restore = gazebo.calls[2][1]
    assert {c.type for c in restore.state.entities[0].components} == reset_pobs.BUOYANCY_TYPES
    assert len(gazebo.calls) == (3 if paused else 4)
    # A second reset while paused at time zero also waits for reset application.
    if paused:
        assert gazebo.reset(paused=True) == 1


def test_missing_metadata_does_not_reset_the_world():
    gazebo = FakeGazebo()
    type_id = next(iter(reset_pobs.BUOYANCY_TYPES))
    del gazebo.snapshot.state.entities[5].components[type_id]
    with pytest.raises(RuntimeError, match='missing or incomplete'):
        gazebo.reset()
    assert gazebo.calls == []
    assert gazebo.snapshot.stats.sim_time.sec == 10


def test_failed_restore_leaves_world_paused():
    gazebo = FakeGazebo(fail_restore=True)
    with pytest.raises(RuntimeError, match='restore failed'):
        gazebo.reset()
    assert gazebo.snapshot.stats.paused
    assert len(gazebo.calls) == 3

