#!/usr/bin/env python3
"""Reset Gazebo in place, retaining link metadata needed by buoyancy."""

import argparse
from copy import deepcopy
import sys
import time

from gz.msgs.boolean_pb2 import Boolean
from gz.msgs.empty_pb2 import Empty
from gz.msgs.serialized_map_pb2 import SerializedStepMap
from gz.msgs.world_control_pb2 import WorldControl
from gz.msgs.world_control_state_pb2 import WorldControlState
from gz.transport import Node


def component_id(name):
    """Gazebo component IDs use gz::common::hash64 (64-bit FNV-1a)."""
    value = 0xcbf29ce484222325
    for byte in name.encode():
        value = ((value ^ byte) * 0x100000001b3) & 0xffffffffffffffff
    return value


BUOYANCY_TYPES = {
    component_id('gz_sim_components.Volume'),
    component_id('gz_sim_components.CenterOfVolume'),
}


def buoyancy_components(snapshot):
    return {
        entity_id: {
            key: deepcopy(component)
            for key, component in entity.components.items()
            if key in BUOYANCY_TYPES
        }
        for entity_id, entity in snapshot.state.entities.items()
        if any(key in BUOYANCY_TYPES for key in entity.components)
    }


class GazeboReset:
    def __init__(self, world='lake', node=None):
        self.node = node if node is not None else Node()
        self.prefix = f'/world/{world}/'

    def request(self, service, message, response_type):
        ok, response = self.node.request(
            self.prefix + service, message, type(message), response_type, 3000)
        if not ok or (isinstance(response, Boolean) and not response.data):
            raise RuntimeError(f'Gazebo service failed: {self.prefix}{service}')
        return response

    def state(self):
        return self.request('state', Empty(), SerializedStepMap)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            snapshot = self.state()
            if predicate(snapshot):
                return snapshot
            time.sleep(0.02)
        raise RuntimeError('Timed out waiting for Gazebo to apply the reset state')

    def reset(self, paused=False):
        initial = self.state()
        saved = buoyancy_components(initial)
        if not saved or any(set(items) != BUOYANCY_TYPES for items in saved.values()):
            raise RuntimeError(
                'Buoyancy metadata is missing or incomplete. Start a fresh simulator '
                'once with pobs stop / pobs sim, then use pobs reset for later resets.')

        control = WorldControl()
        control.pause = True
        self.request('control', control, Boolean)
        self.wait_for(lambda state: state.stats.paused)
        control.reset.all = True
        self.request('control', control, Boolean)

        def reset_applied(state):
            at_zero = state.stats.sim_time.sec == 0 and state.stats.sim_time.nsec == 0
            # At time zero, wait for metadata removal to establish that this
            # asynchronous reset was applied before restoring the components.
            was_at_zero = (initial.stats.sim_time.sec == 0
                           and initial.stats.sim_time.nsec == 0)
            return (state.stats.paused and at_zero
                    and (not was_at_zero or buoyancy_components(state) != saved))

        after = self.wait_for(reset_applied)
        if not set(saved).issubset(after.state.entities):
            raise RuntimeError('Reset changed the buoyant link IDs; world remains paused')

        restore = WorldControlState()
        restore.world_control.pause = True
        for entity_id, components in saved.items():
            entity = restore.state.entities.add()
            entity.id = entity_id
            for component in components.values():
                entity.components.add().CopyFrom(component)
        # Never restore poses, velocities, or other pre-reset state.
        self.request('control/state', restore, Boolean)
        self.wait_for(lambda state: buoyancy_components(state) == saved)
        if not paused:
            resume = WorldControl()
            resume.pause = False
            self.request('control', resume, Boolean)
        return len(saved)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='lake')
    parser.add_argument('--paused', action='store_true', help='leave Gazebo paused')
    args = parser.parse_args()
    try:
        count = GazeboReset(args.world).reset(paused=args.paused)
    except RuntimeError as error:
        print(f'Reset failed: {error}', file=sys.stderr)
        print('If the reset had started, leave Gazebo paused until resolved.', file=sys.stderr)
        return 1
    mode = 'paused' if args.paused else 'running'
    print(f'Gazebo reset ({mode}); restored buoyancy for {count} links. '
          'Container and ROS nodes kept running.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
<<<<<<< HEAD

=======
>>>>>>> fa5d34c652b143725b0f1a5955c850ad6e767c47
