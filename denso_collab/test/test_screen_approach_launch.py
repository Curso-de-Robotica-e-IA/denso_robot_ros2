"""A failed stage must never start the next robot motion."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch_ros.actions import Node


source = Path(__file__).resolve().parents[1] / 'launch/screen_approach.launch.py'
spec = spec_from_file_location('screen_approach', source)
module = module_from_spec(spec)
spec.loader.exec_module(module)
module.get_package_share_directory = lambda name: '/tmp/' + name


def start(runs, random, sim=False, touch=False):
    context = LaunchContext()
    context.launch_configurations.update({
        'test_runs': str(runs), 'use_random_holder_pose': str(random).lower(),
        'sim': str(sim).lower(), 'touch_enabled': str(touch).lower(),
    })
    return module._start_tests(context)


def exit_actions(actions, code):
    handler = next(a for a in actions if isinstance(a, RegisterEventHandler)).event_handler
    callback = handler._OnActionEventBase__on_event
    return callback(SimpleNamespace(returncode=code), None)


assert isinstance(start(0, True)[0], EmitEvent)
assert isinstance(start(101, True)[0], EmitEvent)

first = start(2, True)
assert any(isinstance(a, Node) and a.node_executable == 'random_collab_pose' for a in first)
assert isinstance(exit_actions(first, 1)[0], EmitEvent)

screen = exit_actions(first, 0)
assert any(isinstance(a, IncludeLaunchDescription) for a in screen)
assert any(isinstance(a, Node) and a.node_executable == 'move_to_screen_standoff' for a in screen)
assert isinstance(exit_actions(screen, 1)[0], EmitEvent)

second = exit_actions(screen, 0)
assert any(isinstance(a, Node) and a.node_executable == 'random_collab_pose' for a in second)
assert not any(isinstance(a, IncludeLaunchDescription) for a in second)
last_screen = exit_actions(second, 0)
assert not any(isinstance(a, IncludeLaunchDescription) for a in last_screen)
assert isinstance(exit_actions(last_screen, 0)[0], EmitEvent)

without_random = start(2, False)
assert any(isinstance(a, IncludeLaunchDescription) for a in without_random)
assert not any(isinstance(a, Node) and a.node_executable == 'random_collab_pose' for a in without_random)
next_screen = exit_actions(without_random, 0)
assert not any(isinstance(a, IncludeLaunchDescription) for a in next_screen)
assert isinstance(exit_actions(next_screen, 0)[0], EmitEvent)

sim_touch = start(1, False, sim=True, touch=True)
assert any(isinstance(a, Node) and a.node_executable == 'sim_touch_contact.py' for a in sim_touch)
real_touch = start(1, False, sim=False, touch=True)
assert not any(isinstance(a, Node) and a.node_executable == 'sim_touch_contact.py' for a in real_touch)
