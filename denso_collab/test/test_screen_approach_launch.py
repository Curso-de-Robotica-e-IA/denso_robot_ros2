"""Check that a failed random pose cannot start screen approach."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext
from launch.actions import EmitEvent, GroupAction, IncludeLaunchDescription
from launch_ros.actions import Node


source = Path(__file__).resolve().parents[1] / 'launch/screen_approach.launch.py'
spec = spec_from_file_location('screen_approach', source)
module = module_from_spec(spec)
spec.loader.exec_module(module)

assert isinstance(module.after_random(SimpleNamespace(returncode=1), None)[0], EmitEvent)
assert isinstance(module.after_random(SimpleNamespace(returncode=0), None)[0], IncludeLaunchDescription)

actions = module.generate_launch_description().entities
random_pose = next(action for action in actions if isinstance(action, Node))
screen = next(action for action in actions if isinstance(action, GroupAction))
context = LaunchContext()
context.launch_configurations['use_random_holder_pose'] = 'false'
assert not random_pose.condition.evaluate(context) and screen.condition.evaluate(context)
context.launch_configurations['use_random_holder_pose'] = 'true'
assert random_pose.condition.evaluate(context) and not screen.condition.evaluate(context)
