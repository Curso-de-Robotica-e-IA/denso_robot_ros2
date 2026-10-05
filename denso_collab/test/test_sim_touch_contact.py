"""Virtual contact must require the screen bounds and a small positive gap."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


source = Path(__file__).resolve().parents[1] / 'scripts/sim_touch_contact.py'
spec = spec_from_file_location('sim_touch_contact', source)
module = module_from_spec(spec)
spec.loader.exec_module(module)
inside = module.within_touch_region

assert inside(0, 0, 0.001, 0.070, 0.150, 0.002)
assert not inside(0, 0, 0.003, 0.070, 0.150, 0.002)
assert not inside(0, 0, -0.001, 0.070, 0.150, 0.002)
assert not inside(0.04, 0, 0.001, 0.070, 0.150, 0.002)
assert not inside(0, 0.08, 0.001, 0.070, 0.150, 0.002)
