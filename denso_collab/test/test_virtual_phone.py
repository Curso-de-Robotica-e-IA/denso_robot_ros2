"""Screen selection may change the texture, never frames or collision shapes."""

import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


source = Path(get_package_share_directory('denso_robot_descriptions')) / 'urdf/dual_denso_robot.urdf.xacro'


def model(target):
    xml = subprocess.check_output([
        'xacro', str(source), 'model:=vs050', 'sim:=true', 'right_virtual_phone:=true',
        'left_hall_touch_camera:=true', 'target:=' + target,
    ])
    return ET.fromstring(xml)


rgb, captcha = model('rgb'), model('captcha')
assert [link.attrib for link in rgb.findall('link')] == [link.attrib for link in captcha.findall('link')]
assert [ET.tostring(j) for j in rgb.findall('joint')] == [ET.tostring(j) for j in captcha.findall('joint')]
assert [ET.tostring(c) for c in rgb.findall('link/collision')] == [ET.tostring(c) for c in captcha.findall('link/collision')]
for root, filename in ((rgb, 'phone_screen.obj'), (captcha, 'phone_captcha.obj')):
    visual = root.find("link[@name='right_virtual_phone_screen_frame']/visual/geometry/mesh")
    assert visual.get('filename').endswith('/' + filename)
    mesh = Path(visual.get('filename').removeprefix('file://'))
    assert mesh.exists()
    material = mesh.with_suffix('.mtl')
    assert material.exists()
    texture_name = next(line.split()[1] for line in material.read_text().splitlines()
                        if line.startswith('map_Kd '))
    assert (material.parent / texture_name).exists()
