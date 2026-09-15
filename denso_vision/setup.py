"""Setuptools definition for denso_vision."""

from glob import glob

from setuptools import find_packages, setup


package_name = 'denso_vision'

setup(
    name=package_name,
    version='1.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DENSO WAVE INCORPORATED',
    maintainer_email='fa-support@denso-wave.com',
    description=(
        'AprilTag detection and planar coordinate estimation for the DENSO '
        'cellphone holder.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cellphone_holder_detector = '
            'denso_vision.cellphone_holder_detector:main',
        ],
    },
)
