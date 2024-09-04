from setuptools import setup
import os
from glob import glob

package_name = 'manipulator_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    
    py_modules=[    ],
    install_requires=['setuptools'],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'kinematics_control = manipulator_control.kinematics_control:main',
            'manipulator_motionplanner = manipulator_control.manipulator_motionplanner:main',
            'manipulator_mover = manipulator_control.manipulator_mover:main'
        ],
    },
)
