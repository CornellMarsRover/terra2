import os
from glob import glob

from setuptools import setup

package_name = 'cmr_controls'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmr',
    maintainer_email='aks237@cornell.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # PHOBOS_APPEND
            'swerve_controller_node = cmr_controls.swerve_controller_node:main',
            'arm_controller_node = cmr_controls.arm_controller_node:main',
            'ik_node = cmr_controls.ik_node:main',
            'keyboard_controller_node = cmr_controls.keyboard_controller_node:main'
        ],
    },
)
