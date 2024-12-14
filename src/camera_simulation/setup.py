from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'camera_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/camera_simulation_launch.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/camera_world.wbt']),
        ('share/' + package_name + '/textures', glob('worlds/textures/*')),
        ('share/' + package_name + '/resource', ['resource/camera.urdf']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Camera simulation using Webots and ROS 2',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # PHOBOS APPEND
            'camera_controller = camera_simulation.camera_controller:main'
        ],
    },
)
