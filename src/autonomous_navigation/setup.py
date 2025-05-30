# setup.py

from setuptools import setup

package_name = 'autonomous_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/localization_real.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_autonomy.launch.py']),
        ('share/' + package_name + '/config', ['config/sim_waypoints_condensed.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_engquad.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_engquad2.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_test_site_utah.yaml']),
        ('share/' + package_name + '/config', ['config/best.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Rover Autonomy',
    license='Apache License 2.0',
    tests_require=[],
    entry_points={
        'console_scripts': [
            # PHOBOS APPEND
            'state_machine = autonomous_navigation.state_machine:main',
            'global_planner = autonomous_navigation.global_planner:main',
            'local_planner = autonomous_navigation.local_planner:main',
            'costmap = autonomous_navigation.costmap:main',
            'controller = autonomous_navigation.controller:main',
            'kalman_localization = autonomous_navigation.kalman_localization:main',
            'localization_sim = autonomous_navigation.localization_sim:main',
            'object_detection = autonomous_navigation.object_detection:main',
            'led_node = autonomous_navigation.led_node:main',
            'new_kalman = autonomous_navigation.new_kalman:main',
        ],
    },
)
