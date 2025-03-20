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
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py']),
        ('share/' + package_name + '/launch', ['launch/costmap.launch.py']),
        ('share/' + package_name + '/launch', ['launch/localization_real.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sim_autonomy.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_costmap_params.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_real.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_engquad.yaml']),
        ('share/' + package_name + '/config', ['config/sim_waypoints_condensed.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Rover navigation package using GPS waypoints and Twist messages',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # PHOBOS APPEND
            'state_machine = autonomous_navigation.state_machine:main',
            'planner = autonomous_navigation.planner:main',
            'localization_sim = autonomous_navigation.localization_sim:main',
            'object_recognition_sim = autonomous_navigation.object_recognition:main',
            'costmap_sim = autonomous_navigation.costmap_sim:main',
            'controller = autonomous_navigation.controller:main',
            'ekf_localization = autonomous_navigation.ekf_localization:main',
            'kalman_localization = autonomous_navigation.kalman_localization:main',
            'imu_calibration = autonomous_navigation.imu_calibration:main',
            'ekf = autonomous_navigation.ekf_node:main',
            'rtk_localization = autonomous_navigation.rtk_localization:main',
            'costmap_real = autonomous_navigation.costmap_real:main',
            'rtk_filter = autonomous_navigation.rtk_filtered:main',
            'ukf = autonomous_navigation.ukf:main',
            'p2 = autonomous_navigation.planner2:main',
            'ukf_yaw = autonomous_navigation.ukf_yaw:main',
        ],
    },
)
