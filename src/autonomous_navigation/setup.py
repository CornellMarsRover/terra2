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
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_real.yaml']),
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
            'ackerman_real = autonomous_navigation.nav_ackerman_real:main',
            'detect_ground = autonomous_navigation.ground_plane_detection:main'
        ],
    },
)
