from setuptools import setup, find_packages

package_name = 'cmr_servo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/servo_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CMR',
    maintainer_email='team@cornellmarsrover.org',
    description='ROS2 bridge for CMR servo board end effector controls.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_hw_node = cmr_servo_control.servo_hw_node:main',
            'arm_servo_control_node = cmr_servo_control.arm_servo_control_node:main',
            'keyboard_input_node = cmr_servo_control.keyboard_input_node:main',
            'servo_probe = cmr_servo_control.servo_probe:main',
        ],
    },
)
