from setuptools import find_packages, setup

package_name = 'cmr_arm_arucos'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/aruco_detection.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cmr',
    maintainer_email='aks237@cornell.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=[],
    entry_points={
        'console_scripts': [
            # PHOBOS_APPEND
            'arm_aruco_detection = cmr_arm_arucos.aruco_detection_node:main',
            'simple_camera_node = cmr_arm_arucos.simple_camera_node:main'
        ],
    },
)
