from setuptools import find_packages, setup
from glob import glob
package_name = 'cmr_zed'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', glob('srv/*.srv')),
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
            'zed_publisher_node = cmr_zed.zed_camera_publisher:main',
            'zed_pointcloud = cmr_zed.zed_pointcloud_publisher:main',
        ],
    },
)
