from setuptools import find_packages, setup

package_name = 'cmr_rtkgps'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            # PHOBOS APPEND
            'gps_basestation = cmr_rtkgps.basestation:main',
            'basestation_known = cmr_rtkgps.basestation_known:main',
            'gps_rover = cmr_rtkgps.rover:main',
            'data_logger = cmr_rtkgps.gps_logger:main',
            'displacement_logger = cmr_rtkgps.displacement_logger:main',
        ],
    },
)
