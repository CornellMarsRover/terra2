from setuptools import setup, find_packages

package_name = "cmr_imu"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),  # This will include 'cmr_imu' and 'cmr_imu.lib' and subpackages
    package_data={'': ['*.py', '*.yaml', '*.json', '*.launch.py']},  # Include necessary data files
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Package to obtain data from the IMU sensor",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # PHOBOS_APPEND
            'imu_node = cmr_imu.imu:main',
            'device_model = cmr_imu.device_model',
            'idp = cmr_imu.i_data_processor', 
            'ipr = cmr_imu.i_protocol_resolver',
            'jdp = cmr_imu.jy901s_dataProcessor',
            'p4r = cmr_imu.protocol_485_resolver' 
        ],
    },
)
