from setuptools import setup

package_name = "cmr_controller_remote"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/connect.launch.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Remote connection scripts for Jetson to receive remote control inputs wirelessly",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # PHOBOS_APPEND
            'connect_node = cmr_controller_remote.connect:main',
            'send_miniarm = cmr_controller_remote.miniarm_udp:main',
        ],
    },
)
