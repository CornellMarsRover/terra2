from setuptools import setup

package_name = "cmr_rovernet"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            'share/' + package_name + '/launch',
            [
                'launch/rovernet.launch.py',
                'launch/steer_only.launch.py',
                'launch/drive_only.launch.py',
                'launch/single_drive_wheel.launch.py',
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="The CCB-Jetson communication package omitting the micro ros agent",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "armnet_node = cmr_rovernet.armnet:main",
            # PHOBOS_APPEND
            'drivesnet_node = cmr_rovernet.drivesnet:main',
            'usama_control_testing_node = cmr_rovernet.usama_control_testing:ros_main',
            'steer_only_control_node = cmr_rovernet.steer_only_control:ros_main',
            'drive_only_control_node = cmr_rovernet.drive_only_control:ros_main',
            'single_drive_wheel_control_node = cmr_rovernet.single_drive_wheel_control:ros_main',
            'ackermann_keyboard_node = cmr_rovernet.ackermann_keyboard:main',
            'cmr_read_node = cmr_rovernet.ccb_read:main',
            'debug_node = cmr_rovernet.debug:main'
        ],
    },
)
