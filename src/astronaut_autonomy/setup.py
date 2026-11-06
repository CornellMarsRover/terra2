from glob import glob

from setuptools import find_packages, setup


package_name = "astronaut_autonomy"

config_files = [
    "config/astronaut_mission.yaml",
    "config/command_fusion.yaml",
    "config/human_gesture_detection.yaml",
    "config/voice_command.yaml",
]
config_files.extend(glob("config/yolo*-pose.pt"))
launch_files = glob("launch/*.launch.py")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", config_files),
        (
            "share/" + package_name + "/launch",
            launch_files,
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Astronaut gesture/voice perception and mission control",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "human_gesture_detection = "
            "astronaut_autonomy.human_gesture_detection:main",
            "astronaut_mission = astronaut_autonomy.astronaut_mission:main",
            "astronaut_command_fusion = "
            "astronaut_autonomy.command_fusion_node:main",
            "astronaut_voice_command = "
            "astronaut_autonomy.voice_command_node:main",
            "astronaut_mission_simulator = "
            "astronaut_autonomy.astronaut_mission_simulator:main",
        ],
    },
)
