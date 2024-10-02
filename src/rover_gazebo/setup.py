import os
from glob import glob

from setuptools import setup

package_name = "rover_gazebo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        # install yaml parameter files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        # install rviz config files
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        # install map files
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.world")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ajay",
    maintainer_email="aks237@cornell.edu",
    description="Gazebo simulation rover",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
