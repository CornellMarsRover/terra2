from setuptools import setup
import os
from glob import glob


package_name = "cmr_keyboard"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + '/config', ['config/yolov8n.pt']),
        (
            os.path.join("share", package_name, "templates"),
            glob(os.path.join("templates", "*.png")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Package for autonomous keyboard typing with the arm",
    license="MIT",
    entry_points={
        "console_scripts": [
            # PHOBOS_APPEND
            'keyboard_detection = cmr_keyboard.keyboard_detector:main',
            'computer_camera_publisher = cmr_keyboard.computer_cam_publisher_node:main',
            'keyboard_edges = cmr_keyboard.keyboard_edges:main',
        ],
    },
)
