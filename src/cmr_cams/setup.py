from setuptools import setup
from glob import glob

package_name = "cmr_cams"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Camera processing nodes for Cornell Mars Rover",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detection_node = cmr_cams.object_detection:main",
            # PHOBOS_APPEND
        ],
    },
)
