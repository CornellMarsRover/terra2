"""ament_python setup for urc_mock_rover."""

from glob import glob
from setuptools import setup

package_name = "urc_mock_rover"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, f"{package_name}.drivers"],
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Mock Astrotech rover for GCS development.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_rover_node = urc_mock_rover.mock_rover_node:main",
        ],
    },
)
