"""ament_python setup for astrotech_rover."""

from glob import glob
from setuptools import setup

package_name = "astrotech_rover"

setup(
    name=package_name,
    version="0.2.0",
    packages=[
        package_name,
        f"{package_name}.drivers",
        f"{package_name}.drivers.mock",
    ],
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
    description="Astrotech rover driver node (real drivers + optional mock) "
                "for the URC 2026 science-payload GCS.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "astrotech_node = astrotech_rover.astrotech_node:main",
        ],
    },
)
