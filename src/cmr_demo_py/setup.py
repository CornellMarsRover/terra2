from setuptools import setup

package_name = "cmr_demo_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Some demo stuff for Python",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_effort_control = cmr_demo_py.arm_effort_control:main",
            "drives_calibrator = cmr_demo_py.drives_calibrator:main",
            # PHOBOS_APPEND
        ],
    },
)
