from setuptools import setup

package_name = "cmr_aruco_py"

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
    description="Python nodes for AR tag detection",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "aruco_marker_translate = cmr_aruco_py.aruco_marker_translate:main",
            # PHOBOS_APPEND
        ],
    },
)
