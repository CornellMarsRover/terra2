from setuptools import setup

package_name = "cmr_py_test"

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
    description="Integration and node-level unit tests in Python",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "utils_test_node = cmr_py_test.utils_test_node:main",
            # PHOBOS_APPEND
        ],
    },
)
