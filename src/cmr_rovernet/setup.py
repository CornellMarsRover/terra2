from setuptools import setup

package_name = "cmr_rovernet"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/rovernet.launch.py']),
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
            'drivesnet_node = cmr_rovernet.drivesnet:main'
        ],
    },
)
