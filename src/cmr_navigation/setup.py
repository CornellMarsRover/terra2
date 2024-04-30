from setuptools import setup

package_name = "cmr_navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/nav.launch.py']),
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
            # PHOBOS_APPEND
            'gps_node = cmr_navigation.gps:main',
            'gps_node_fake = cmr_navigation.gps_fake:main',
            'context_node = cmr_navigation.context:main',
            'drive_to_coord_node = cmr_navigation.drive_to_coord:main' 
        ],
    },
)