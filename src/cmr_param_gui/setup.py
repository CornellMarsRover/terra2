from setuptools import setup

package_name = "cmr_param_gui"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/gui.launch.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="Basic Parameter GUI for Rover Drives and Arm Dev (NOT FOR COMP)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # PHOBOS_APPEND
            'gui_node = cmr_param_gui.gui:main'
        ],
    },
)
