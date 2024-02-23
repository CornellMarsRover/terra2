from setuptools import setup
import os

package_name = "cmr_arm_sim"

urdf_files = [os.path.join('urdf', file) for file in os.listdir('urdf') if file.endswith('.urdf') or file.endswith('.xacro')]
mesh_files = [os.path.join('meshes', file) for file in os.listdir('meshes') if file.endswith('.stl')]


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        (f"share/{package_name}/urdf", urdf_files),
        (f"share/{package_name}/meshes", mesh_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="The arm simulator for the 2024 CMR arm",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # PHOBOS_APPEND
        ],
    },
)
