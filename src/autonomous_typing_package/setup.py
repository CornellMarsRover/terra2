from setuptools import setup

package_name = "autonomous_typing_package"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name, ['key_position.json']),
        ("share/" + package_name + "/launch", ["launch/node.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CMR",
    maintainer_email="team@cornellmarsrover.org",
    description="CMR autonomous typing challenge",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            'autonomous_typing = autonomous_typing_package.typing_mission_ros:main',
            'arm_coordinator   = autonomous_typing_package.arm_coordinator:main',
            'manual_key_pose_pub = autonomous_typing_package.manual_key_pose_pub:main',
            'fake_near_ee_key_pose_pub = autonomous_typing_package.fake_near_ee_key_pose_pub:main',
            'direct_joint_nudge_pub = autonomous_typing_package.direct_joint_nudge_pub:main',
            'direct_moteus_nudge = autonomous_typing_package.direct_moteus_nudge:main',
            'ik_cartesian_nudge = autonomous_typing_package.ik_cartesian_nudge:main',
            'moteus_joint_state_pub = autonomous_typing_package.moteus_joint_state_pub:main',
        ],
    },
)
