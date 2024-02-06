from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("Arm", package_name="cmr_arm_simulator").to_moveit_configs()
    return generate_static_virtual_joint_tfs_launch(moveit_config)
