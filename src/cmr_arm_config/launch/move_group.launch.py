from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (MoveItConfigsBuilder("arm", package_name="cmr_arm_config")
        .parameter('octomap_frame', 'base_link')
        .parameter('octomap_resolution', 0.05)
        .parameter('max_range', 5.0)
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
