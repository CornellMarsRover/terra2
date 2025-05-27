#!/usr/bin/env python3
"""
Launch file to start Gazebo with a randomly selected Gazebo world file.
"""

import os
import random
import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    package = get_package_share_directory('rover_gazebo')

    gazebo_model_path = os.path.join(package, 'gazebo_models')

    # Set the GAZEBO_MODEL_PATH environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # Define the path to the 'worlds' directory
    worlds_dir = os.path.join(package, 'worlds')

    # Pattern to match random world files
    randworld_pattern = os.path.join(worlds_dir, 'randworld_condensed*.world')

    # Use glob to find all matching world files
    randworld_files = glob.glob(randworld_pattern)

    # Randomly select one world file from the list
    selected_world_file = random.choice(randworld_files)
    empty_world_file = os.path.join(worlds_dir, 'emptyworld.world')
    #selected_world_file = empty_world_file
    terrain_file = '/home/cmr/cmr/elevation_maps/utah.world'
    #selected_world_file = terrain_file
    log_message = f"Randomly selected world file {selected_world_file}"

    # Log the selected world file
    log_info = LogInfo(msg=log_message)

    # Get the path to the 'gazebo_ros' package's launch file
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')

    # Include the Gazebo launch file, specifying the selected world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': selected_world_file, 'verbose': 'false'}.items(),
    )

    return LaunchDescription([
        log_info,
        gazebo
    ])
