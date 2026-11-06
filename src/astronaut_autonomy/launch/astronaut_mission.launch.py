from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("astronaut_autonomy"))
    mission_config = str(share / "config" / "astronaut_mission.yaml")
    fusion_config = str(share / "config" / "command_fusion.yaml")
    voice_config = str(share / "config" / "voice_command.yaml")
    enable_drive_adapter = LaunchConfiguration("enable_drive_adapter")
    enable_voice = LaunchConfiguration("enable_voice")
    voice_model_path = LaunchConfiguration("voice_model_path")
    voice_input_mode = LaunchConfiguration("voice_input_mode")
    voice_wav_file = LaunchConfiguration("voice_wav_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_drive_adapter",
                default_value="false",
                description=(
                    "Publish astronaut commands to /cmd_vel/autonomy; leave "
                    "false until rover integration is reviewed"
                ),
            ),
            DeclareLaunchArgument(
                "enable_voice",
                default_value="false",
                description="Start the offline Vosk voice-command source",
            ),
            DeclareLaunchArgument(
                "voice_model_path",
                default_value="",
                description="Directory containing an extracted Vosk model",
            ),
            DeclareLaunchArgument(
                "voice_input_mode",
                default_value="microphone",
                description="Voice input: microphone or wav",
            ),
            DeclareLaunchArgument(
                "voice_wav_file",
                default_value="",
                description="Mono PCM16 WAV file used in wav mode",
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_command_fusion",
                name="astronaut_command_fusion",
                output="screen",
                parameters=[fusion_config],
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_voice_command",
                name="astronaut_voice_command",
                output="screen",
                parameters=[
                    voice_config,
                    {
                        "model_path": ParameterValue(
                            voice_model_path, value_type=str
                        ),
                        "input_mode": ParameterValue(
                            voice_input_mode, value_type=str
                        ),
                        "wav_file": ParameterValue(
                            voice_wav_file, value_type=str
                        ),
                    },
                ],
                condition=IfCondition(enable_voice),
            ),
            Node(
                package="astronaut_autonomy",
                executable="astronaut_mission",
                name="astronaut_mission",
                output="screen",
                parameters=[
                    mission_config,
                    {
                        "enable_drive_adapter": ParameterValue(
                            enable_drive_adapter, value_type=bool
                        )
                    },
                ],
            ),
        ]
    )
