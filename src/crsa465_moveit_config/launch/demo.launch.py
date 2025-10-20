from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription 

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("CRSA465", package_name="crsa465_moveit_config").to_moveit_configs()

    # 1. Start with an empty LaunchDescription
    ld = LaunchDescription()

    # 2. Set the 'use_rviz' launch configuration to 'False'
    # This overrides the default value declared inside generate_demo_launch
    ld.add_action(
        SetLaunchConfiguration(
            name="use_rviz", 
            value="False"
        )
    )

    # 3. Add the result of the generate_demo_launch function
    # It will use the 'use_rviz' value set above
    ld.add_action(generate_demo_launch(moveit_config))

    return ld