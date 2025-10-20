from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration, SetEnvironmentVariable
from moveit_configs_utils.launches import generate_demo_launch
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # Definir carpeta de logs relativa
    log_dir = os.path.join(os.getcwd(), 'logs')
    os.makedirs(log_dir, exist_ok=True)

    moveit_config = MoveItConfigsBuilder("CRSA465", package_name="crsa465_moveit_config").to_moveit_configs()

    ld = LaunchDescription()
    # Definir variable de entorno ROS_LOG_DIR para redirigir logs
    ld.add_action(SetEnvironmentVariable(name='ROS_LOG_DIR', value=log_dir))
    ld.add_action(SetEnvironmentVariable(name='RCUTILS_CONSOLE_OUTPUT_FORMAT', value="[{severity} {time}] [{name}]: {message}"))
    ld.add_action(
        SetLaunchConfiguration(name="use_rviz", value="False")
    )
    ld.add_action(generate_demo_launch(moveit_config))
    ld.add_action(
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'default_call_service_timeout': 5.0,
                'call_services_in_new_thread': True,
                'send_action_goals_in_new_thread': True
            }],
            output='log',
        )
    )

    ld.add_action(
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi_node',
            output='both'
        )
    )

    ld.add_action(
        Node(
            package='CRSA465',
            executable='control_node',
            name='control_node',
            output='log'
        )
    )

    return ld
