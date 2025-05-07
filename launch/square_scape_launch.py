from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    andino_gz_launch_path = os.path.join(
        get_package_share_directory('andino_gz_classic'),
        'launch',
        'andino_one_robot.launch.py'
    )

    world_path = os.path.join(
        get_package_share_directory('turtle_controller'),
        'worlds',
        'square_pass.world'
    )

    andino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_gz_launch_path),
        launch_arguments={'rviz': 'true', 'world': world_path}.items()
    )

    controller_server_node = Node(
        package='turtle_controller',
        executable='controller_server',
        name='controller_server',
        output='screen'
    )

    keyboard_node = Node(
        package='keyboard',
        executable='keyboard',
        name='keyboard',
        output='screen'
    )

    controler_action = Node(
        package='turtle_controller',
        executable='controller_scape.py',
        name='controller_scape',
        output='screen'
    )

    # Node do pacote 'turtle_controller' com nome 'keyboard_parser'
    key_parser = Node(
        package='turtle_controller',
        executable='keyboard_parser',
        name='keyboard_parser',
        output='screen'
    )

    return LaunchDescription([
        andino_launch,
        controller_server_node,
        keyboard_node,
        controler_action,
        key_parser
    ])
