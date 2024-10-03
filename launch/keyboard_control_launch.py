from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Caminho do launch file 'andino_one_robot.launch.py' no pacote 'andino_gz_classic'
    andino_gz_launch_path = os.path.join(
        get_package_share_directory('andino_gz_classic'),
        'launch',
        'andino_one_robot.launch.py'
    )

    # IncludeLaunchDescription para o launch file 'andino_one_robot.launch.py' com parâmetro rviz false
    andino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_gz_launch_path),
        launch_arguments={'rviz': 'false'}.items()
    )

    # Node do pacote 'turtle_controller' com nome 'controller_server'
    controller_server_node = Node(
        package='turtle_controller',
        executable='controller_server',
        name='controller_server',
        output='screen'
    )

    # Node do pacote 'keyboard' com nome 'keyboard'
    keyboard_node = Node(
        package='keyboard',
        executable='keyboard',
        name='keyboard',
        output='screen'
    )

    # Retorna o LaunchDescription com os três componentes
    return LaunchDescription([
        andino_launch,
        controller_server_node,
        keyboard_node
    ])
