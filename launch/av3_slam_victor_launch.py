from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    rviz_config = os.path.join(
                    get_package_share_directory('andino_slam'), 
                    'rviz', 
                    'andino_slam.rviz'
    )    
    
    andino_gz_launch_path = os.path.join(
        get_package_share_directory('andino_gz_classic'),
        'launch',
        'andino_one_robot.launch.py'
    )

    andino_slam_launch_path = os.path.join(
        get_package_share_directory('andino_slam'),
        'launch',
        'slam_toolbox_online_async.launch.py'
    )

    andino_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_gz_launch_path),
        launch_arguments={'rviz': 'true', 
                            'world': 'src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_house.world',
                            'initial_pose_x': '-4.50',
                            'initial_pose_y': '0.5',
                            'initial_pose_z': '0.01',
                            'initial_pose_yaw': '0.00',
                            'rviz_config_file': rviz_config}.items()                           
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(andino_slam_launch_path)
    )
 
    teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node',
            output='screen',
            prefix = 'xterm -e',
         )

    return LaunchDescription([
        andino_launch,
        teleop_node,
        slam_launch
    ])
