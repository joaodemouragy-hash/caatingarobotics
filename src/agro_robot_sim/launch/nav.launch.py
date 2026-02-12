import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_name = 'agro_robot_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Processar URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robo_caatinga.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 2. Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # 3. Spawn Robot
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'agro_robot', '-z', '0.5'],
        output='screen'
    )

    # 4. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 5. Inicia o SLAM Toolbox (Mapeamento)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            )
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        slam,
    ])
