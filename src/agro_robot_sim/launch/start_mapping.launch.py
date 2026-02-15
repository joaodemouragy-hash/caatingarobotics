import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = 'agro_robot_sim'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Caminhos dos arquivos
    xacro_file = os.path.join(pkg_share, 'urdf', 'robo_caatinga.urdf.xacro')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    # Vamos criar esse arquivo já já.
    rviz_config_file = os.path.join(pkg_share, 'config', 'mapping.rviz')

    # 2. Processar URDF
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 3. Node: Robot State Publisher (Publica a TF do robô)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 4. Gazebo (Simulação)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        )
    )

    # 5. Spawn do Robô (Coloca o robô no Gazebo)
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'agro_robot', '-z', '0.5'],
        output='screen'
    )

    # 6. SLAM Toolbox (Cria o MAP e corrige a posição)
    # Usamos um TimerAction para dar tempo do Gazebo carregar antes de iniciar o SLAM
    slam = TimerAction(
        period=5.0,  # Espera 5 segundos
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('slam_toolbox'),
                        'launch',
                        'online_async_launch.py',
                    )
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam_params_file': slam_params_file
                }.items()
            )
        ]
    )

    # 7. RViz2 (Visualização)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gazebo,
        spawn,
        slam,
        rviz
    ])
