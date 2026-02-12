import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_agro = get_package_share_directory('agro_robot_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 1. Argumento do Mundo
    # O default é vazio. Se vazio, o Gazebo abre o mundo padrão.
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Caminho completo para o arquivo world'
    )

    # 2. Configurar o Gazebo Server
    # Aqui passamos o argumento 'world' explicitamente
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 3. Configurar o Gazebo Client (Interface)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 4. Processar o Robô (URDF/Xacro)
    xacro_file = os.path.join(pkg_agro, 'urdf', 'robo_caatinga.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    # 5. Publicar Estado do Robô (TF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # 6. Spawnar o Robô no Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', '-entity', 'agro_robot',
            '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot
    ])
