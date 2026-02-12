import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_agro_sim = get_package_share_directory('agro_robot_sim')

    # Caminho do mapa que você salvou (ajuste se necessário)
    map_file = os.path.join(pkg_agro_sim, 'maps', 'meu_mapa.yaml')

    # Parâmetros padrão do Nav2
    params_file = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': params_file,
                'autostart': 'true',  # Inicia automaticamente
            }.items()
        )
    ])
