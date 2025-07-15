# base_node_launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PKG = 'ardupilot_bringup'

def generate_launch_description():
    # Declare launch arguments
    package_share_dir = get_package_share_directory(PKG)
    plugin_path = f"{package_share_dir}/config/plugins.yaml"
    config_path = f"{package_share_dir}/config/config.yaml"

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    pluginlists_yaml = LaunchConfiguration('pluginlists_yaml')
    config_yaml = LaunchConfiguration('config_yaml')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value="tcp://:5760@127.0.0.1:5760"),
        DeclareLaunchArgument('gcs_url', default_value="udp://@127.0.0.1:14550"),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component',default_value='1'),
        DeclareLaunchArgument('pluginlists_yaml', default_value=plugin_path),
        DeclareLaunchArgument('config_yaml', default_value=config_path),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),
        DeclareLaunchArgument('namespace', default_value='mavros'),

        Node(
            package='mavros',
            executable='mavros_node',
            namespace=namespace,
            output=log_output,
            parameters=[
                {'fcu_url': fcu_url},
                {'gcs_url': gcs_url},
                {'tgt_system': tgt_system},
                {'tgt_component': tgt_component},
                {'fcu_protocol': fcu_protocol},
                pluginlists_yaml,
                config_yaml,
            ]
        )
    ])
