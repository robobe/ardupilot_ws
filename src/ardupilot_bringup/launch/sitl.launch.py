
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

PKG = 'ardupilot_bringup'

def generate_launch_description():
    model_declare = DeclareLaunchArgument('model', description="copter model: quad, JSON", default_value='quad')
    param_file_declare = DeclareLaunchArgument('param_file', description="Path to the parameter file", default_value='copter_default.param')

    model = LaunchConfiguration('model')
    param_file = LaunchConfiguration('param_file')

    package_share_dir = get_package_share_directory(PKG)
    sitl_bin = f"{package_share_dir}/bin/arducopter"
    param_path = PathJoinSubstitution([
        package_share_dir,
        'params',
        param_file
    ])
    

    ld = LaunchDescription()
    sitl = ExecuteProcess(
            cmd=[sitl_bin, '--model', model, '--speedup', '1', '--slave', '0',
                 '--defaults', param_path,
                 '--sim-address', '127.0.0.1', '-I0'],
            output='screen'
        )
    
    ld.add_action(model_declare)
    ld.add_action(param_file_declare)
    ld.add_action(sitl)
    return ld