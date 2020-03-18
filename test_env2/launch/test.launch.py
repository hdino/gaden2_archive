from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.actions
import pathlib

#parameters_file_name = 'default.yaml'

def generate_launch_description():
    launch_file_path = str(pathlib.Path(__file__))
    if not launch_file_path.endswith('.launch.py'):
        raise RuntimeError('Name of launch file should end with .launch.py')
    
    install_path = pathlib.Path(__file__).parents[1] # get current path and go one level up
    parameters_file_path = pathlib.Path(install_path, 'launch', 'test.config.yaml')
    print(parameters_file_path)
    #parameters_file_path += '/config/' + parameters_file_name
    parameters_file_path = pathlib.Path(get_package_share_directory('test_env2'), 'launch', 'test.config.yaml')
    print(parameters_file_path)
    return LaunchDescription([
        launch_ros.actions.Node(
            package='gaden_environment',
            node_executable='environment',
            output='screen',
            emulate_tty=True,
            parameters=[
                parameters_file_path
            ],
         ),
    ])
