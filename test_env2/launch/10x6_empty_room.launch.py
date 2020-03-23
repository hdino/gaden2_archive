from launch import LaunchDescription
#from launch.substitutions import EnvironmentVariable
#from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.actions
import pathlib

def generate_launch_description():
    launch_file_extension = '.launch.py'
    
    # get the path of this file in the install folder
    launch_file_path = pathlib.Path(__file__)
    
    launch_file_name = launch_file_path.name
    if not launch_file_name.endswith(launch_file_extension):
        raise RuntimeError('Name of launch file should end with .launch.py')
    
    # the scenario name is the file name without its suffix
    # e.g.: scenario1.launch.py would become scenario1
    scenario = launch_file_name[:-len(launch_file_extension)]
    
    # the following block transforms the path:
    # workspace/install/PACKAGE_NAME/share/PACKAGE_NAME/launch/THIS_FILE.launch.py
    # into the path:
    # workspace/src/PACKAGE_NAME/SCENARIO/
    # (always with a trailing path separator)
    scenario_path_temp = list(pathlib.Path(__file__).parts)[:-4]
    scenario_path_temp[-2] = 'src'
    scenario_path_temp.append(scenario)
    scenario_path = pathlib.Path(*scenario_path_temp)
    scenario_path_str = str(scenario_path)
    if not scenario_path_str.endswith(os.path.sep):
        scenario_path_str += os.path.sep
    
    #config_file = str(scenario_path / 'config.yaml')
    
    #print(scenario_path_str)
    #print(config_file)
    
    #print(pathlib.Path(__file__).parents[5])
    
    #parameters_file_path = launch_file_path[:-len(launch_file_extension)] + '.config.yaml'
    
    #install_path = pathlib.Path(__file__).parents[1] # get current path and go one level up
    #parameters_file_path = pathlib.Path(install_path, 'launch', 'test.config.yaml')
    #print(parameters_file_path)
    #parameters_file_path += '/config/' + parameters_file_name
    #parameters_file_path = pathlib.Path(get_package_share_directory('test_env2'), 'launch', 'test.config.yaml')
    #print(parameters_file_path)
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='gaden_environment',
            node_executable='environment',
            output='screen',
            emulate_tty=True,
            parameters=[
                #config_file,
                {'base_path': scenario_path_str}
            ],
        ),
    ])
