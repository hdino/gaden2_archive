from launch import LaunchDescription
import os
import launch_ros.actions
import pathlib

scenario = '10x6_empty_room'

def generate_launch_description():
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
    
    return LaunchDescription([
        launch_ros.actions.Node(
            package='gaden_preprocessing',
            node_executable='preprocessing',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'base_path': scenario_path_str}
            ],
        ),
    ])
