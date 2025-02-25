from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare 
import launch_ros.actions
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Load parameters from YAML file
    config_file = PathJoinSubstitution([
        FindPackageShare('ui_package'),
        'param',
        'config.yaml'
    ])
    flask_script = PathJoinSubstitution([
        FindPackageShare('ui_package'), 
        'src', 
        'flask_app.py'
    ])
    flask_dir =get_package_share_directory("ui_package")
    flask_launch =IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(flask_dir,'launch', 'flask_launch.py'))
            )
    # Include other launch files
    folder_handler_node=launch_ros.actions.Node(
            package='ui_package',
            executable='folders_handler.py',
            name='ui_folder_handler',
            output='screen'
        )
    rosbridge_server = launch_ros.actions.Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridgeserver',
            output='screen'
        )

    way_points_navigation_launch = launch_ros.actions.Node(
            package='ui_package',
            executable='way_points_handler.py',
            name='ui_folder_handler',
            output='screen'
        )

    # Create the launch description
    return LaunchDescription([
        folder_handler_node,
        rosbridge_server,
        way_points_navigation_launch,
        flask_launch
    #      ExecuteProcess(
    #      cmd=['python3', flask_script],
    #      output='screen'
    #  ),
        
    ])
