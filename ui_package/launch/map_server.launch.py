import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='path/to/your/map.yaml',
            description='Full path to map file'
        ),

        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
        )
    ])
