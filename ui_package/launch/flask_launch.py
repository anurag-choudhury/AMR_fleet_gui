#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_ros.actions
from launch.actions import ExecuteProcess



def generate_launch_description():
    flask_script = PathJoinSubstitution([
        FindPackageShare('ui_package'), 
        'src',
    ])
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', 'flask_app.py'],
            cwd=flask_script,
            output='screen'
        )
    ])
