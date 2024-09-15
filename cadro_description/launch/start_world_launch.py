#!/usr/bin/python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription,LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix

pkg_name='cadro_description'

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_models_dir = get_package_share_directory(pkg_name)

    # world_file_name = 'bookstore.world'
    # world_path = os.path.join(get_package_share_directory('cadro_description'), 'world', world_file_name)
    
    # # Set the Gazebo world path
    # gazebo_world = LaunchConfiguration('world', default=world_path)
    
    install_dir = get_package_prefix(pkg_name)

    launch_actions = []

    # Handle the environment variable logic
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     gazebo_models_path = os.path.join(pkg_models_dir, 'models')
    #     os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
    #     # Log the info that the path has been updated
    #     launch_actions.append(
    #         LogInfo(msg=f"iamhere GAZEBO_MODEL_PATH is set to: {gazebo_models_path}")
    #     )
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"
    #     # Log the info for the else case
    #     launch_actions.append(
    #         LogInfo(msg=f"GAZEBO_MODEL_PATH is set to: {install_dir}/share")
    #     )


    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Launch Gazebo with the ROS factory plugin
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        # launch_arguments={'world': gazebo_world}.items()  # Ensure factory plugin is included
    )   
    
    return LaunchDescription([
        launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        [get_package_share_directory(
            'aws_robomaker_bookstore_world'), '/launch/bookstore.launch.py']
    )
),
        # DeclareLaunchArgument(
        #   'world',
        #   default_value=[os.path.join(pkg_models_dir, 'world', 'service.world'), ''], # Change name of world file if required.
        #   description='SDF world file'),
        # gazebo,
        # ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen'),
    ])
