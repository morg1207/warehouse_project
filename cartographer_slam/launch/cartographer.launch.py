import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'cartographer_slam'
    cartographer_config_dir = os.path.join(get_package_share_directory(package_name ), 'config')
    configuration_basename = 'cartographer.lua'
    rviz_file = os.path.join(get_package_share_directory(package_name ),'rviz','rviz_config.rviz')

    return LaunchDescription([

        #~~~~~~~~~~~~~~~~~~~SLAM ALGORITH~~~~~~~~~~~~~~~~~
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
        #~~~~~~~~~~~~~~~~~~~PUBLISH MAP~~~~~~~~~~~~~~~~~
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),


    ])