import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    package_name = 'attach_shelf'
    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','rviz_config.rviz')


    return LaunchDescription([
        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='attach_shelf',
            executable='approach_service_server_node',
            name='attach_shelf',
            output='screen',
        ),
        #~~~~~~~~~~~~~~~~~~rviz2~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        )
         
        ])