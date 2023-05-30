import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution

def generate_launch_description():
    print('Init')
    package_name = 'map_server'
    config_directory = os.path.join(get_package_share_directory(package_name), 'config')
    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','rviz_config.rviz')
    print('Init2 ')
    #~~~~~~~~~~~~~~~~~~Declare parameters~~~~~~~~~~~~~++
    # Declara el argumento para el archivo de configuración YAML
    config_file = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_real.yaml',
        description='Path to the map select'
    )
    config_file =  LaunchConfiguration('map_file')
    print('Init3')
    print(config_directory)
    # Define la ruta completa del archivo de mapa
    map_file_path = os.path.join(str(PathJoinSubstitution(config_file)),'dasd')

    print(map_file_path)
    print('Init3')
        # Imprime un comentario
    print(map_file_path)
    # Verifica si el archivo de mapa existe
    #check_map_file = launch.actions.OpaqueFunction(function=lambda context: print(f"Checking if map file '{map_file_path}' exists..."))


    return LaunchDescription([
        config_file,
        #check_map_file,    

        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file_path} 
                       ]),

        #~~~~~~~~~~~~~~~~~~rviz2~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
        
        #~~~~~~~~~~~~~~~~~~lifeclycler_manager~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])            
        ])