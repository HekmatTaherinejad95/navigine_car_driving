import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import ExecuteProcess
import launch_ros.actions

def generate_launch_description():

    
    return LaunchDescription([

        launch.actions.ExecuteProcess( 
         cmd=['ros2', 'launch', 'velodyne', 'velodyne-all-nodes-VLP16-launch.py'], 
         output='screen'),

        Node(
            package='wheeltec_n100_imu',
            executable='imu_node',
            name='imu_node'
        ),

        Node(
            package='basler_ros2',
            executable='basler_node',
            name='basler'
        ),
        
        launch.actions.ExecuteProcess( 
         cmd=['ros2', 'run', 'atgm336h5n3x', 'nmea_node', '--dev', '/dev/ttyUSB0'], 
         output='screen'),

        launch.actions.ExecuteProcess( 
         cmd=['ros2', 'launch', 'localization', 'loc.launch.py'], 
         output='screen')

        
        
        # Add more nodes as needed for your packages

    ])
