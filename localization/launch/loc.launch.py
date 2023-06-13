import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory(
        'localization'), 'config', 'config.rviz')
    assert os.path.exists(rviz_config_dir)
    
    params = os.path.join(get_package_share_directory(
        'localization'), 'config', 'params.yaml')
    assert os.path.exists(params)

    map_path = os.path.join(get_package_share_directory(
        'localization'), 'resource', 'map.pcd')
    assert os.path.exists(map_path)
    
    # bag_dir = os.path.join(get_package_share_directory(
    #     'localization'), 'bag', 'rosbag.db3')
    # assert os.path.exists(bag_dir)
    

    return LaunchDescription([
        # Node(package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     output='screen'
        # ),
        # Node(package='localization',
        #     executable='map_pub',
        #     name='map_publisher',
        #     output='screen',
        #     arguments=[map_path],
        # ),
        # Node(package='localization',
        #     executable='icp_loc',
        #     name='icp_localization',
        #     output='screen',
        #     arguments=[map_path],
        #     parameters = [params]
        # ),
        Node(package='localization',
            executable='robot_tfs',
            name='robot_tfs',
            output='screen',
            arguments=[],
        ),
        # Node(package='localization',
        #     executable='fixed_frames',
        #     name='fixed_frame_broadcaster',
        #     output='screen',
        #     arguments=[],
        # ),
        
        # ExecuteProcess(
        #     cmd=['ros2', 'bag', 'play', bag_dir],
        #     output='screen', 
        #     log_cmd=True
        # )
    ])