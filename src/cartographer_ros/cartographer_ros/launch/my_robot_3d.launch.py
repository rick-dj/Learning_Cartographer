import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', 
                os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files'),
                '-configuration_basename', 'my_robot_3d.lua'
            ],
            remappings=[
                ('points2', '/camera/depth/points'),
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])
