# ...existing code...
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
            parameters=[{'use_sim_time': True}],   # 实时传感器通常为 False；若回放 bag 则改为 True
            arguments=[
                '-configuration_directory',
                os.path.join(get_package_share_directory('cartographer_ros'), 'configuration_files'),
                '-configuration_basename', 'my_robot_2d.lua'   # 改为 2D 配置
            ],
            remappings=[
                ('scan', '/scan'),   # 将 cartographer 的 scan 订阅重映射到实际激光话题
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
# ...existing code...