import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('vins'),
      'config',
      'realsense_mono_imu_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='vins',
            executable='vins_node',
            name='vins_node',
            parameters=[config]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])