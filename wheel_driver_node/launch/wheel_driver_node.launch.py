from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    # driver_dir_1 = os.path.join(get_package_share_directory('wheel_driver_node'), 'config', 'arm_driver_config.yaml')
    return LaunchDescription([
        Node(
            package="wheel_driver_node",
            executable="wheel_driver_node",
            name="wheel_driver_node",
            # parameters=[driver_dir_1],
        ),
    ])