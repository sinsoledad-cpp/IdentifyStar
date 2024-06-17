import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('stars_serial_driver'), 'config', 'serial_driver.yaml')

    rm_serial_driver_node = Node(
        package='stars_serial_driver',
        executable='stars_serial_driver_node',
        namespace='',
        output='both',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([rm_serial_driver_node])
