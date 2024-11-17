from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('group')
    world_path = os.path.join(package_share_directory, 'worlds', 'my_world.wbt')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['webots', '--mode=realtime', world_path],
            output='screen'
        )
    ])

