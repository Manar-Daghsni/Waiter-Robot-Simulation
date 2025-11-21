from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_my = get_package_share_directory('my_worlds')
    world = os.path.join(pkg_my, 'worlds', 'restaurant.world')

    # include the existing turtlebot3_world launch, but override the world path
    tb3_launch = os.path.join(pkg_tb3, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb3_launch),
            launch_arguments={'world': world}.items()
        )
    ])
