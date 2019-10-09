import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get gazebo_ros package path
    gazebo_ros_share_path = get_package_share_directory('gazebo_ros')
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    zero_g_world_path = os.path.join(lobot_desc_share_path, 'worlds/zero_g_world.sdf')
    # Launch gzserver
    gzserver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_path, 'launch',
                    'gzserver.launch.py')),
        launch_arguments={'extra_gazebo_args': '__log_level:=info','world': zero_g_world_path, 'pause': 'true'}.items())

    # Launch gzclient
    gzclient = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_path, 'launch',
                    'gzclient.launch.py')),
             )
    return LaunchDescription([
        gzserver,
        gzclient
    ])
