import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

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

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True',
                              description='Bool to specify if gazebo should be launched with GUI or not'),
        gzserver,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_path, 'launch',
                    'gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui'))
        ),
    ])
