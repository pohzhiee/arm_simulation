import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.actions import UnsetEnvironmentVariable

# Note: when spawning the entity, the name is extremely important as this will affect the pid controller
def generate_launch_description():
    # Get gazebo_ros package path
    gazebo_ros_share_path = get_package_share_directory('gazebo_ros')
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    arm_urdf_path = os.path.join(lobot_desc_share_path,'robots/arm_standalone.urdf')
    zero_g_world_path = os.path.join(lobot_desc_share_path, 'worlds/zero_g_world.sdf')
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'arm','-file',arm_urdf_path],
                        output='screen')
    return LaunchDescription([
        spawn_entity
    ])
