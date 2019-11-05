import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Note: when spawning the entity, the name is extremely important as this will affect the pid controller
def generate_launch_description():
    # Get gazebo_ros package path
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    arm_urdf_path = os.path.join(lobot_desc_share_path,'robots/arm_standalone.urdf')
    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
                        arguments=['-entity', 'arm','-file',arm_urdf_path],
                        output='screen')
    return LaunchDescription([
        spawn_entity
    ])
