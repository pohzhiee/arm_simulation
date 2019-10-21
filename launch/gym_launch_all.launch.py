import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    # Get gazebo_ros package path
    sim_share_path = get_package_share_directory('arm_simulation')
    # Launch param server
    params_server = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sim_share_path, 'launch',
                    'params_server.launch.py')),
             )
    # Launch arm spawner
    arm_spawner = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sim_share_path, 'launch',
                    'gazebo_spawn_arm.launch.py')),
             )
    # Launch forward kinematics service
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    arm_urdf_path = os.path.join(lobot_desc_share_path,'robots/arm_standalone.urdf')

    fk_node = launch_ros.actions.Node(
        package='arm_fk_cpp', node_executable='arm_fk_exec', arguments=[arm_urdf_path, "__log_level:=info"], output='screen')

    return LaunchDescription([
        params_server,
        arm_spawner,
        fk_node
    ])
