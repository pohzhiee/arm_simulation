import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get gazebo_ros package path
    gazebo_ros_share_path = get_package_share_directory('gazebo_ros')
    lobot_desc_share_path = get_package_share_directory('lobot_description')
    sim_share_path = get_package_share_directory('arm_simulation')

    gym_zero_g_world_path = os.path.join(lobot_desc_share_path, "worlds/gym_zero_g_world.sdf")
    zero_g_world_path = os.path.join(lobot_desc_share_path, "worlds/zero_g_world.sdf")
    cmd = ['"', gym_zero_g_world_path, '"', ' if "', LaunchConfiguration('gym'),   '" == "True" ',
           'else ', '"', zero_g_world_path, '"']
    # The expression above expression is evaluated to something like this:
    # <some_path>/gym_zero_g_world.sdf if "<LaunchConfig evaluation result>" == "True" else <some_path>/zero_g_world.sdf
    py_cmd = PythonExpression(cmd)

    # Launch gzserver
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_share_path, 'launch',
                                                   'gzserver.launch.py')),
        launch_arguments={'extra_gazebo_args': '__log_level:=info', 'pause': 'true', 'world': py_cmd}.items())

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    # spawn_entity = Node(package='gazebo_ros', node_executable='spawn_entity.py',
    #                    arguments=['-entity', 'arm_standalone','-file',arm_urdf_path],
    #                    output='screen')

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_share_path, 'launch',
                                                   'spawn_entity.launch.py')),
    )
    # Launch gzclient
    gzclient = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_path, 'launch','gzclient.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')))

    return LaunchDescription([
        DeclareLaunchArgument('gym', default_value='False',
                              description='Bool to specify if the simulation is to be ran with openai gym training'),
        DeclareLaunchArgument('gui', default_value='True',
                              description='Bool to specify if gazebo should be launched with GUI or not'),
        gzserver,
        spawn_entity,
        gzclient,
    ])
