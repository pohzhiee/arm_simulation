import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkgShareDir  = get_package_share_directory('arm_simulation')
    configDir = os.path.join(pkgShareDir, 'config', 'params.yaml')
    node1 = launch_ros.actions.Node(
        package='ros2_control_parameter_server', node_executable='param_server_exec', arguments=[configDir, "__log_level:=info"], output='screen')
    return launch.LaunchDescription([node1])
