import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable

def generate_launch_description():
    home = os.environ.get('HOME', '')
    repo = os.environ.get('REPO_NAME', '')
    path = os.path.join(home, repo, 'ros2_ws', 'src', 'test_pkg', 'config', 'waves_wamv.yaml')
    print(path)
    print(path)
    return LaunchDescription([
        # Launch Gazebo Sim (Ignition) with the waves world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': 'waves.sdf'}.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            output='screen',
            parameters=[{'config_file':path}]
        ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=['/model/wamv-v/joint/left_propeller_joint/cmd_thrust@/arg_wamv/left_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'],
        #     output='screen'
        # ),

    ])
