from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the `gz_sim.launch.py` in the `ros_gz_sim` package
    gz_launch_file_dir = get_package_share_directory('ros_gz_sim')
    gz_launch_file = os.path.join(gz_launch_file_dir, 'launch', 'gz_sim.launch.py')

    # Define the 'gz_args' launch argument (empty.sdf world)
    gz_args = LaunchConfiguration('gz_args', default='empty.sdf')

    return LaunchDescription([
        # Include the `gz_sim.launch.py` launch file with the `gz_args` argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_file),
            launch_arguments={'gz_args': gz_args}.items(),
        ),
    ])
