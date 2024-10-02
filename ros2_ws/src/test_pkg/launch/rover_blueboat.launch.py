from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for a wildthumper rover."""
    pkg_project_bringup = get_package_share_directory("arg_bringup")
    pkg_project_gazebo = get_package_share_directory("ardupilot_gz_gazebo")
    pkg_model = get_package_share_directory("ardupilot_sitl_models")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    declare_gz_sim_resource_path = DeclareLaunchArgument(
        name='GZ_SIM_RESOURCE_PATH',
        default_value='/home/arg/ros2_gazebo/SITL_Models/Gazebo/models',
        description='Path to the Gazebo Sim resource directory'
    )
    gz_sim_resource_path = LaunchConfiguration("GZ_SIM_RESOURCE_PATH")
    world_file_path = PathJoinSubstitution([gz_sim_resource_path, 'waves.sdf'])

    # Wild Thumper rover.
    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("arg_bringup"),
                        "launch",
                        "blueboat.launch.py",
                    ]
                ),
            ]
        )
    )

    # Gazebo.
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            "gz_args": "-v4 -s -r "
            f'/home/arg/ros2_gazebo/Gazebo/worlds/blueboat_waves.sdf'
        }.items(),
    )

    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={"gz_args": "-v4 -g"}.items(),
    )

    # RViz.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", f'{Path(pkg_project_bringup) / "rviz" / "blueboat.rviz"}'],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Open RViz."
            ),
            gz_sim_server,
            gz_sim_gui,
            rover,
            rviz,
        ]
    )