from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction  # Use from launch.actions, not launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        # Group all nodes under 'wamv'
        GroupAction(
            actions=[
                # Launch joy node for joystick input
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                ),

                # Launch joystick controller node
                Node(
                    package='test_pkg',
                    executable='my_joystick',
                    name='my_joystick',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                ),
            ],
        ),
    ])
