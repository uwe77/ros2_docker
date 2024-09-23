from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction

def generate_launch_description():
    # Declare a launch argument for the group (namespace)
    group_name = LaunchConfiguration('group_name')

    return LaunchDescription([
        # Declare a launch argument to accept group_name from the command line
        DeclareLaunchArgument(
            'group_name',
            default_value='racecar',
            description='Namespace for the nodes'
        ),

        # Group the joystick_controller and joy_node under a specific namespace
        GroupAction([
            # Push the namespace (e.g., robot1)
            PushRosNamespace(group_name),

            # Launch the joystick_controller node
            Node(
                package='test_pkg',
                executable='joy_car',
                name='joy_car',
                output='screen'
            ),

            # Launch the joy_node (which listens to joystick input)
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen'
            ),
        ])
    ])
