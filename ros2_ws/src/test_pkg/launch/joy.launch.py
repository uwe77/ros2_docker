from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # Declare the launch argument for the group (namespace)
    group_name = LaunchConfiguration('group_name')

    return LaunchDescription([
        # Declare a launch argument to accept group_name from the command line
        DeclareLaunchArgument(
            'group_name',
            default_value='robot1',
            description='Namespace for the nodes'
        ),
        
        # Group the nodes under a specific namespace
        GroupAction([
            # Push the namespace (e.g., robot1)
            PushRosNamespace(group_name),

            # Launch the joystick_controller node
            Node(
                package='test_pkg',
                executable='my_joystick',
                name='my_joystick',
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
