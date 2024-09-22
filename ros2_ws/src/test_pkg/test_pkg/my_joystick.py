#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickController(Node):
    def __init__(self, group_name=''):
        # Initialize node with namespace (group_name) if provided
        super().__init__('joystick_controller', namespace=group_name)

        # Subscriber to the /joy topic (this will be under /group_name/joystick_controller/joy)
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',  # Topic will be prefixed with the namespace
            self.joy_callback,
            10
        )
        self.joy_subscription  # prevent unused variable warning

        # Publisher to control robot movement (this will be under /group_name/joystick_controller/cmd_vel)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Variable to hold the axes and button values from the joystick
        self.axes = []
        self.buttons = []

    def joy_callback(self, msg):
        # Update axes and buttons based on joystick input
        self.axes = msg.axes
        self.buttons = msg.buttons

        # Create a Twist message for robot movement
        twist = Twist()

        # Map joystick axes to robot control (for example, axis 1 for linear speed and axis 0 for angular velocity)
        linear_speed = self.axes[1] * 1.0  # Scale this value based on your robot's needs
        angular_speed = self.axes[0] * 1.0  # Scale this value as needed

        twist.linear.x = linear_speed
        twist.angular.z = angular_speed

        # Publish the Twist message to control the robot
        self.cmd_vel_publisher.publish(twist)

        # Log the joystick state
        self.get_logger().info(f'Linear: {linear_speed}, Angular: {angular_speed}')


def main(args=None):
    rclpy.init(args=args)

    # Initialize group_name with a default value if args is None
    group_name = ''
    
    if args is not None and len(args) > 1:
        group_name = args[1]  # Assuming the second argument is the group name

    # Create the joystick controller node with the namespace (group_name)
    joystick_controller = JoystickController(group_name)

    rclpy.spin(joystick_controller)

    # Shutdown once we're done
    joystick_controller.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    import sys
    main(args=sys.argv)
