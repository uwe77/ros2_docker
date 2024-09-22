#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

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

        # Publishers for the port and starboard motor thrust commands
        self.port_motor_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_port_joint/cmd_thrust', 10)
        self.stbd_motor_publisher = self.create_publisher(Float64, '/model/blueboat/joint/motor_stbd_joint/cmd_thrust', 10)

        # Variable to hold the axes and button values from the joystick
        self.axes = []
        self.buttons = []

    def joy_callback(self, msg):
        # Update axes and buttons based on joystick input
        self.axes = msg.axes
        self.buttons = msg.buttons

        # Get linear and angular velocities from joystick axes (map as needed)
        linear_speed = self.axes[1]  # Assuming forward/backward on axis 1
        angular_speed = self.axes[0]  # Assuming turning on axis 0

        # Calculate thrust for each motor
        port_thrust = Float64()
        stbd_thrust = Float64()

        # Simple differential drive logic: adjust thrust based on linear and angular speeds
        port_thrust.data = (linear_speed - angular_speed) * 70  # scale by motor multiplier
        stbd_thrust.data = (linear_speed + angular_speed) * 70  # scale by motor multiplier

        # Publish thrust commands for both motors
        self.port_motor_publisher.publish(port_thrust)
        self.stbd_motor_publisher.publish(stbd_thrust)

        # Log the values for debugging
        self.get_logger().info(f'Port Thrust: {port_thrust.data}, Starboard Thrust: {stbd_thrust.data}')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Check if args is None and handle it gracefully
    if args is None:
        args = []

    # Extract group_name from arguments (if provided)
    group_name = ''
    if len(args) > 1:
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
