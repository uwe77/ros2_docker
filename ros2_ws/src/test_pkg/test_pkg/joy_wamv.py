import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

class WamvThrustJoystickControl(Node):
    def __init__(self):
        super().__init__('wamv_thrust_joystick_control')

        # Publishers to left and right propeller thrust command topics
        self.left_thruster_publisher = self.create_publisher(Float64, '/arg_wamv/left_propeller_joint/cmd_thrust', 10)
        self.right_thruster_publisher = self.create_publisher(Float64, '/arg_wamv/right_propeller_joint/cmd_thrust', 10)

        # Subscriber to joystick inputs
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # Initialize variables for the thrusters
        self.left_thrust = Float64()
        self.right_thrust = Float64()

    def joy_callback(self, joy_msg):
        # Map joystick axes to thrust values for the left and right propellers
        # Left stick vertical (axis 1) for left propeller thrust
        # Right stick vertical (axis 4) for right propeller thrust

        self.left_thrust.data = joy_msg.axes[1] * 1500  # Scale based on max thrust
        self.right_thrust.data = joy_msg.axes[3] * 1500

        # Publish the thrust commands
        self.left_thruster_publisher.publish(self.left_thrust)
        self.right_thruster_publisher.publish(self.right_thrust)

        # Log the output for debugging
        self.get_logger().info(f'Left Thrust: {self.left_thrust.data}, Right Thrust: {self.right_thrust.data}')

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the joystick control node
    wamv_thrust_control = WamvThrustJoystickControl()

    try:
        rclpy.spin(wamv_thrust_control)
    except KeyboardInterrupt:
        pass
    finally:
        wamv_thrust_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
