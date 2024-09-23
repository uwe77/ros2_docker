import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self, group_name=''):
        super().__init__('cmd_vel_publisher', namespace=group_name)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',  # Topic will be prefixed with the namespace
            self.joy_callback,
            10
        )
        self.axes = []
        self.buttons = []

    def joy_callback(self, msg):
        # Update axes and buttons based on joystick input
        self.axes = msg.axes
        self.buttons = msg.buttons

        # Get linear and angular velocities from joystick axes (map as needed)
        linear_speed = self.axes[1]  # Assuming forward/backward on axis 1
        angular_speed = self.axes[3]  # Assuming turning on axis 0

        twist = Twist()
        twist.linear.x = linear_speed*60
        twist.angular.z = angular_speed*60

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    if args is None:
        args = []

    # Extract group_name from arguments (if provided)
    group_name = ''
    if len(args) > 1:
        group_name = args[1]  # Assuming the second argument is the group name

    # Create the joystick controller node with the namespace (group_name)
    node = CmdVelPublisher(group_name)

    rclpy.spin(node)

    # Shutdown once we're done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
