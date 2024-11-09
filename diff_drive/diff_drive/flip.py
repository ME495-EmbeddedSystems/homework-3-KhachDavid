import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist


def flip_entry():
    """Entry point for the flip node."""
    rclpy.init()
    node = Flip()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()


class Flip(Node):
    """Node for flipping the robot."""

    def __init__(self):
        """Initialize the flip node."""
        super().__init__('flip')
        self.get_logger().info('Flip node started')

        # publisher to cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Set the flip parameters
        self.flip_velocity = 12.0
        self.flip_duration = 0.4
        self.rest_duration = 0.5

        # Create the timer
        self.timer = self.create_timer(self.flip_duration, self.perform_flip)

    def perform_flip(self):
        """Perform the flip."""
        # Step 1: Accelerate forward
        self.move_robot(self.flip_velocity)
        time.sleep(self.flip_duration)

        # Step 2: Stop briefly
        self.stop_robot()
        time.sleep(self.rest_duration)

        # Step 3: Accelerate backward to flip
        self.move_robot(-self.flip_velocity)
        time.sleep(self.flip_duration)

        # Step 4: Accelarate in random direction
        self.move_random()
        time.sleep(self.flip_duration)

        # Step 4: Stop again briefly
        self.stop_robot()
        time.sleep(self.rest_duration)

    def move_robot(self, velocity):
        """Move the robot with the given linear velocity."""
        twist = Twist()
        twist.linear.x = velocity
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

    def move_random(self):
        """Move the robot in a random direction."""
        twist = Twist()
        twist.linear.x = self.flip_velocity
        twist.angular.z = self.flip_velocity
        self.publisher.publish(twist)


if __name__ == '__main__':
    flip_entry()
