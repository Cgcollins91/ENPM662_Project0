import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # To control velocity
from nav_msgs.msg import Odometry    # For receiving odometry data
import time

class TBOpenLoop(Node):

    def __init__(self):
        super().__init__('tb_openloop')

        # Publisher to the '/cmd_vel' topic to send velocity commands to TurtleBot3
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Optional: Subscriber to the '/odom' topic to receive odometry data
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.subscription  # prevent unused variable warning

        # Initialize the timer to repeatedly publish velocity commands every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_velocity)

        # Variables to track the current and target velocities
        self.target_velocity = 0.5  # Set a target forward velocity in m/s
        self.current_velocity = 0.0
        self.current_x = 0.0  # To track the robot's position in the x direction

    def publish_velocity(self):
        """Publish velocity commands to move the TurtleBot3."""
        # Create a new velocity message
        velocity_msg = Twist()
        
        # Set forward linear velocity and no rotation
        velocity_msg.linear.x = self.target_velocity
        velocity_msg.angular.z = 0.0  # No rotation, move straight

        # Log the velocity for debugging
        self.get_logger().info(f'Publishing velocity: linear.x = {self.target_velocity}')

        # Publish the velocity command to the '/cmd_vel' topic
        self.publisher_.publish(velocity_msg)

    def odom_callback(self, msg):
        """Callback to process odometry data from the '/odom' topic."""
        # Extract the robot's position (x, y, z) and linear velocity
        self.current_x = msg.pose.pose.position.x
        self.current_velocity = msg.twist.twist.linear.x

        # Log the robot's current position and velocity for debugging
        self.get_logger().info(f'Odom - Position x: {self.current_x}, Velocity: {self.current_velocity}')

def main(args=None):
    """Main function to initialize the ROS node and keep it running."""
    rclpy.init(args=args)

    tb_openloop = TBOpenLoop()
    rclpy.spin(tb_openloop)

    # Cleanup before exiting
    tb_openloop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
