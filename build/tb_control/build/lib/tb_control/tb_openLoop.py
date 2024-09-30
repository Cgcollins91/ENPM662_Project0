import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TB_Openloop(Node):

    def __init__(self):
        super().__init__('tb_openloop')

        # Initialize Variables
        self.target_velocity  = 0.5  # Target velocity in m/s
        self.current_velocity = 0.0  # Current velocity
        self.current_x        = 0.0  # Current position x
        self.start_coast_x    = 0.0  # Position at the start of coasting
        self.coast_delta_x    = 2.0  # Distance to coast in meters

        # State Variables
        self.accelerating = True   # Start with accelerating
        self.coasting     = False  # Not coasting yet
        self.decelerating = False  # Not decelerating yet

        # Publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to listen to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Create a timer to regularly publish velocity
        self.create_timer(0.1, self.publish_velocity)

        self.get_logger().info('TB_Openloop node started')

    def odom_callback(self, msg):
        # Update current x position and velocity from odometry
        self.current_x        = msg.pose.pose.position.x
        self.current_velocity = msg.twist.twist.linear.x

        # Log the current velocity and position
        self.get_logger().info(f'Odom - Position x: {self.current_x}, Velocity: {self.current_velocity}')
        self.get_logger().info(f'accelerating: {self.accelerating}, decelerating: {self.decelerating}, coasting:{self.coasting}')
     

        # Start accelerating until target_velocity is met
        if self.accelerating:
            if self.current_velocity < self.target_velocity:
                self.accelerate_turtle()

            else: # Target Velocity met, save x position when target veolcity is met
                self.start_coast_x = self.current_x  # Save current x as start of coasting
                self.accelerating = False            # No Longer accelerating
                self.coasting = True                 # Start Coast Phase
                self.get_logger().info('Starting coast phase')

        elif self.coasting: # Coast until delta_x over coast phase is traversed
            if self.current_x < self.start_coast_x + self.coast_delta_x:
                self.coast_turtle()
            else:
                self.coasting     = False  # End Coast Phase
                self.decelerating = True   # Start deceleration phase
                self.get_logger().info('Starting deceleration phase')

        elif self.decelerating:
            self.decelerate_turtle()

    def publish_velocity(self):
        # Publishes velocity based on the current phase
        if self.accelerating:
            self.accelerate_turtle()
        elif self.coasting:
            self.coast_turtle()
        elif self.decelerating:
            self.decelerate_turtle()

    def accelerate_turtle(self):
        # Create a Twist message to accelerate
        velocity_msg          = Twist()
        velocity_msg.linear.x = min(self.current_velocity + 0.1, self.target_velocity)  # Increment velocity
        self.publisher_.publish(velocity_msg)
        self.get_logger().info(f'Accelerating to target velocity: {velocity_msg.linear.x}')

    def coast_turtle(self):
        # Maintain constant velocity during coasting
        velocity_msg          = Twist()
        velocity_msg.linear.x = self.target_velocity  # Maintain target velocity
        self.publisher_.publish(velocity_msg)
        self.get_logger().info('Coasting at constant velocity')

    def decelerate_turtle(self):
        # Reduce velocity during deceleration
        velocity_msg          = Twist()
        velocity_msg.linear.x = max(self.current_velocity - 0.1, 0.0)  # Decrease velocity gradually
        self.publisher_.publish(velocity_msg)
        self.get_logger().info(f'Decelerating to stop: {velocity_msg.linear.x}')


def main(args=None):
    rclpy.init(args=args)
    tb_openloop = TB_Openloop()
    rclpy.spin(tb_openloop)
    tb_openloop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
