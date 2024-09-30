import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # Change to turtlesim.msg.Pose if using Turtlesim

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            Odometry,  # Change to turtlesim.msg.Pose if using Turtlesim
            '/odom',  # Change to /turtle1/pose if using Turtlesim
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        position = msg.pose.pose.position  # Extract position from odometry
        self.get_logger().info(f'Position - x: {position.x}, y: {position.y}')

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
