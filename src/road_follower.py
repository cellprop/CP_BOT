import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RoadFollower(Node):
    def __init__(self):
        super().__init__('road_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, 'scan', self.laser_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.left_distance = 0.0
        self.right_distance = 0.0

    def laser_callback(self, msg):
        # Assuming the Lidar has 360 degrees range and we're interested in the left and right sides
        self.left_distance = min(min(msg.ranges[0:90]), 10.0)  # 0 to 90 degrees
        self.right_distance = min(min(msg.ranges[270:360]), 10.0)  # 270 to 360 degrees

    def control_loop(self):
        twist = Twist()

        # Basic proportional control
        if self.left_distance < 1.0:
            twist.angular.z = -0.5  # Turn right
        elif self.right_distance < 1.0:
            twist.angular.z = 0.5  # Turn left
        else:
            twist.linear.x = 0.5  # Move forward

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = RoadFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
