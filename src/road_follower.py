import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import csv

class RoadFollower(Node):
    def __init__(self):
        super().__init__('road_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_position = (0.0, 0.0, 0.0)
        self.current_orientation = 0.0
        self.waypoints = self.read_waypoints('/home/your_username/dev_ws/src/CP_BOT/src/Location.csv')
        self.current_waypoint_index = 0

    def read_waypoints(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # Skip header
            for row in reader):
                waypoints.append((float(row[0]), float(row[1]), float(row[2])))
        return waypoints

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)

    def quaternion_to_euler(self, q):
        # Convert quaternion to euler angles (yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def control_loop(self):
        twist = Twist()

        if self.current_waypoint_index < len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index]
            distance = self.distance_to_target(target)
            angle_to_target = self.angle_to_target(target)

            if distance > 0.2:  # Continue moving towards the waypoint
                twist.linear.x = 0.5
                twist.angular.z = self.angle_diff(angle_to_target, self.current_orientation)
            else:
                self.current_waypoint_index += 1  # Move to the next waypoint

        self.publisher_.publish(twist)

    def distance_to_target(self, target):
        return math.sqrt((target[0] - self.current_position[0])**2 + (target[1] - self.current_position[1])**2)

    def angle_to_target(self, target):
        return math.atan2(target[1] - self.current_position[1], target[0] - self.current_position[0])

    def angle_diff(self, target_angle, current_angle):
        a = target_angle - current_angle
        return (a + math.pi) % (2 * math.pi) - math.pi

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