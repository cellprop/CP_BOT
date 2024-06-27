import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import pandas as pd
import math

class RoadFollower(Node):
    def __init__(self):
        super().__init__('road_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.waypoints = self.read_waypoints('/home/vihaan/dev_ws/src/CP_BOT/src/Location.csv')
        self.current_position = self.waypoints[0]  # Set initial position to the first waypoint
        self.current_orientation = 0.0
        self.current_waypoint_index = 1  # Start from the second waypoint

    def read_waypoints(self, file_path):
        data = pd.read_csv(file_path, skiprows=1)  # Skip the first row
        latitudes = data[data.columns[1]].values  # Ensure to read the second column for latitude
        longitudes = data[data.columns[0]].values  # Ensure to read the first column for longitude

        # Normalize coordinates using min-max scaling
        lat_min, lat_max = latitudes.min(), latitudes.max()
        lon_min, lon_max = longitudes.min(), longitudes.max()

        # Scale coordinates to fit within a defined Gazebo area (e.g., 100x100 meters)
        x_coords = (longitudes - lon_min) / (lon_max - lon_min) * 100
        y_coords = (latitudes - lat_min) / (lat_max - lat_min) * 100

        waypoints = list(zip(x_coords, y_coords))
        return waypoints

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        )
        orientation_q = msg.pose.pose.orientation
        self.current_orientation = self.quaternion_to_euler(orientation_q)
        #print("Current position: ", self.current_position)

    def quaternion_to_euler(self, q):
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
            if distance > 0.1:  # Increase the threshold to 0.5 meters
                twist.linear.x = 0.1
                twist.angular.z = self.angle_diff(angle_to_target, self.current_orientation)
            else:
                self.current_waypoint_index += 1  # Move to the next waypoint
                
                twist.linear.x = 0.0  # Stop when reaching the last waypoint
                twist.angular.z = 0.0
        self.publisher_.publish(twist)
        print("current waypoint: ", self.current_waypoint_index, self.waypoints[self.current_waypoint_index])

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
