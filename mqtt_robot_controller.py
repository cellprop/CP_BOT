import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MqttRobotController(Node):
    def __init__(self):
        super().__init__('mqtt_robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect("localhost", 1883, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT broker")
        client.subscribe("robot/control")

    def on_message(self, client, userdata, msg):
        command = msg.payload.decode()
        twist = Twist()

        if command == "forward":
            twist.linear.x = 1.0  # Increase forward speed
            twist.angular.z = 0.0
        elif command == "backward":
            twist.linear.x = -1.0  # Increase backward speed
            twist.angular.z = 0.0
        elif command == "left":
            twist.linear.x = 0.0
            twist.angular.z = 1.0  # Increase turn left speed
        elif command == "right":
            twist.linear.x = 0.0
            twist.angular.z = -1.0  # Increase turn right speed
        elif command == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MqttRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
