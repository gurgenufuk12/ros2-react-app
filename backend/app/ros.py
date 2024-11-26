import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import json

class ROSBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.map_data = None

    def map_callback(self, msg):
        self.map_data = {
            "width": msg.info.width,
            "height": msg.info.height,
            "data": list(msg.data)
        }

    def publish_cmd_vel(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)

    def get_map_data(self):
        return json.dumps(self.map_data)
