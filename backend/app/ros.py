import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import json
import logging
logger = logging.getLogger(__name__)


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
        try:
            # Explicitly convert to float
            twist.linear.x = float(linear['x'])
            twist.linear.y = float(linear['y'])
            twist.linear.z = float(linear['z'])
            twist.angular.x = float(angular['x'])
            twist.angular.y = float(angular['y'])
            twist.angular.z = float(angular['z'])
        except (KeyError, TypeError, ValueError) as e:
            logger.error(f"Invalid data format: {e}")
            return

        self.cmd_vel_publisher.publish(twist)
        logger.info(f"Published cmd_vel: {twist}")

    def get_map_data(self):
        return json.dumps(self.map_data)
