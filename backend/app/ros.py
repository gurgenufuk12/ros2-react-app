import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import json
import logging
from rosidl_runtime_py import message_to_ordereddict
logger = logging.getLogger(__name__)


class ROSBridge(Node):
    def __init__(self):
        super().__init__('ros_bridge')
        self.dynamic_subscribers = {}
        self.dynamic_data = {}
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )
        # self.cmd_vel_publisher = self.create_publisher(
        #     Twist,
        #     '/cmd_vel',
        #     10
        # )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile=qos
        )
        self.amcl_pose = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            qos_profile=qos
        )
        self.odom_message = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.odom_data = None
        self.map_data = None
        self.pose_data = None
        self.topic_list = None

        print(f"ROSBridge initialized, waiting for map data...")

    def map_callback(self, msg):
        print(
            f"Map callback received! Dimensions: {msg.info.width}x{msg.info.height}")
        self.map_data = {
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "origin": {
                "position": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                },
                "orientation": {
                    "x": msg.info.origin.orientation.x,
                    "y": msg.info.origin.orientation.y,
                    "z": msg.info.origin.orientation.z,
                    "w": msg.info.origin.orientation.w,
                },
            },
            "data": list(msg.data)
        }

    def amcl_callback(self, msg):
        print(f"AMCL callback received!")
        self.pose_data = {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w,
                },
            },
            "covariance": list(msg.pose.covariance)
        }

    def publish_cmd_vel(self, linear, angular):
        # twist = Twist()

        # twist.linear.x = float(linear.get('x', 0.0))
        # twist.linear.y = float(linear.get('y', 0.0))
        # twist.linear.z = float(linear.get('z', 0.0))

        # twist.angular.x = float(angular.get('x', 0.0))
        # twist.angular.y = float(angular.get('y', 0.0))
        # twist.angular.z = float(angular.get('z', 0.0))

        # self.cmd_vel_publisher.publish(twist)
        # logger.info(f"Published Twist: {twist}")
        twist_stamped = TwistStamped()

        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"

        twist_stamped.twist.linear.x = float(linear.get('x', 0.0))
        twist_stamped.twist.linear.y = float(linear.get('y', 0.0))
        twist_stamped.twist.linear.z = float(linear.get('z', 0.0))

        twist_stamped.twist.angular.x = float(angular.get('x', 0.0))
        twist_stamped.twist.angular.y = float(angular.get('y', 0.0))
        twist_stamped.twist.angular.z = float(angular.get('z', 0.0))

        self.cmd_vel_publisher.publish(twist_stamped)
        logger.info(f"Published TwistStamped: {twist_stamped}")

    def odom_callback(self, msg):
        self.odom_data = {
            "pose": {
                "position": {
                    "x": msg.pose.pose.position.x,
                    "y": msg.pose.pose.position.y,
                    "z": msg.pose.pose.position.z,
                },
                "orientation": {
                    "x": msg.pose.pose.orientation.x,
                    "y": msg.pose.pose.orientation.y,
                    "z": msg.pose.pose.orientation.z,
                    "w": msg.pose.pose.orientation.w,
                },
            },
            "covariance": list(msg.pose.covariance),
            "twist": {
                "linear": {
                    "x": msg.twist.twist.linear.x,
                    "y": msg.twist.twist.linear.y,
                    "z": msg.twist.twist.linear.z,
                },
                "angular": {
                    "x": msg.twist.twist.angular.x,
                    "y": msg.twist.twist.angular.y,
                    "z": msg.twist.twist.angular.z,
                },
            },
            "covariance": list(msg.pose.covariance),
        }

    def create_dynamic_subscription(self, topic_name, msg_type):
        if topic_name in self.dynamic_subscribers:
            self.destroy_subscription(self.dynamic_subscribers[topic_name])
            del self.dynamic_subscribers[topic_name]
            print(f"Removed existing subscription for {topic_name}")

        try:
            msg_module = __import__(msg_type.split('/')[0] + '.msg',
                                    fromlist=[msg_type.split('/')[-1]])
            msg_class = getattr(msg_module, msg_type.split('/')[-1])

            print(f"Creating subscription for {topic_name}")
            self.dynamic_subscribers[topic_name] = self.create_subscription(
                msg_class,
                topic_name,
                lambda msg: self.dynamic_callback(topic_name, msg),
                QoSProfile(depth=10)
            )
            return True
        except Exception as e:
            print(f"Error creating subscription: {e}")
            return False

    def dynamic_callback(self, topic_name, msg):
        self.dynamic_data[topic_name] = message_to_ordereddict(msg)

    def get_dynamic_data(self, topic_name):
        return self.dynamic_data.get(topic_name)

    def get_odom_data(self):
        return self.odom_data

    def get_map_data(self):
        return self.map_data

    def get_pose_data(self):
        return self.pose_data

    def get_topic_list(self):
        self.topic_list = []
        for name, types in self.get_topic_names_and_types():
            self.topic_list.append({
                "name": name,
                "type": types[0]
            })
        return self.topic_list
