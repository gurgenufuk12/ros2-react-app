# websocket.py
import json
import asyncio
import logging
from websockets import WebSocketServerProtocol
from ros import ROSBridge

logger = logging.getLogger(__name__)


class WebSocketManager:
    def __init__(self, ros_bridge):
        self.connected = set()
        self.topic_subscribers = {}
        self.topic_tasks = {}
        self.update_tasks = {}
        self.pose_subscribers = set()
        self.odom_subscribers = set()
        self.ros_bridge = ros_bridge
        self.update_pose_task = None
        self.update_odom_task = None

    async def handler(self, websocket: WebSocketServerProtocol):
        self.connected.add(websocket)
        try:
            await self.receive_messages(websocket)
        finally:
            self.connected.remove(websocket)
            self.pose_subscribers.discard(websocket)
            if not self.pose_subscribers and self.update_pose_task:
                self.update_pose_task.cancel()
                self.update_pose_task = None

    async def receive_messages(self, websocket):
        async for message in websocket:
            await self.process_message(websocket, message)

    async def process_message(self, websocket, message):
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            logger.error("Failed to decode message: %s", message)
            await websocket.send(json.dumps({"error": "Invalid JSON format"}))
            return

        msg_type = data.get("type")
        print(f"Received message: {msg_type}")

        if msg_type == "subscribe_topic":
            topic_name = data.get("topic")
            msg_type = data.get("msg_type")
            instance_id = data.get("instanceId")

            print(
                f"Subscribing instance {instance_id} to topic: {topic_name}")

            if not all([topic_name, msg_type, instance_id]):
                await websocket.send(json.dumps({
                    "error": "Missing required fields (topic, msg_type, or instanceId)"
                }))
                return

            success = self.ros_bridge.create_dynamic_subscription(
                topic_name, msg_type)
            print(f"Subscription success: {success}")

            if success:
                if topic_name not in self.topic_subscribers:
                    self.topic_subscribers[topic_name] = {}
                    self.topic_tasks[topic_name] = asyncio.create_task(
                        self.update_topic(topic_name)
                    )

                self.topic_subscribers[topic_name][instance_id] = websocket
                print(
                    f"Subscribed instance {instance_id} to topic: {topic_name}")

                await websocket.send(json.dumps({
                    "type": "subscribe_success",
                    "topic": topic_name,
                    "instanceId": instance_id
                }))
            else:
                await websocket.send(json.dumps({
                    "error": f"Failed to subscribe to topic: {topic_name}",
                    "instanceId": instance_id
                }))

        elif msg_type == "cmd_vel":
            self.ros_bridge.publish_cmd_vel(
                data.get("linear"), data.get("angular"))
            pass

        elif msg_type == "get_map":
            print(f"Received message: {msg_type}")
            map_data = self.ros_bridge.get_map_data()

            if map_data is not None:
                response = {
                    "type": "map_data",
                    "data": map_data
                }
                await websocket.send(json.dumps(response))
            else:
                print("Map data not available")
                await websocket.send(json.dumps({"error": "Map data not available"}))

            pass

        elif msg_type == "subscribe_pose":
            print("Client subscribed to pose updates")
            self.pose_subscribers.add(websocket)
            if not self.update_pose_task:
                self.update_pose_task = asyncio.create_task(self.update_pose())

        elif msg_type == "unsubscribe_pose":
            print("Client unsubscribed from pose updates")
            self.pose_subscribers.discard(websocket)
            if not self.pose_subscribers and self.update_pose_task:
                self.update_pose_task.cancel()
                self.update_pose_task = None

        elif msg_type == "get_topics":
            topics = self.ros_bridge.get_topic_list()
            response = {
                "type": "topic_list",
                "data": topics
            }
            await websocket.send(json.dumps(response))

        elif msg_type == "subscribe_odom":
            self.odom_subscribers.add(websocket)
            if not self.update_odom_task:
                self.update_odom_task = asyncio.create_task(self.update_odom())

        elif msg_type == "unsubscribe_odom":
            self.odom_subscribers.discard(websocket)
            if not self.odom_subscribers and self.update_odom_task:
                self.update_odom_task.cancel()
                self.update_odom_task = None

        elif msg_type == "unsubscribe_topic":
            topic_name = data.get("topic")
            instance_id = data.get("instanceId")
            if topic_name in self.topic_subscribers:
                if instance_id in self.topic_subscribers[topic_name]:
                    self.topic_subscribers[topic_name].pop(instance_id)
                    if not self.topic_subscribers[topic_name]:
                        self.topic_tasks[topic_name].cancel()
                        del self.topic_subscribers[topic_name]
                        del self.topic_tasks[topic_name]
                        print(f"Unsubscribed from topic: {topic_name}")
                    await websocket.send(json.dumps({
                        "type": "unsubscribe_success",
                        "topic": topic_name,
                        "instanceId": instance_id
                    }))
                else:
                    await websocket.send(json.dumps({
                        "error": f"Instance {instance_id} not subscribed to topic: {topic_name}"
                    }))
            else:
                await websocket.send(json.dumps({
                    "error": f"Topic {topic_name} not found"
                }))

    async def update_pose(self):
        logger.info("Pose update task started")
        try:
            while True:
                pose_data = self.ros_bridge.get_pose_data()
                if pose_data:
                    logger.debug(f"Got pose data: {pose_data}")
                    try:
                        message = {
                            "type": "pose_data",
                            "data": pose_data
                        }
                        await self.broadcast_pose(json.dumps(message))
                    except Exception as e:
                        logger.error(f"Error broadcasting pose: {e}")
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.info("Pose update task cancelled")
        except Exception as e:
            logger.error(f"Error in pose update task: {e}")

    async def broadcast_pose(self, message):
        logger.debug(
            f"Broadcasting to {len(self.pose_subscribers)} subscribers")
        if not self.pose_subscribers:
            logger.warning("No pose subscribers")
            return

        for ws in self.pose_subscribers.copy():
            try:
                await asyncio.sleep(0.5)
                await ws.send(message)
                logger.debug("Pose data sent successfully")
            except Exception as e:
                logger.error(f"Error sending to subscriber: {e}")
                self.pose_subscribers.remove(ws)

    async def update_odom(self):
        try:
            while True:
                odom_data = self.ros_bridge.get_odom_data()
                if odom_data:
                    message = {
                        "type": "odom_data",
                        "data": odom_data
                    }
                    await self.broadcast_odom(json.dumps(message))
                await asyncio.sleep(0.1)  # 10Hz update rate
        except asyncio.CancelledError:
            logger.info("Odom update task cancelled")
        except Exception as e:
            logger.error(f"Error in odom update: {e}")

    async def broadcast_odom(self, message):
        to_remove = set()
        for ws in self.odom_subscribers:
            try:
                await asyncio.sleep(0.5)
                await ws.send(message)
            except Exception as e:
                logger.error(f"Error sending odom: {e}")
                to_remove.add(ws)
        self.odom_subscribers -= to_remove

    async def update_topic(self, topic_name):
        try:
            while True:
                topic_data = self.ros_bridge.get_dynamic_data(topic_name)
                if topic_data is not None:
                    try:
                        message = {
                            "type": "topic_data",
                            "topic": topic_name,
                            "data": topic_data
                        }
                        message_json = json.dumps(message)
                        logger.info(f"Topic data prepared: {message_json}")
                        await self.broadcast_topic(topic_name, message_json)
                    except Exception as e:
                        logger.error(f"Error preparing topic data: {e}")
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.info(f"Topic update cancelled for {topic_name}")
            pass

    async def broadcast_topic(self, topic_name, message):
        # print(f"Broadcasting to topic: {topic_name}")
        if topic_name not in self.topic_subscribers:
            return
        formatted_message = {
            "type": "topic_data",
            "topic": topic_name,
            "data": json.loads(message)
        }

        message_json = json.dumps(formatted_message)
        to_remove = []

        for instance_id, ws in self.topic_subscribers[topic_name].items():
            try:
                await ws.send(message_json)
                # print(f"Sent data to instance {instance_id}")
            except Exception as e:
                logger.error(f"Error sending to instance {instance_id}: {e}")
                to_remove.append(instance_id)

        for instance_id in to_remove:
            del self.topic_subscribers[topic_name][instance_id]
            print(f"Removed instance {instance_id} from topic {topic_name}")

        if not self.topic_subscribers[topic_name]:
            if topic_name in self.topic_tasks:
                self.topic_tasks[topic_name].cancel()
                del self.topic_tasks[topic_name]
            del self.topic_subscribers[topic_name]
            print(f"Removed topic {topic_name} - no subscribers left")
