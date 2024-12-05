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
        # Register client
        self.connected.add(websocket)
        try:
            await self.receive_messages(websocket)
        finally:
            # Unregister client
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
            print("Subscribing to topic")
            topic_name = data.get("topic")
            msg_type = data.get("msg_type")
            print(topic_name, msg_type)
            if not topic_name or not msg_type:
                await websocket.send(json.dumps({
                    "error": "Missing topic name or message type"
                }))
                return

            success = self.ros_bridge.create_dynamic_subscription(
                topic_name, msg_type)
            print(f"Subscription success: {success}")
            if success:
                if topic_name not in self.topic_subscribers:
                    self.topic_subscribers[topic_name] = set()
                    self.topic_tasks[topic_name] = asyncio.create_task(
                        self.update_topic(topic_name)
                    )
                    self.topic_subscribers[topic_name].add(websocket)
                    logger.info(f"Subscribed to topic: {topic_name}")
                else:
                    await websocket.send(json.dumps({
                        "error": f"Failed to subscribe to topic: {topic_name}"
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
            print(
                f"Received unsubscribe request for topic: {topic_name}")

            if topic_name in self.topic_subscribers:
                self.topic_subscribers[topic_name].discard(websocket)
                print(f"Removed subscriber for topic: {topic_name}")

                if not self.topic_subscribers[topic_name]:
                    if topic_name in self.topic_tasks:
                        self.topic_tasks[topic_name].cancel()
                        await self.topic_tasks[topic_name]
                        del self.topic_tasks[topic_name]
                    del self.topic_subscribers[topic_name]
                    print(
                        f"Cleaned up resources for topic: {topic_name}")

                await websocket.send(json.dumps({
                    "type": "unsubscribe_success",
                    "topic": topic_name
                }))
                print(
                    f"Sent unsubscribe confirmation for topic: {topic_name}")
            else:
                print(
                    f"Unsubscribe request for non-existent topic: {topic_name}")

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
        logger.info(f"Broadcasting to topic: {topic_name}")
        if topic_name not in self.topic_subscribers:
            return

        for ws in self.topic_subscribers[topic_name].copy():
            try:
                await ws.send(message)
                logger.info(f"Successfully sent data for topic: {topic_name}")
            except Exception as e:
                logger.error(f"Error sending topic data: {e}")
                self.topic_subscribers[topic_name].remove(ws)
                if not self.topic_subscribers[topic_name]:
                    self.topic_tasks[topic_name].cancel()
                    del self.topic_subscribers[topic_name]
                    del self.topic_tasks[topic_name]
                    logger.info(f"Unsubscribed from topic: {topic_name}")
