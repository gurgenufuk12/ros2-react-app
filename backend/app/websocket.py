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
        self.pose_subscribers = set()
        self.ros_bridge = ros_bridge
        self.update_pose_task = None

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
        if msg_type == "cmd_vel":
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
        else:
            logger.warning("Unknown message type: %s", msg_type)

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
