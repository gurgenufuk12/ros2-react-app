import json
import logging
import websockets
from websockets.server import WebSocketServerProtocol

logger = logging.getLogger(__name__)


class WebSocketManager:
    def __init__(self, ros_bridge):
        self.ros_bridge = ros_bridge

    async def handler(self, websocket: WebSocketServerProtocol):
        """
        WebSocket bağlantısı üzerinden gelen mesajları işle.
        """

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    print("json.JSONDecodeError")
                    logger.error("Failed to decode message: %s", message)
                    await websocket.send(json.dumps({"error": "Invalid JSON format"}))
                    continue

                if data.get("type") == "cmd_vel":
                    logger.info("Received cmd_vel message: %s", data)
                    self.ros_bridge.publish_cmd_vel(
                        data.get("linear"), data.get("angular"))

                elif data.get("type") == "get_map":
                    logger.info("Received get_map request")
                    map_data = await self.ros_bridge.get_map_data()
                    response = json.dumps(
                        {"type": "map_data", "data": map_data})
                    await websocket.send(response)

                else:
                    logger.warning("Unknown message type: %s",
                                   data.get("type"))

        except Exception as e:
            logger.error("Error handling WebSocket message: %s", e)
        finally:
            try:
                await websocket.close()
                logger.info("WebSocket connection closed")
            except Exception as e:
                logger.error("Error closing WebSocket connection: %s", e)
