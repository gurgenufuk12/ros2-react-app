import asyncio
import json
from websockets import WebSocketServerProtocol

class WebSocketManager:
    def __init__(self, ros_bridge):
        self.ros_bridge = ros_bridge

    async def handler(self, websocket: WebSocketServerProtocol, path: str):
        async for message in websocket:
            data = json.loads(message)
            if data["type"] == "cmd_vel":
                self.ros_bridge.publish_cmd_vel(data["linear"], data["angular"])
            elif data["type"] == "get_map":
                await websocket.send(self.ros_bridge.get_map_data())
