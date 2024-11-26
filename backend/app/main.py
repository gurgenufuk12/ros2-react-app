import asyncio
from fastapi import FastAPI
import rclpy
from websockets import serve
from ros import ROSBridge
from websocket import WebSocketManager

app = FastAPI()

def start_websocket_server(ros_bridge):
    manager = WebSocketManager(ros_bridge)
    return serve(manager.handler, "0.0.0.0", 8765)

@app.on_event("startup")
async def startup():
    rclpy.init()
    ros_bridge = ROSBridge()
    asyncio.create_task(start_websocket_server(ros_bridge))
