import asyncio
from websockets import serve
from fastapi import FastAPI
import rclpy
from ros import ROSBridge
from websocket import WebSocketManager
from functools import partial

app = FastAPI()

# Define the WebSocket server coroutine


async def start_websocket_server(ros_bridge):
    manager = WebSocketManager(ros_bridge)

    # Use functools.partial to properly pass the handler with required arguments
    server = await serve(partial(manager.handler), "0.0.0.0", 8765)
    print("WebSocket server started")
    return server


@app.on_event("startup")
async def startup():
    # Initialize ROS2
    rclpy.init()
    ros_bridge = ROSBridge()
    print("ROS2 node initialized")

    # Start the WebSocket server as a background task
    asyncio.create_task(start_websocket_server(ros_bridge))
