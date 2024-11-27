import asyncio
from websockets import serve
from fastapi import FastAPI
import rclpy
from ros import ROSBridge
from websocket import WebSocketManager
from functools import partial

app = FastAPI()


async def start_websocket_server(ros_bridge):
    manager = WebSocketManager(ros_bridge)

    server = await serve(partial(manager.handler), "172.16.66.124", 8765)
    print("WebSocket server started")
    return server


async def ros_spin(ros_bridge):
    """ROS 2 spinning process."""
    while rclpy.ok():
        rclpy.spin_once(ros_bridge, timeout_sec=0.1)
        # Non-blocking delay to yield control to other tasks.
        await asyncio.sleep(0.1)


@app.on_event("startup")
async def startup():
    rclpy.init()
    ros_bridge = ROSBridge()
    print("ROS2 node initialized")

    # Start the WebSocket server
    asyncio.create_task(start_websocket_server(ros_bridge))

    # Start the ROS spin loop
    asyncio.create_task(ros_spin(ros_bridge))


@app.on_event("shutdown")
async def shutdown():
    print("Shutting down ROS2...")
    rclpy.shutdown()
