import { log } from "console";
import React, { useState, useEffect } from "react";

const Controls: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null); // Store WebSocket in state
  const [isConnected, setIsConnected] = useState<boolean>(false); // Connection status

  // Initialize WebSocket connection once when the component mounts
  useEffect(() => {
    const websocket = new WebSocket("ws://0.0.0.0:8765");

    websocket.onopen = () => {
      console.log("WebSocket connected!");
      setIsConnected(true);
    };

    websocket.onerror = (error) => {
      console.error("WebSocket error:", error);
      setIsConnected(false);
    };

    websocket.onclose = () => {
      console.log("WebSocket closed");
      setIsConnected(false);
    };

    setWs(websocket); // Save the WebSocket instance in state

    // Cleanup the WebSocket when the component unmounts
    return () => {
      if (websocket) {
        websocket.close();
      }
    };
  }, []); // Empty dependency array ensures this runs only once when the component mounts

  // Send a command to the robot

  const sendCommand = (linear: number, angular: number) => {
    if (ws && isConnected) {
      const message = {
        type: "cmd_vel",
        linear: {
          x: parseFloat(linear.toFixed(2)),
          y: 0.0,
          z: 0.0,
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: parseFloat(angular.toFixed(2)),
        },
      };
      ws.send(JSON.stringify(message));
      console.log("Command sent:", message);
    } else {
      console.log("WebSocket is not connected.");
    }
  };

  return (
    <div>
      <button onClick={() => sendCommand(0.5, 0)}>Forward</button>
      <button onClick={() => sendCommand(-0.5, 0)}>Backward</button>
      <button onClick={() => sendCommand(0, 0.5)}>Turn Left</button>
      <button onClick={() => sendCommand(0, -0.5)}>Turn Right</button>
      <p>{isConnected ? "Connected to WebSocket" : "Disconnected"}</p>
    </div>
  );
};

export default Controls;
