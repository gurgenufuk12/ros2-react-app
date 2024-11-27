import React, { useState, useEffect, useRef } from "react";
import { Joystick } from "react-joystick-component";
import "../styles/Controls.css";
const Controls: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  const linearVelocityRef = useRef(0);
  const angularVelocityRef = useRef(0);

  useEffect(() => {
    const websocket = new WebSocket("ws://172.16.66.124:8765");

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

    setWs(websocket);

    return () => {
      websocket.close();
    };
  }, []);

  const startSendingCommand = () => {
    if (ws && isConnected) {
      intervalRef.current = setInterval(() => {
        sendCommand(linearVelocityRef.current, angularVelocityRef.current);
      }, 100);
    }
  };

  const stopSendingCommand = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    sendCommand(0, 0);
  };

  const handleMove = (event: any) => {
    const { x, y } = event;
    const maxSpeed = 1;
    const linear = (y / 2) * maxSpeed;
    const angular = (-x / 2) * maxSpeed;
    linearVelocityRef.current = parseFloat(linear.toFixed(2));
    angularVelocityRef.current = parseFloat(angular.toFixed(2));
  };

  const handleStart = () => {
    startSendingCommand();
  };

  const handleStop = () => {
    linearVelocityRef.current = 0;
    angularVelocityRef.current = 0;
    stopSendingCommand();
  };

  const sendCommand = (linear: number, angular: number) => {
    if (ws && isConnected) {
      const message = {
        type: "cmd_vel",
        linear: {
          x: linear,
          y: 0.0,
          z: 0.0,
        },
        angular: {
          x: 0.0,
          y: 0.0,
          z: angular,
        },
      };
      ws.send(JSON.stringify(message));
      console.log("Command sent:", message);
    } else {
      console.log("WebSocket is not connected.");
    }
  };

  return (
    <div className="controls-container">
      <Joystick
        size={250}
        baseColor="#EEEEEE"
        stickColor="#BBBBBB"
        move={handleMove}
        start={handleStart}
        stop={handleStop}
      />
      <p>{isConnected ? "Connected to WebSocket" : "Disconnected"}</p>
    </div>
  );
};

export default Controls;
