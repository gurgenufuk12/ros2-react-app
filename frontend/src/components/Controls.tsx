import React, { useState, useRef } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import { Joystick } from "react-joystick-component";
import "../styles/Controls.css";
const Controls: React.FC = () => {
  const intervalRef = useRef<NodeJS.Timeout | null>(null);
  const { ws, isConnected } = useWebSocket();
  const linearVelocityRef = useRef(0);
  const angularVelocityRef = useRef(0);
  const [linearVelocity, setLinearVelocity] = useState(0);
  const [angularVelocity, setAngularVelocity] = useState(0);

  const startSendingCommand = () => {
    if (ws && isConnected) {
      intervalRef.current = setInterval(() => {
        sendCommand(linearVelocityRef.current, angularVelocityRef.current);
      }, 1);
    }
  };

  const stopSendingCommand = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;
    }
    sendCommand(0, 0);
    setAngularVelocity(0);
    setLinearVelocity(0);
  };

  const handleMove = (event: any) => {
    const { x, y } = event;
    const maxSpeed = 1;
    const linear = (y / 2) * maxSpeed;
    const angular = (-x / 2) * maxSpeed;

    linearVelocityRef.current = parseFloat(linear.toFixed(2));
    angularVelocityRef.current = parseFloat(angular.toFixed(2));
    setLinearVelocity(linearVelocityRef.current);
    setAngularVelocity(angularVelocityRef.current);
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
      <span>Lineer Velocity :{linearVelocity}</span>
      <span>Angular Velocity :{angularVelocity}</span>

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
