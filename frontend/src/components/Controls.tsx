import React, { useState, useEffect, useRef } from "react";

const Controls: React.FC = () => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState<boolean>(false);
  const intervalRef = useRef<NodeJS.Timeout | null>(null);

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

    setWs(websocket);

    return () => {
      if (websocket) {
        websocket.close();
      }
    };
  }, []);

  const startSendingCommand = (linear: number, angular: number) => {
    if (ws && isConnected) {
      sendCommand(linear, angular);

      intervalRef.current = setInterval(() => {
        sendCommand(linear, angular);
      }, 100);
    }
  };

  const stopSendingCommand = () => {
    if (intervalRef.current) {
      clearInterval(intervalRef.current);
      intervalRef.current = null;

      sendCommand(0, 0);
    }
  };

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
      <button
        onMouseDown={() => startSendingCommand(0.5, 0)}
        onMouseUp={stopSendingCommand}
        onMouseLeave={stopSendingCommand}
      >
        Forward
      </button>
      <button
        onMouseDown={() => startSendingCommand(-0.5, 0)}
        onMouseUp={stopSendingCommand}
        onMouseLeave={stopSendingCommand}
      >
        Backward
      </button>
      <button
        onMouseDown={() => startSendingCommand(0, 0.5)}
        onMouseUp={stopSendingCommand}
        onMouseLeave={stopSendingCommand}
      >
        Turn Left
      </button>
      <button
        onMouseDown={() => startSendingCommand(0, -0.5)}
        onMouseUp={stopSendingCommand}
        onMouseLeave={stopSendingCommand}
      >
        Turn Right
      </button>
      <p>{isConnected ? "Connected to WebSocket" : "Disconnected"}</p>
    </div>
  );
};

export default Controls;
