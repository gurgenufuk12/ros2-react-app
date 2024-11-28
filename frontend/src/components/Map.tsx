// Map.tsx
import React, { useEffect, useState } from "react";
import MapRenderer from "./MapRenderer";
import "../styles/Map.css";
import { Pose } from "../types/Pose";

interface MapData {
  width: number;
  height: number;
  resolution: number;
  origin: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  data: number[];
}

const Map: React.FC = () => {
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [robotPose, setRobotPose] = useState<Pose | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const ws = new WebSocket("ws://10.34.112.152:8765"); // IP OF YOUR COMPUTER

    ws.onopen = () => {
      console.log("WebSocket connected!");
      setLoading(true);
      console.log("sending get_map");
      ws.send(JSON.stringify({ type: "get_map" }));
      console.log("sending subscribe_pose");
      ws.send(JSON.stringify({ type: "subscribe_pose" }));
    };

    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        console.log("Received message type:", message.type);
        console.log("Full message:", message);

        switch (message.type) {
          case "map_data":
            console.log("Setting map data");
            setMapData(message.data);
            setLoading(false);
            break;
          case "pose_data":
            console.log("Setting pose data:", message.data);
            setRobotPose(message.data);
            break;
          default:
            console.warn("Unknown message type:", message.type);
        }
      } catch (error) {
        console.error("Error processing message:", error);
        setError("Failed to process message");
      }
    };

    ws.onerror = (error) => {
      console.error("WebSocket error:", error);
      setError("WebSocket connection error");
      setLoading(false);
    };

    ws.onclose = () => {
      console.log("WebSocket disconnected");
      setError("WebSocket connection closed");
      setLoading(false);
    };

    return () => {
      if (ws.readyState === WebSocket.OPEN) {
        ws.close();
      }
    };
  }, []);

  if (loading) {
    return <div className="map-container">Loading map data...</div>;
  }

  if (error) {
    return <div className="map-container">Error: {error}</div>;
  }

  if (!mapData) {
    return <div className="map-container">No map data available</div>;
  }

  return (
    <div className="map-container">
      <MapRenderer mapData={mapData} robotPose={robotPose} />
    </div>
  );
};

export default Map;
