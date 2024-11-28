import React, { useEffect, useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
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
  const { ws, isConnected } = useWebSocket();
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [robotPose, setRobotPose] = useState<Pose | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (ws && isConnected) {
      ws.send(JSON.stringify({ type: "get_map" }));
      ws.send(JSON.stringify({ type: "subscribe_pose" }));

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
      return () => {
        if (ws.readyState === WebSocket.OPEN) {
          ws.close();
        }
      };
    }
  }, [ws, isConnected]);

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
