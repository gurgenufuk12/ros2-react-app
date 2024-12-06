import React, { useEffect, useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import MapRenderer from "./MapRenderer";
import "../styles/Map.css";
import { Pose } from "../types/Pose";

interface MapData {
  data:{
    width: number;
    height: number;
    resolution: number;
    origin: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
    data: number[];
  }
}

const Map: React.FC = () => {
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();
  const [mapData, setMapData] = useState<MapData | null>(null);
  const [robotPose, setRobotPose] = useState<Pose | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (isConnected) {
      addMessageHandler("map_data", setMapData);
      addMessageHandler("pose_data", setRobotPose);
      setLoading(false);
      sendMessage({ type: "get_map" });
      sendMessage({ type: "subscribe_pose" });
    }

    return () => {
      removeMessageHandler("map_data", setMapData);
      removeMessageHandler("pose_data", setRobotPose);
    };
  }, [isConnected]);
  
  if (loading) {
    return <div className="map-container">Loading map data...</div>;
  }
  // console.log("mapData", mapData);

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
