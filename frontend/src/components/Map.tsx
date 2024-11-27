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
    const ws = new WebSocket("ws://172.16.66.124:8765");

    ws.onopen = () => {
      console.log("WebSocket connected!");
      setLoading(true);
      ws.send(JSON.stringify({ type: "get_map" }));
      ws.send(JSON.stringify({ type: "get_pose" }));
    };
    // INFO: The WebSocket object has a number of event listeners that can be used to handle different events.
    ws.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        if (message.type === "map_data") {
          console.log("mmmmmmmmmmmmmmmmmmmmmmm");

          setMapData(message.data);
          setLoading(false);
        }
        if (message.type === "pose_data") {
          console.log("pppppppppppppppppppppppppppppp");

          setRobotPose(message.data);
        } else if (message.error) {
          setError(message.error);
          setLoading(false);
        }
      } catch (e) {
        setError("Failed to parse data");
        setLoading(false);
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
  console.log(robotPose);

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
