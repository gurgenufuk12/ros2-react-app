import React, { useEffect, useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import "../styles/Odometry.css";

interface OdomData {
  pose: {
    position: {
      x: number;
      y: number;
      z: number;
    };
    orientation: {
      x: number;
      y: number;
      z: number;
      w: number;
    };
  };
  // covariance: number[];
  twist: {
    linear: {
      x: number;
      y: number;
      z: number;
    };
    angular: {
      x: number;
      y: number;
      z: number;
    };
  };
}
const OdomListener: React.FC = () => {
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();
  const [odomData, setOdomData] = useState<OdomData | null>(null);
  const [loading, setLoading] = useState<boolean>(true);
  const [error, setError] = useState<string | null>(null);
  useEffect(() => {
    if (isConnected) {
      addMessageHandler("odom_data", setOdomData);
      setLoading(false);
      sendMessage({ type: "subscribe_odom" });
    }
    return () => {
      removeMessageHandler("odom_data");
    };
  }, [isConnected]);

  if (loading) {
    return <div>Loading odom data...</div>;
  }

  if (error) {
    return <div>Error: {error}</div>;
  }
  if (!odomData) {
    return <div>No odom data available</div>;
  }
  console.log("odomData", odomData);

  return (
    <div>
      <h2>Odom Listener</h2>

      <div className="odom-data">
        <div>
          <h3>Position</h3>
          <p>X: {odomData.pose.position.x}</p>
          <p>Y: {odomData.pose.position.y}</p>
          <p>Z: {odomData.pose.position.z}</p>
        </div>
        <div>
          <h3>Orientation</h3>
          <p>X: {odomData.pose.orientation.x}</p>
          <p>Y: {odomData.pose.orientation.y}</p>
          <p>Z: {odomData.pose.orientation.z}</p>
          <p>W: {odomData.pose.orientation.w}</p>
        </div>
        <div>
          <h3>Twist</h3>
          <p>Linear</p>
          <p>X: {odomData.twist.linear.x}</p>
          <p>Y: {odomData.twist.linear.y}</p>
          <p>Z: {odomData.twist.linear.z}</p>
          <p>Angular</p>
          <p>X: {odomData.twist.angular.x}</p>
          <p>Y: {odomData.twist.angular.y}</p>
          <p>Z: {odomData.twist.angular.z}</p>
        </div>
      </div>
    </div>
  );
};

export default OdomListener;
