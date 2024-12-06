import React, { useState, useEffect, useRef } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import "../styles/Odometry.css";

interface OdomData {
  data: {
    pose: {
      position: { x: number; y: number; z: number };
      orientation: { x: number; y: number; z: number; w: number };
    };
    twist: {
      linear: { x: number; y: number; z: number };
      angular: { x: number; y: number; z: number };
    };
  };
  timestamp?: string;
}

const OdomListener: React.FC = () => {
  const [autoScroll, setAutoScroll] = useState(true);
  const [odomHistory, setOdomHistory] = useState<OdomData[]>([]);
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const scrollRef = useRef<HTMLDivElement>(null);
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();

  const handleScroll = () => {
    if (scrollRef.current) {
      const { scrollTop, scrollHeight, clientHeight } = scrollRef.current;
      const isAtBottom = Math.abs(scrollHeight - clientHeight - scrollTop) < 10;
      setAutoScroll(isAtBottom);
    }
  };

  useEffect(() => {
    if (autoScroll && scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [odomHistory, autoScroll]);

  const handleOdomData = (data: OdomData) => {
    const currentTime = Date.now();
    if (currentTime - lastUpdateTime >= 1000) {
      const dataWithTimestamp = {
        ...data,
        timestamp: new Date().toLocaleTimeString(),
      };
      setOdomHistory((prev) => [...prev, dataWithTimestamp]);
      setLastUpdateTime(currentTime);
    }
  };

  useEffect(() => {
    if (isConnected) {
      addMessageHandler("odom_data", handleOdomData);
      sendMessage({ type: "subscribe_odom" });
    }

    return () => {
      removeMessageHandler("odom_data", handleOdomData);
      sendMessage({ type: "unsubscribe_odom" });
    };
  }, [isConnected, lastUpdateTime]);
  return (
    <div className="odom-window">
      <div className="odom-history" ref={scrollRef} onScroll={handleScroll}>
        {odomHistory.map((item, index) => (
          <div key={index} className="odom-entry">
            <h4>Timestamp: {item.timestamp}</h4>

            {/* Position Data */}
            {item.data.pose && item.data.pose.position && (
              <div className="position-data">
                <h5>Position</h5>
                <p>X: {item.data.pose.position.x}</p>
                <p>Y: {item.data.pose.position.y}</p>
                <p>Z: {item.data.pose.position.z}</p>
              </div>
            )}

            {/* Twist Data */}
            {item.data.twist && (
              <div className="twist-data">
                <h5>Twist - Linear</h5>
                <p>X: {item.data.twist.linear.x}</p>
                <p>Y: {item.data.twist.linear.y}</p>
                <p>Z: {item.data.twist.linear.z}</p>

                <h5>Twist - Angular</h5>
                <p>X: {item.data.twist.angular.x}</p>
                <p>Y: {item.data.twist.angular.y}</p>
                <p>Z: {item.data.twist.angular.z}</p>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  );
};

export default OdomListener;
