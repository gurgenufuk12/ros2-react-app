import React, { useState, useEffect, useRef } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import "../styles/Odometry.css";

interface OdomData {
  pose: {
    position: { x: number; y: number; z: number };
    orientation: { x: number; y: number; z: number; w: number };
  };
  twist: {
    linear: { x: number; y: number; z: number };
    angular: { x: number; y: number; z: number };
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

  useEffect(() => {
    if (isConnected) {
      addMessageHandler("odom_data", (data: OdomData) => {
        const currentTime = Date.now();
        if (currentTime - lastUpdateTime >= 1000) {
          const dataWithTimestamp = {
            ...data,
            timestamp: new Date().toLocaleTimeString(),
          };
          setOdomHistory((prev) => [...prev, dataWithTimestamp]);
          setLastUpdateTime(currentTime);
        }
      });
      sendMessage({ type: "subscribe_odom" });
    }

    return () => {
      removeMessageHandler("odom_data");
      sendMessage({ type: "unsubscribe_odom" });
    };
  }, [isConnected, lastUpdateTime]);

  return (
    <div className="odom-window">
      <div className="odom-history" ref={scrollRef} onScroll={handleScroll}>
        {odomHistory.map((data, index) => (
          <div key={index} className="odom-entry">
            <div className="odom-data">
              <div>
                <h4>Position</h4>
                <p>X: {data.pose.position.x}</p>
                <p>Y: {data.pose.position.y}</p>
                <p>Z: {data.pose.position.z}</p>
              </div>
              <div>
                <h4>Orientation</h4>
                <p>X: {data.pose.orientation.x}</p>
                <p>Y: {data.pose.orientation.y}</p>
                <p>Z: {data.pose.orientation.z}</p>
                <p>W: {data.pose.orientation.w}</p>
              </div>
              <div>
                <h4>Twist</h4>
                <p>Linear X: {data.twist.linear.x}</p>
                <p>Linear Y: {data.twist.linear.y}</p>
                <p>Linear Z: {data.twist.linear.z}</p>
                <p>Angular X: {data.twist.angular.x}</p>
                <p>Angular Y: {data.twist.angular.y}</p>
                <p>Angular Z: {data.twist.angular.z}</p>
              </div>
            </div>
            <hr />
          </div>
        ))}
      </div>
    </div>
  );
};

export default OdomListener;
