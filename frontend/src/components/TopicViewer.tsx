// src/components/TopicViewer.tsx
import React, { useEffect, useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import "../styles/TopicViewer.css";

interface Topic {
  name: string;
  type: string;
}

const TopicViewer: React.FC = () => {
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();
  const [topics, setTopics] = useState<Topic[]>([]);

  useEffect(() => {
    if (isConnected) {
      addMessageHandler("topic_list", (data) => {
        setTopics(data.data);
      });

      // Request topics
      sendMessage({ type: "get_topics" });
    }

    return () => {
      removeMessageHandler("topic_list", (data) => {
        setTopics([]);
      });
    };
  }, [isConnected, addMessageHandler, removeMessageHandler, sendMessage]);

  const handleTopicSelect = (topicName: string) => {
    if (topicName && isConnected) {
      sendMessage({ type: "subscribe", topic: topicName });
    }
  };
  console.log();

  return (
    <div className="topic-viewer">
      <h3>ROS2 Topics</h3>

      <ul>
        {topics.map((topic, index) => (
          <p key={index}>
            <strong>{topic.name}</strong> - <em>{topic.type}</em>
          </p>
        ))}
      </ul>
    </div>
  );
};

export default TopicViewer;
