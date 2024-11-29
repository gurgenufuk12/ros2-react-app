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
      // Add handler for topic list messages
      addMessageHandler("topic_list", (data) => {
        setTopics(data);
      });

      // Request topics
      sendMessage({ type: "get_topics" });
    }

    return () => {
      removeMessageHandler("topic_list");
    };
  }, [isConnected, addMessageHandler, removeMessageHandler, sendMessage]);

  const handleTopicSelect = (topicName: string) => {
    if (topicName && isConnected) {
      sendMessage({ type: "subscribe", topic: topicName });
    }
  };

  return (
    <div className="topic-viewer">
      <h3>ROS2 Topics</h3>
      <select
        name="topics"
        id="topics"
        onChange={(e) => handleTopicSelect(e.target.value)}
      >
        <option value="">Select a topic</option>
        {topics.map((topic) => (
          <option key={topic.name} value={topic.name}>
            {topic.name} ({topic.type})
          </option>
        ))}
      </select>
    </div>
  );
};

export default TopicViewer;
