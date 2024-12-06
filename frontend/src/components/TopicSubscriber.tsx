import React, { useState, useEffect, useRef } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import { WindowComponentProps } from "../types/ComponentProps";
import "../styles/TopicSubscriber.css";

const DefaultTopics = [
  {
    topicName: "/tf",
    msgType: "tf2_msgs/msg/TFMessage",
  },
  {
    topicName: "/odom",
    msgType: "nav_msgs/msg/Odometry",
  },
];

const TopicSubscriber: React.FC<WindowComponentProps> = ({ instanceId }) => {
  const [topicName, setTopicName] = useState("");
  const [msgType, setMsgType] = useState("");
  const [subscribedTopics, setSubscribedTopics] = useState<Set<string>>(
    new Set()
  );
  const [topicData, setTopicData] = useState<Record<string, any>>({});
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();

  const messageHandlerRef = useRef<(message: any) => void>();

  const subscribedTopicsRef = useRef<Set<string>>(subscribedTopics);

  useEffect(() => {
    subscribedTopicsRef.current = subscribedTopics;
  }, [subscribedTopics]);

  useEffect(() => {
    const handler = (message: any) => {
      // console.log(`[${instanceId}] Received message:`, message);

      const currentSubscribedTopics = subscribedTopicsRef.current;
      if (
        message.type === "topic_data" &&
        currentSubscribedTopics.has(message.topic)
      ) {
        // console.log(`[${instanceId}] Setting data for topic:`, message.topic);
        setTopicData((prev) => ({
          ...prev,
          [message.topic]: message.data,
        }));
      }
    };

    messageHandlerRef.current = handler;

    // console.log(`[${instanceId}] Adding message handler for 'topic_data'`);
    addMessageHandler("topic_data", handler);

    return () => {
      // console.log(`[${instanceId}] Removing message handler for 'topic_data'`);
      if (messageHandlerRef.current) {
        removeMessageHandler("topic_data", messageHandlerRef.current);
      }
    };
  }, [instanceId, addMessageHandler, removeMessageHandler]);

  const handleSubscribe = () => {
    if (isConnected && topicName && msgType) {
      // console.log(`[${instanceId}] Subscribing to topic:`, topicName);
      sendMessage({
        type: "subscribe_topic",
        topic: topicName,
        msg_type: msgType,
        instanceId,
      });

      setSubscribedTopics((prev) => new Set(prev).add(topicName));
    }
  };

  const handleUnsubscribe = (topic: string) => {
    if (isConnected) {
      // console.log(`[${instanceId}] Unsubscribing from topic:`, topic);
      sendMessage({
        type: "unsubscribe_topic",
        topic: topic,
        instanceId,
      });

      setSubscribedTopics((prev) => {
        const next = new Set(prev);
        next.delete(topic);
        return next;
      });

      setTopicData((prev) => {
        const next = { ...prev };
        delete next[topic];
        return next;
      });
    }
  };

  return (
    <div className="topic-subscriber">
      <div className="input-group">
        <input
          type="text"
          value={topicName}
          onChange={(e) => setTopicName(e.target.value)}
          placeholder="Topic name (e.g., /tf)"
        />
        <input
          type="text"
          value={msgType}
          onChange={(e) => setMsgType(e.target.value)}
          placeholder="Message type (e.g., tf2_msgs/msg/TFMessage)"
        />
        <button onClick={handleSubscribe}>Subscribe</button>
      </div>
      <div className="select-group">
        Choose from Default Topics:
        <select
          value={topicName}
          onChange={(e) => {
            setTopicName(e.target.value);
            setMsgType(
              DefaultTopics.find((topic) => topic.topicName === e.target.value)
                ?.msgType || ""
            );
          }}
        >
          <option value="">Select Topic</option>

          {DefaultTopics.map(({ topicName }) => (
            <option key={topicName} value={topicName}>
              {topicName}
            </option>
          ))}
        </select>
      </div>
      <div className="subscribed-topics">
        {Array.from(subscribedTopics).map((topic) => (
          <div key={topic} className="topic-header">
            <span>{topic}</span>
            <button onClick={() => handleUnsubscribe(topic)}>
              Unsubscribe
            </button>
          </div>
        ))}
      </div>

      <div className="topic-data">
        {Object.entries(topicData).map(([topic, data]) => (
          <div key={topic} className="data-container">
            <h4>{topic}</h4>
            <pre>{JSON.stringify(data, null, 2)}</pre>
          </div>
        ))}
      </div>
    </div>
  );
};

export default TopicSubscriber;
