import React, { useState, useEffect, useRef } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";
import { WindowComponentProps } from "../types/ComponentProps";
import "../styles/TopicSubscriber.css";

interface Topic {
  name: string;
  type: string;
}

const TopicSubscriber: React.FC<WindowComponentProps> = ({ instanceId }) => {
  const [topicName, setTopicName] = useState("");
  const [msgType, setMsgType] = useState("");
  const [subscribedTopics, setSubscribedTopics] = useState<Set<string>>(
    new Set()
  );
  const [showInputs, setShowInputs] = useState(true);
  const [topics, setTopics] = useState<Topic[]>([]);
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
  const handleSubscribe = () => {
    if (!msgType || !topicName) {
      //CHANGE LATER
      return alert("fill the selected areas");
    }
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
    if (topicName !== "" && msgType !== "") {
      setShowInputs(false);
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
    setShowInputs(true);
  };

  return (
    <div className="topic-subscriber">
      {showInputs ? (
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
      ) : null}
      <div className="select-group">
        Choose from Default Topics:
        <select
          onChange={(e) => {
            const topicName = e.target.value;
            const topic = topics.find((t) => t.name === topicName);
            if (topic) {
              setTopicName(topic.name);
              setMsgType(topic.type);
            }
          }}
        >
          <option value="">Select a topic</option>
          {topics.map((topic, index) => (
            <option key={index} value={topic.name}>
              {topic.name}
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
