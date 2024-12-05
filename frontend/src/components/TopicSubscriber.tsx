import React, { useState } from "react";
import { useWebSocket } from "../contexts/WebSocketContext";

interface TopicData {
  [key: string]: any;
}

const TopicSubscriber: React.FC = () => {
  const [topicName, setTopicName] = useState("");
  const [msgType, setMsgType] = useState("");
  const [topicData, setTopicData] = useState<Record<string, TopicData>>({});
  const { isConnected, sendMessage, addMessageHandler, removeMessageHandler } =
    useWebSocket();

  const handleSubscribe = () => {
    if (isConnected) {
      if (topicName && msgType) {
        sendMessage({
          type: "subscribe_topic",
          topic: topicName,
          msg_type: msgType,
        });
        addMessageHandler("topic_data", (data: any) => {
          console.log("data", data);

          setTopicData((prev) => ({
            ...prev,
            data: data,
          }));
        });
      }
    }
  };
  const handleUnsubscribe = () => {
    removeMessageHandler("subscribe_topic");
    sendMessage({ type: "unsubscribe_topic", topic: topicName });
  };

  console.log("topicData", topicData);

  return (
    <div className="topic-subscriber">
      <div className="input-group">
        <input
          type="text"
          value={topicName}
          onChange={(e) => setTopicName(e.target.value)}
          placeholder="Topic name (e.g. /cmd_vel)"
        />
        <input
          type="text"
          value={msgType}
          onChange={(e) => setMsgType(e.target.value)}
          placeholder="Message type (e.g. geometry_msgs/Twist)"
        />
        <button onClick={handleSubscribe}>Subscribe</button>
        <button onClick={handleUnsubscribe}>Unsubscribe</button>
      </div>

      <div className="topic-data">
        {Object.entries(topicData).map(([topic, data]) => (
          <div key={topic}>
            <h3>{topic}</h3>
            <pre>{JSON.stringify(data, null, 2)}</pre>
          </div>
        ))}
      </div>
    </div>
  );
};

export default TopicSubscriber;
