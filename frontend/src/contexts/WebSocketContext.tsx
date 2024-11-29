import React, {
  createContext,
  useContext,
  useEffect,
  useState,
  useRef,
} from "react";

interface WebSocketContextType {
  ws: WebSocket | null;
  isConnected: boolean;
  sendMessage: (message: any) => void;
  addMessageHandler: (type: string, handler: (data: any) => void) => void;
  removeMessageHandler: (type: string) => void;
}

const WebSocketContext = createContext<WebSocketContextType>({
  ws: null,
  isConnected: false,
  sendMessage: () => {},
  addMessageHandler: () => {},
  removeMessageHandler: () => {},
});

export const WebSocketProvider: React.FC<{ children: React.ReactNode }> = ({
  children,
}) => {
  const [ws, setWs] = useState<WebSocket | null>(null);
  const [isConnected, setIsConnected] = useState(false);
  const messageHandlersRef = useRef<{ [key: string]: (data: any) => void }>({});

  const sendMessage = (message: any) => {
    if (ws && isConnected) {
      console.log("Sending message:", message);
      ws.send(JSON.stringify(message));
    }
  };

  const addMessageHandler = (type: string, handler: (data: any) => void) => {
    console.log("Adding handler for:", type);
    messageHandlersRef.current[type] = handler;
  };

  const removeMessageHandler = (type: string) => {
    console.log("Removing handler for:", type);
    delete messageHandlersRef.current[type];
  };

  useEffect(() => {
    const websocket = new WebSocket("ws://172.16.66.124:8765");

    websocket.onopen = () => {
      console.log("WebSocket connected");
      setIsConnected(true);
    };

    websocket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data);
        // console.log("Received message:", message);

        if (message.type && messageHandlersRef.current[message.type]) {
          // console.log("Handling message type:", message.type);
          messageHandlersRef.current[message.type](message.data);
        }
      } catch (error) {
        console.error("Error handling message:", error);
      }
    };

    setWs(websocket);

    return () => {
      console.log("Closing WebSocket");
      websocket.close();
    };
  }, []);

  return (
    <WebSocketContext.Provider
      value={{
        ws,
        isConnected,
        sendMessage,
        addMessageHandler,
        removeMessageHandler,
      }}
    >
      {children}
    </WebSocketContext.Provider>
  );
};

export const useWebSocket = () => useContext(WebSocketContext);
