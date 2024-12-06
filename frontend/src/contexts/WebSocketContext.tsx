import React, {
  createContext,
  useContext,
  useEffect,
  useRef,
  useState,
} from "react";

interface WebSocketContextType {
  ws: WebSocket | null;
  isConnected: boolean;
  sendMessage: (message: any) => void;
  addMessageHandler: (type: string, handler: (data: any) => void) => void;
  removeMessageHandler: (type: string, handler: (data: any) => void) => void;
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


  const messageHandlersRef = useRef<{ [key: string]: ((data: any) => void)[] }>(
    {}
  );

  const sendMessage = (message: any) => {
    if (ws && isConnected) {
      console.log("Sending message:", message);
      ws.send(JSON.stringify(message));
    }
  };

  const addMessageHandler = (type: string, handler: (data: any) => void) => {
    console.log("Adding handler for:", type);
    if (!messageHandlersRef.current[type]) {
      messageHandlersRef.current[type] = [];
    }
    messageHandlersRef.current[type].push(handler);
  };

  const removeMessageHandler = (type: string, handler: (data: any) => void) => {
    console.log("Removing handler for:", type);
    const handlers = messageHandlersRef.current[type];
    if (handlers) {
      const index = handlers.indexOf(handler);
      if (index !== -1) {
        handlers.splice(index, 1);
        console.log(`Handler removed for type: ${type}`);
      } else {
        console.warn(`Handler not found for type: ${type}`);
      }
      if (handlers.length === 0) {
        delete messageHandlersRef.current[type];
      }
    }
  };

  useEffect(() => {
    const websocket = new WebSocket("ws://10.120.66.142:8765");

    websocket.onopen = () => {
      console.log("WebSocket connected");
      setWs(websocket);
      setIsConnected(true);
    };

    websocket.onmessage = (event) => {
      const message = JSON.parse(event.data);
      // console.log("WebSocket received:", message);
      const type = message.type;
      const handlers = messageHandlersRef.current[type];
      if (handlers) {
        handlers.forEach((handler) => {
          try {
            handler(message);
          } catch (error) {
            console.error("Error in message handler:", error);
          }
        });
      }
    };

    websocket.onclose = () => {
      console.log("WebSocket disconnected");
      setIsConnected(false);
    };

    websocket.onerror = (error) => {
      console.error("WebSocket error:", error);
    };

    return () => {
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
