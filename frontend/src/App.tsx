import React from "react";
import Dashboard from "./Dashboard";
import { WebSocketProvider } from "./contexts/WebSocketContext";

const App: React.FC = () => {
  return (
    <WebSocketProvider>
      <Dashboard />
    </WebSocketProvider>
  );
};

export default App;
