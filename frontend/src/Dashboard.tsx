import React from "react";
import Map from "./components/Map";
import Controls from "./components/Controls";
import "./App.css";

const App: React.FC = () => {
  return (
    <div className="app-container">
      <div className="left-pane">
        <Map />
      </div>
      <div className="right-pane">
        <Controls />
      </div>
    </div>
  );
};

export default App;
