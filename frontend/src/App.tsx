import React from "react";
// import Map from "./components/Map";
import Controls from "./components/Controls";

const App: React.FC = () => {
  return (
    <div>
      <h1>Robot Control Panel</h1>
      <Controls />
      {/* <Map /> */}
    </div>
  );
};

export default App;
