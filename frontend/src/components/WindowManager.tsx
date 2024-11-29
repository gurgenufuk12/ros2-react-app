import React from "react";
import { useWindowStore } from "../stores/windowStore";
import { microApps } from "../config/microApps";
import FloatingWindow from "./FloatingWindow";

const WindowManager: React.FC = () => {
  const { openWindows, removeWindow } = useWindowStore();

  return (
    <div style={{ position: "relative", height: "100%", overflow: "hidden" }}>
      {Object.entries(openWindows).map(([id, position]) => {
        const app = microApps.find((app) => app.id === id);
        if (!app) return null;

        const AppComponent = app.component;

        return (
          <FloatingWindow
            key={id}
            id={id}
            title={app.title}
            width={app.width}
            height={app.height}
            initialPosition={position}
            onClose={() => removeWindow(id)}
          >
            <AppComponent />
          </FloatingWindow>
        );
      })}
    </div>
  );
};

export default WindowManager;
