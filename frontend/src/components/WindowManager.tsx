import React from "react";
import { useWindowStore } from "../stores/windowStore";
import { microApps } from "../config/microApps";
import FloatingWindow from "./FloatingWindow";

const WindowManager: React.FC = () => {
  const { openWindows, removeWindow } = useWindowStore();

  return (
    <div style={{ position: "relative", height: "100%", overflow: "hidden" }}>
      {openWindows.map((window) => {
        const app = microApps.find((app) => app.id === window.appId);
        if (!app) return null;

        const AppComponent = app.component;

        return (
          <FloatingWindow
            key={window.id}
            id={window.id}
            title={app.title}
            width={app.width}
            height={app.height}
            initialPosition={window.position}
            onClose={() => removeWindow(window.id)}
          >
            <AppComponent instanceId={window.id} />
          </FloatingWindow>
        );
      })}
    </div>
  );
};

export default WindowManager;
