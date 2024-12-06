import { create } from "zustand";

interface Position {
  x: number;
  y: number;
}

interface WindowInstance {
  id: string;
  appId: string;
  position: Position;
}

interface WindowState {
  openWindows: WindowInstance[];
  addWindow: (appId: string) => void;
  removeWindow: (instanceId: string) => void;
}

export const useWindowStore = create<WindowState>((set) => ({
  openWindows: [],
  addWindow: (appId) =>
    set((state) => {
      const instanceId = `${appId}-${Date.now()}`;
      const offset = state.openWindows.length * 30;
      return {
        openWindows: [
          ...state.openWindows,
          {
            id: instanceId,
            appId,
            position: { x: 100 + offset, y: 100 + offset },
          },
        ],
      };
    }),
  removeWindow: (instanceId) =>
    set((state) => ({
      openWindows: state.openWindows.filter((w) => w.id !== instanceId),
    })),
}));
