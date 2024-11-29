import { create } from "zustand";

interface Position {
  x: number;
  y: number;
}

interface WindowState {
  openWindows: Record<string, Position>;
  addWindow: (id: string, position: Position) => void;
  removeWindow: (id: string) => void;
  updatePosition: (id: string, position: Position) => void;
}

export const useWindowStore = create<WindowState>()((set) => ({
  openWindows: {},

  addWindow: (id, position) =>
    set((state) => ({
      openWindows: { ...state.openWindows, [id]: position },
    })),

  removeWindow: (id) =>
    set((state) => {
      const { [id]: _, ...rest } = state.openWindows;
      return { openWindows: rest };
    }),

  updatePosition: (id, position) =>
    set((state) => ({
      openWindows: { ...state.openWindows, [id]: position },
    })),
}));
