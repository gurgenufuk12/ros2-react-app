import React from "react";
import { ThemeProvider, CssBaseline, Box } from "@mui/material";
import { WebSocketProvider } from "./contexts/WebSocketContext";
import Sidebar from "./components/Sidebar";
import WindowManager from "./components/WindowManager";
import { theme } from "./theme/theme";

const App: React.FC = () => {
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <WebSocketProvider>
        <Box
          sx={{
            display: "flex",
            height: "100vh",
            backgroundColor: "background.default",
            overflow: "hidden",
          }}
        >
          <Sidebar />
          <Box
            component="main"
            sx={{ flexGrow: 1, position: "relative", p: 3 }}
          >
            <WindowManager />
          </Box>
        </Box>
      </WebSocketProvider>
    </ThemeProvider>
  );
};

export default App;
