import { createTheme } from "@mui/material";

export const theme = createTheme({
  palette: {
    primary: {
      main: "#2563eb",
      light: "#3b82f6",
      dark: "#1d4ed8",
    },
    secondary: {
      main: "#7c3aed",
      light: "#8b5cf6",
      dark: "#6d28d9",
    },
    background: {
      default: "#f8fafc",
      paper: "#ffffff",
    },
  },
  typography: {
    fontFamily: '"Inter", "Roboto", "Helvetica", "Arial", sans-serif',
  },
  shape: {
    borderRadius: 12,
  },
  shadows: [
    "none",
    "0px 2px 4px rgba(0,0,0,0.05)",
    "0px 3px 6px rgba(0,0,0,0.1)",
    "0px 4px 8px rgba(0,0,0,0.1)",
    "0px 5px 10px rgba(0,0,0,0.1)",
    "0px 6px 12px rgba(0,0,0,0.1)",
    "0px 7px 14px rgba(0,0,0,0.1)",
    "0px 8px 16px rgba(0,0,0,0.1)",
    "0px 9px 18px rgba(0,0,0,0.1)",
    "0px 10px 20px rgba(0,0,0,0.1)",
    "0px 11px 22px rgba(0,0,0,0.1)",
    "0px 12px 24px rgba(0,0,0,0.1)",
    "0px 13px 26px rgba(0,0,0,0.1)",
    "0px 14px 28px rgba(0,0,0,0.1)",
    "0px 15px 30px rgba(0,0,0,0.1)",
    "0px 16px 32px rgba(0,0,0,0.1)",
    "0px 17px 34px rgba(0,0,0,0.1)",
    "0px 18px 36px rgba(0,0,0,0.1)",
    "0px 19px 38px rgba(0,0,0,0.1)",
    "0px 20px 40px rgba(0,0,0,0.1)",
    "0px 21px 42px rgba(0,0,0,0.1)",
    "0px 22px 44px rgba(0,0,0,0.1)",
    "0px 23px 46px rgba(0,0,0,0.1)",
    "0px 24px 48px rgba(0,0,0,0.1)",
    "0px 25px 50px rgba(0,0,0,0.1)",
  ],
});
