import React from "react";
import Draggable from "react-draggable";
import { Paper, IconButton, Typography, Box } from "@mui/material";
import { styled } from "@mui/material/styles";
import CloseIcon from "@mui/icons-material/Close";
import MinimizeIcon from "@mui/icons-material/Minimize";

const WindowContainer = styled(Paper)(({ theme }) => ({
  position: "absolute",
  backgroundColor: theme.palette.background.paper,
  borderRadius: theme.shape.borderRadius,
  overflow: "hidden",
  boxShadow: theme.shadows[8],
  border: `1px solid ${theme.palette.divider}`,
  transition: "box-shadow 0.3s ease",
  "&:hover": {
    boxShadow: theme.shadows[12],
  },
}));

const WindowHeader = styled(Box)(({ theme }) => ({
  padding: "12px 16px",
  background: `linear-gradient(45deg, ${theme.palette.primary.main}, ${theme.palette.primary.dark})`,
  color: theme.palette.common.white,
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  cursor: "move",
  userSelect: "none",
}));

const WindowContent = styled(Box)(({ theme }) => ({
  padding: theme.spacing(2),
  height: "calc(100% - 64px)",
  overflow: "auto",
  "&::-webkit-scrollbar": {
    width: "8px",
  },
  "&::-webkit-scrollbar-track": {
    background: theme.palette.background.default,
  },
  "&::-webkit-scrollbar-thumb": {
    background: theme.palette.primary.light,
    borderRadius: "4px",
  },
}));

interface Props {
  id: string;
  title: string;
  onClose: () => void;
  children: React.ReactNode;
  width?: number;
  height?: number;
  initialPosition?: { x: number; y: number };
}

const FloatingWindow: React.FC<Props> = ({
  id,
  title,
  onClose,
  children,
  width = 400,
  height = 300,
  initialPosition,
}) => {
  return (
    <Draggable
      handle=".window-header"
      defaultPosition={initialPosition}
      bounds="parent"
    >
      <WindowContainer sx={{ width, height }}>
        <WindowHeader className="window-header">
          <Typography variant="subtitle1" fontWeight="medium">
            {title}
          </Typography>
          <Box>
            <IconButton size="small" sx={{ color: "white", mr: 1 }}>
              <MinimizeIcon fontSize="small" />
            </IconButton>
            <IconButton size="small" onClick={onClose} sx={{ color: "white" }}>
              <CloseIcon fontSize="small" />
            </IconButton>
          </Box>
        </WindowHeader>
        <WindowContent>{children}</WindowContent>
      </WindowContainer>
    </Draggable>
  );
};

export default FloatingWindow;
