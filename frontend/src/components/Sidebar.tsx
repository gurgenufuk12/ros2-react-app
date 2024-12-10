import React from "react";
import { styled } from "@mui/material/styles";
import {
  Drawer,
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Box,
  Typography,
} from "@mui/material";
import { microApps } from "../config/microApps";
import { useWindowStore } from "../stores/windowStore";

const StyledDrawer = styled(Drawer)(({ theme }) => ({
  "@media (max-width: 600px)": {
    width: 240,
    flexShrink: 0,
  },
  "& .MuiDrawer-paper": {
    width: 100,
    boxSizing: "border-box",
    backgroundColor: theme.palette.background.paper,
    borderRight: "none",
    boxShadow: theme.shadows[2],
    "& .MuiListItemText-primary": {
      display: "none",
    },
    ":hover": {
      width: 280,
      cursor: "pointer",
      "& .MuiListItemText-primary": {
        display: "block",
      },
    },
  },
}));

const StyledListItem = styled(ListItem)(({ theme }) => ({
  margin: "8px 16px",
  borderRadius: theme.shape.borderRadius,
  "&:hover": {
    backgroundColor: theme.palette.primary.light,
    "& .MuiListItemIcon-root, & .MuiListItemText-primary": {
      color: theme.palette.common.white,
    },
  },
  transition: "all 0.2s ease-in-out",
}));

const Logo = styled(Box)(({ theme }) => ({
  padding: theme.spacing(3),
  borderBottom: `1px solid ${theme.palette.divider}`,
  marginBottom: theme.spacing(2),
}));

const Sidebar: React.FC = () => {
  const addWindow = useWindowStore((state) => state.addWindow);

  return (
    <StyledDrawer variant="permanent">
      <Logo>
        <Typography variant="h6" color="primary" fontWeight="bold">
          Panel
        </Typography>
      </Logo>
      <List>
        {microApps.map((app) => (
          <StyledListItem key={app.id} onClick={() => addWindow(app.id)}>
            <ListItemIcon sx={{ minWidth: 40 }}>
              <app.icon />
            </ListItemIcon>
            <ListItemText primary={app.title} />
          </StyledListItem>
        ))}
      </List>
    </StyledDrawer>
  );
};

export default Sidebar;
