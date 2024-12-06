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
  width: 280,
  flexShrink: 0,
  ":hover": {
    cursor: "pointer",
  },
  "& .MuiDrawer-paper": {
    width: 280,
    boxSizing: "border-box",
    backgroundColor: theme.palette.background.paper,
    borderRight: "none",
    boxShadow: theme.shadows[2],
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
          ROS2 Control Panel
        </Typography>
      </Logo>
      <List>
        {microApps.map((app) => (
          <StyledListItem
            key={app.id}
            onClick={() =>
              addWindow(app.id)
            }
          >
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
