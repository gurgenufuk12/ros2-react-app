import Map from "../components/Map";
import Controls from "../components/Controls";
import TopicViewer from "../components/TopicViewer";
import { MicroApp } from "../types/MicroApp";
import LocationOnIcon from "@mui/icons-material/LocationOn";
import SportsEsportsIcon from "@mui/icons-material/SportsEsports";
import TopicIcon from "@mui/icons-material/Topic";
export const microApps: MicroApp[] = [
  {
    id: "map",
    title: "Map Viewer",
    icon: LocationOnIcon,
    width: 800,
    height: 600,
    component: Map,
  },
  {
    id: "controls",
    title: "Robot Controls",
    icon: SportsEsportsIcon,
    width: 500,
    height: 600,
    component: Controls,
  },
  {
    id: "topics",
    title: "Topic Viewer",
    icon: TopicIcon,
    width: 500,
    height: 400,
    component: TopicViewer,
  },
];
