import Map from "../components/Map";
import Controls from "../components/Controls";
import TopicViewer from "../components/TopicViewer";
import OdomListener from "../components/OdomListener";
import TopicSubscriber from "../components/TopicSubscriber";
import { MicroApp } from "../types/MicroApp";
import LocationOnIcon from "@mui/icons-material/LocationOn";
import SportsEsportsIcon from "@mui/icons-material/SportsEsports";
import SmartToyIcon from "@mui/icons-material/SmartToy";
import TopicIcon from "@mui/icons-material/Topic";
import { Fullscreen } from "@mui/icons-material";
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
    id: "odom",
    title: "Odom Listener",
    icon: SmartToyIcon,
    width: 500,
    height: 900,
    component: OdomListener,
  },
  {
    id: "topics",
    title: "Topic Viewer",
    icon: TopicIcon,
    width: 500,
    height: 800,
    component: TopicViewer,
  },
  {
    id: "topic-subscriber",
    title: "Topic Subscriber",
    icon: SmartToyIcon,
    width: 500,
    height: 800,
    component: TopicSubscriber,
    allowMultiple: true,
  },
];
