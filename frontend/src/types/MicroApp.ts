import { SvgIconComponent } from "@mui/icons-material";
import { WindowComponentProps } from "../types/ComponentProps";

export interface MicroApp {
  id: string;
  title: string;
  icon: SvgIconComponent;
  width?: number;
  height?: number;
  component: React.ComponentType<WindowComponentProps>;
  allowMultiple?: boolean;
}