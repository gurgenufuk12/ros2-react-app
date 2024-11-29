import { SvgIconComponent } from '@mui/icons-material';

export interface MicroApp {
  id: string;
  title: string;
  icon: SvgIconComponent;
  width?: number;
  height?: number;
  component: React.ComponentType;
}