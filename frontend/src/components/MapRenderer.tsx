import React from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { Pose } from "../types/Pose";
import Robot from "./Robot";

interface MapRendererProps {
  mapData: {
    data: {
      width: number;
      height: number;
      resolution: number;
      origin: {
        position: { x: number; y: number; z: number };
        orientation: { x: number; y: number; z: number; w: number };
      };
      data: number[];
    };
  };
  robotPose: Pose | null;
}

const MapRenderer: React.FC<MapRendererProps> = ({ mapData, robotPose }) => {
  const { width, height, resolution, data } = mapData.data;

  const mapOffsetX = (width * resolution) / 2;
  const mapOffsetY = (height * resolution) / 2;

  const GridCells = () => {
    const cells = [];

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const value = data[index];

        if (value > 50) {
          const worldX = x * resolution - mapOffsetX;
          const worldZ = y * resolution - mapOffsetY;

          cells.push(
            <mesh key={`cell-${x}-${y}`} position={[worldX, 0, worldZ]}>
              <boxGeometry args={[resolution, resolution * 0.5, resolution]} />
              <meshStandardMaterial color="#666666" />
            </mesh>
          );
        }
      }
    }
    return <>{cells}</>;
  };

  return (
    <div style={{ width: "100%", height: "100%" }}>
      <Canvas
        camera={{
          position: [0, 50, 50],
          fov: 75,
        }}
      >
        <ambientLight intensity={0.5} />
        <pointLight position={[10, 10, 10]} />
        <GridCells />
        {robotPose && <Robot pose={robotPose} />}
        <OrbitControls />
        <gridHelper args={[50, 50]} />
      </Canvas>
    </div>
  );
};

export default MapRenderer;
