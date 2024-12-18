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

// MapRenderer.tsx
const MapRenderer: React.FC<MapRendererProps> = ({ mapData, robotPose }) => {
  const { width, height, resolution, data } = mapData.data;
  console.log("🚀 ~ resolution:", resolution);

  // Calculate map dimensions and center
  const mapWidth = width * resolution;
  const mapHeight = height * resolution;
  const mapCenterX = mapWidth / 2;
  console.log("🚀 ~ mapCenterX:", mapCenterX)
  const mapCenterZ = mapHeight / 2;
  

  const GridCells = () => {
    const cells = [];

    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const index = y * width + x;
        const value = data[index];

        if (value > 50) {
          // Position cells relative to map center
          const worldX = x * resolution - mapCenterX;
          const worldZ = y * resolution - mapCenterZ;

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
          position: [0, mapHeight, mapHeight / 2], // Position camera above map center
          fov: 50,
          near: 0.1,
          far: mapHeight * 3,
        }}
      >
        <ambientLight intensity={0.5} />
        <pointLight position={[mapWidth / 2, mapHeight, mapWidth / 2]} />
        <GridCells />
        {robotPose && <Robot pose={robotPose} />}
        <OrbitControls target={[0, 0, 0]} /> {/* Look at map center */}
        <gridHelper
          args={[Math.max(mapWidth, mapHeight), 50]}
          position={[0, -0.1, 0]}
        />
      </Canvas>
    </div>
  );
};

export default MapRenderer;
