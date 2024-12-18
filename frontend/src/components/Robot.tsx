import React, { useRef, useEffect } from "react";
import { Pose } from "../types/Pose";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

const Robot = ({ pose }: { pose: Pose }) => {
  const meshRef = useRef<THREE.Mesh>(null);
  const poseRef = useRef<Pose>(pose);

  useEffect(() => {
    poseRef.current = pose;
  }, [pose]);

  useFrame(() => {
    if (meshRef.current && poseRef.current) {
      const robotX = poseRef.current.data.pose.position.x;
      const robotZ = poseRef.current.data.pose.position.y; // Y yerine Z
      const robotTheta = poseRef.current.data.pose.orientation.z;

      //scale ne olmalı?
        const scale = 0.85

      meshRef.current.position.set(robotX * scale, 0, -robotZ * scale);
      meshRef.current.rotation.set(0, robotTheta, 0); // Rotasyonu düzeltin
    }
  });

  return (
    <group>
      <mesh ref={meshRef}>
        <cylinderGeometry args={[0.15, 0.15, 0.2]} />
        <meshStandardMaterial color="red" />
      </mesh>
    </group>
  );
};

export default Robot;
