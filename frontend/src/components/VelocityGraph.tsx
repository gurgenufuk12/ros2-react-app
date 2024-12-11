import React, { useState, useEffect } from "react";
import { Line } from "react-chartjs-2";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import "../styles/Controls.css";

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
);

interface VelocityGraphProps {
  linearVelocity: number;
  angularVelocity: number;
}

type DisplayMode = "linear" | "angular" | "both";

const VelocityGraph: React.FC<VelocityGraphProps> = ({
  linearVelocity,
  angularVelocity,
}) => {
  const [displayMode, setDisplayMode] = useState<DisplayMode>("both");
  const [velocityHistory, setVelocityHistory] = useState<{
    linear: number[];
    angular: number[];
  }>({ linear: [], angular: [] });

  const maxDataPoints = 50;

  useEffect(() => {
    setVelocityHistory((prev) => ({
      linear: [...prev.linear, linearVelocity].slice(-maxDataPoints),
      angular: [...prev.angular, angularVelocity].slice(-maxDataPoints),
    }));
  }, [linearVelocity, angularVelocity]);

  const labels = Array.from(
    { length: velocityHistory.linear.length },
    (_, i) => `${i + 1}`
  );

  const data = {
    labels,
    datasets: [
      ...(displayMode === "linear" || displayMode === "both"
        ? [
            {
              label: "Linear Velocity",
              data: velocityHistory.linear,
              borderColor: "rgb(75, 192, 192)",
              tension: 0.1,
            },
          ]
        : []),
      ...(displayMode === "angular" || displayMode === "both"
        ? [
            {
              label: "Angular Velocity",
              data: velocityHistory.angular,
              borderColor: "rgb(255, 99, 132)",
              tension: 0.1,
            },
          ]
        : []),
    ],
  };

  const options = {
    responsive: true,
    maintainAspectRatio: false,
    scales: {
      x: {
        title: {
          display: true,
          text: "Data Points",
        },
      },
      y: {
        title: {
          display: true,
          text:
            displayMode === "both"
              ? "Velocity"
              : displayMode === "linear"
              ? "Linear Velocity (m/s)"
              : "Angular Velocity (rad/s)",
        },
      },
    },
    animation: false as const,
  };

  return (
    <div className="velocity-graph">
      <div className="controls">
        <button
          onClick={() => setDisplayMode("linear")}
          className={displayMode === "linear" ? "active" : ""}
        >
          Linear
        </button>
        <button
          onClick={() => setDisplayMode("angular")}
          className={displayMode === "angular" ? "active" : ""}
        >
          Angular
        </button>
        <button
          onClick={() => setDisplayMode("both")}
          className={displayMode === "both" ? "active" : ""}
        >
          Both
        </button>
        {displayMode === "both" && (
          <div className="note">
            <span>Lineer: {linearVelocity}</span>
            <span>Angular: {angularVelocity}</span>
          </div>
        )}
        {displayMode === "linear" && (
          <div className="note">
            <span>Lineer: {linearVelocity}</span>
          </div>
        )}
        {displayMode === "angular" && (
          <div className="note">
            <span>Angular: {angularVelocity}</span>
          </div>
        )}
      </div>
      <div style={{ height: "400px", width: "100%" }}>
        <Line data={data} options={options} />
      </div>
    </div>
  );
};

export default VelocityGraph;
