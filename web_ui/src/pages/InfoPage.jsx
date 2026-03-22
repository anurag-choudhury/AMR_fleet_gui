/* eslint-disable no-unused-vars */
import React, { useRef, useContext, useState, useEffect } from "react";

import Camera from "../components/Camera";
import Logs from "../components/Logs";
import CircularProgress from "../components/CircularProgressBar";

import RobotState from "../components/RobotState";
import { RosContext } from "../app/App";
import { AppConfig } from "../shared/constants/index";

const InfoPage = () => {
  const ros = useContext(RosContext);
  const [sensorsData, setSensorsData] = useState("");

  // State for Daly BMS specific data
  const [batteryLevel, setBatteryLevel] = useState(0);
  const [batteryVoltage, setBatteryVoltage] = useState(0.0);
  const [isCharging, setIsCharging] = useState(false);

  // This string MUST be in the format "key:value" for CircularProgress.js
  const [formattedBatteryString, setFormattedBatteryString] = useState("batt1:0");

  const sensorsTopic = useRef(null);
  const batteryTopic = useRef(null);
  const chargeTopic = useRef(null);

  useEffect(() => {
    if (!ros) return;

    // Initialize Topics
    sensorsTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.SENSORS_TOPIC,
      messageType: "std_msgs/String",
    });

    batteryTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.BATTERY_TOPIC,
      messageType: "robotnik_battery_msgs/msg/BatteryStatus",
    });

    chargeTopic.current = new window.ROSLIB.Topic({
      ros,
      name: AppConfig.CHARGE_STATION_CONNECTED,
      messageType: "std_msgs/Bool",
    });

    // Subscriptions
    sensorsTopic.current.subscribe((message) => {
      if (message && message.data) {
        setSensorsData(message.data);
      }
    });

    batteryTopic.current.subscribe((msg) => {
      // Check if msg exists and has the level property
      if (msg && typeof msg.level !== 'undefined') {
        const level = Math.round(msg.level);
        const voltage = msg.voltage || 0.0;
        const chargingStatus = msg.is_charging || false;

        setBatteryLevel(level);
        setBatteryVoltage(voltage);
        setIsCharging(chargingStatus);

        // Crucial: Update the string that CircularProgress parses
        setFormattedBatteryString(`batt1:${level}`);
      }
    });

    chargeTopic.current.subscribe((message) => {
      if (message && typeof message.data !== 'undefined') {
        setIsCharging(message.data);
      }
    });

    return () => {
      if (sensorsTopic.current) sensorsTopic.current.unsubscribe();
      if (batteryTopic.current) batteryTopic.current.unsubscribe();
      if (chargeTopic.current) chargeTopic.current.unsubscribe();
    };
  }, [ros]); // Re-run if ros connection changes

  return (
    <div className="flex flex-col flex-1 min-h-0 bg-slate-950 overflow-hidden">

      {/* Status bar */}
      <div className="flex items-center justify-between px-4 sm:px-6 py-2.5 bg-slate-900 border-b border-slate-800 flex-shrink-0">
        <div className="flex items-center gap-6">
          <h1 className="text-base sm:text-lg font-bold text-white tracking-wide">Robot Info</h1>
          <div className="flex items-center gap-2">
            <span className={`inline-block w-2.5 h-2.5 rounded-full ${isCharging ? 'bg-emerald-500 animate-pulse' : 'bg-slate-500'}`}></span>
            <span className="text-sm text-slate-300">
              {isCharging ? "Charging" : "Discharging"} ({batteryLevel}% | {batteryVoltage.toFixed(1)}V)
            </span>
          </div>
        </div>
      </div>

      {/* Middle: Messages + Camera */}
      <div className="flex flex-1 min-h-0 overflow-hidden">
        <div className="w-56 lg:w-64 flex-shrink-0 flex flex-col border-r border-slate-800 bg-slate-900">
          <div className="px-3 pt-3 pb-1 flex-shrink-0">
            <p className="text-xs font-bold text-slate-400 uppercase tracking-wider">Messages</p>
          </div>
          <div className="flex-1 min-h-0 overflow-auto px-3 pb-3">
            <div className="rounded-md bg-slate-950 p-2 border border-slate-800 text-xs h-full overflow-auto">
              <Logs />
            </div>
          </div>
        </div>

        <div className="flex-1 min-w-0 min-h-0 flex flex-col">
          {/* <div className="flex-1 min-h-0 relative bg-slate-950 p-3">
            <div className="h-full w-full rounded-lg overflow-hidden border border-slate-800">
              <Camera />
            </div>
          </div> */}
          <div className="flex-shrink-0 px-3 pb-2">
            <RobotState />
          </div>
        </div>
      </div>

      {/* Bottom bar — sensors */}
      {/* <div className="flex items-center gap-6 px-4 py-2.5 bg-slate-900 border-t border-slate-800 flex-shrink-0">
        <span className="text-xs font-bold text-slate-400 uppercase tracking-wider whitespace-nowrap">Sensors</span>
        <div className="flex gap-4">
          <CircularProgress
            sensorsData={formattedBatteryString}
            sensorName="batt1"
            color={batteryLevel < 20 ? "#ef4444" : "#ff7a00"}
            text="Battery"
            units="%"
            minValue={0}
            maxValue={100}
          />
        </div>
      </div> */}
    </div>
  );
};

export default InfoPage;