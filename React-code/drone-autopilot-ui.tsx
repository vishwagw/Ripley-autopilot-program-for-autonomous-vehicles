import React, { useState, useEffect } from 'react';
import { Map, Battery, Compass, Wifi, Activity, Home, Navigation, ToggleLeft, Play, Square } from 'lucide-react';

export default function DroneAutopilotUI() {
  const [batteryLevel, setBatteryLevel] = useState(85);
  const [altitude, setAltitude] = useState(120);
  const [speed, setSpeed] = useState(15);
  const [heading, setHeading] = useState(45);
  const [connected, setConnected] = useState(true);
  const [flightMode, setFlightMode] = useState("Manual");
  const [isFlying, setIsFlying] = useState(false);
  
  // Simulate drone movement
  useEffect(() => {
    if (isFlying) {
      const interval = setInterval(() => {
        setBatteryLevel(prev => Math.max(prev - 0.1, 0));
        setHeading(prev => (prev + 1) % 360);
        setAltitude(prev => prev + (Math.random() - 0.5) * 2);
        setSpeed(prev => prev + (Math.random() - 0.5));
      }, 1000);
      return () => clearInterval(interval);
    }
  }, [isFlying]);

  const flightModes = ["Manual", "Follow Me", "Waypoint", "Return Home", "Orbit"];

  return (
    <div className="flex flex-col w-full h-screen bg-gray-900 text-white">
      {/* Header */}
      <header className="flex justify-between items-center p-4 bg-gray-800">
        <div className="text-xl font-bold">Drone Autopilot</div>
        <div className="flex items-center space-x-4">
          <div className="flex items-center">
            <Wifi className={connected ? "text-green-500" : "text-red-500"} size={20} />
            <span className="ml-1">{connected ? "Connected" : "Disconnected"}</span>
          </div>
          <div className="flex items-center">
            <Battery className="text-green-500" size={20} />
            <span className="ml-1">{batteryLevel.toFixed(1)}%</span>
          </div>
        </div>
      </header>

      {/* Main content */}
      <div className="flex flex-1">
        {/* Left sidebar - Controls */}
        <div className="w-64 bg-gray-800 p-4 flex flex-col">
          <h2 className="text-lg font-bold mb-4">Flight Controls</h2>
          
          <div className="mb-4">
            <label className="block mb-1">Flight Mode</label>
            <select 
              className="w-full bg-gray-700 text-white p-2 rounded"
              value={flightMode}
              onChange={e => setFlightMode(e.target.value)}
            >
              {flightModes.map(mode => (
                <option key={mode} value={mode}>{mode}</option>
              ))}
            </select>
          </div>
          
          <div className="mb-4">
            <label className="block mb-1">Target Altitude (m)</label>
            <input 
              type="range" 
              min="0" 
              max="400" 
              value={altitude}
              onChange={e => setAltitude(Number(e.target.value))}
              className="w-full"
            />
            <div className="flex justify-between">
              <span>0m</span>
              <span>{altitude}m</span>
              <span>400m</span>
            </div>
          </div>
          
          <div className="mb-4">
            <label className="block mb-1">Speed (m/s)</label>
            <input 
              type="range" 
              min="0" 
              max="30" 
              value={speed}
              onChange={e => setSpeed(Number(e.target.value))}
              className="w-full"
            />
            <div className="flex justify-between">
              <span>0m/s</span>
              <span>{speed.toFixed(1)}m/s</span>
              <span>30m/s</span>
            </div>
          </div>
          
          <div className="flex gap-2 mt-auto">
            <button 
              className={`flex-1 p-2 rounded flex items-center justify-center ${isFlying ? 'bg-red-500' : 'bg-green-500'}`}
              onClick={() => setIsFlying(!isFlying)}
            >
              {isFlying ? <><Square size={16} /> Stop</> : <><Play size={16} /> Start</>}
            </button>
            <button 
              className="flex-1 p-2 rounded bg-blue-500 flex items-center justify-center"
              onClick={() => setFlightMode("Return Home")}
            >
              <Home size={16} /> <span className="ml-1">RTH</span>
            </button>
          </div>
        </div>
        
        {/* Main area - Map */}
        <div className="flex-1 flex flex-col">
          {/* Map area */}
          <div className="flex-1 bg-gray-700 relative">
            <div className="absolute inset-0 flex items-center justify-center text-gray-500">
              <Map size={64} />
              <p className="mt-2">Map View</p>
            </div>
            
            {/* Compass overlay */}
            <div className="absolute top-4 right-4 bg-gray-800 bg-opacity-80 p-2 rounded-full h-24 w-24 flex items-center justify-center">
              <div className="relative h-20 w-20">
                <Compass className="h-20 w-20 text-blue-400" />
                <div 
                  className="absolute top-10 left-10 h-6 w-1 bg-red-500 origin-bottom transform -translate-x-1/2"
                  style={{ transform: `rotate(${heading}deg) translateY(-14px)` }}
                />
                <div className="absolute top-10 left-10 text-xs -mt-4 -ml-3">
                  {heading}°
                </div>
              </div>
            </div>
          </div>
          
          {/* Status bar */}
          <div className="h-16 bg-gray-800 p-2 flex justify-between items-center">
            <div className="flex items-center">
              <Activity className="text-green-500 mr-1" size={20} />
              <span>Status: {isFlying ? "Flying" : "Idle"}</span>
            </div>
            <div>
              <span className="mr-4">Mode: {flightMode}</span>
              <span className="mr-4">Alt: {altitude.toFixed(1)}m</span>
              <span className="mr-4">Speed: {speed.toFixed(1)}m/s</span>
              <span>Heading: {heading}°</span>
            </div>
          </div>
        </div>
        
        {/* Right sidebar - Telemetry */}
        <div className="w-64 bg-gray-800 p-4">
          <h2 className="text-lg font-bold mb-4">Telemetry</h2>
          
          <div className="space-y-4">
            <div className="bg-gray-700 p-3 rounded">
              <h3 className="text-sm text-gray-400">Position</h3>
              <div className="grid grid-cols-2 gap-1 mt-1">
                <div>Lat: 37.7749°</div>
                <div>Lon: -122.4194°</div>
              </div>
            </div>
            
            <div className="bg-gray-700 p-3 rounded">
              <h3 className="text-sm text-gray-400">Velocity</h3>
              <div className="grid grid-cols-3 gap-1 mt-1">
                <div>X: 0.0</div>
                <div>Y: 0.0</div>
                <div>Z: 0.0</div>
              </div>
            </div>
            
            <div className="bg-gray-700 p-3 rounded">
              <h3 className="text-sm text-gray-400">Orientation</h3>
              <div className="grid grid-cols-3 gap-1 mt-1">
                <div>R: 0°</div>
                <div>P: 2°</div>
                <div>Y: {heading}°</div>
              </div>
            </div>
            
            <div className="bg-gray-700 p-3 rounded">
              <h3 className="text-sm text-gray-400">Signal</h3>
              <div className="mt-1">
                <div className="flex justify-between">
                  <span>RC Link:</span>
                  <span className="text-green-500">98%</span>
                </div>
                <div className="flex justify-between">
                  <span>Data Link:</span>
                  <span className="text-green-500">100%</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}