import React, { useState, useEffect, useRef } from 'react';
import { 
  Play, 
  Pause, 
  Square, 
  Navigation, 
  Camera, 
  Map, 
  Battery, 
  Wifi, 
  Gauge, 
  Crosshair,
  Home,
  MapPin,
  Route,
  Settings,
  AlertTriangle,
  Radio,
  Compass,
  Plane
} from 'lucide-react';

const DroneGroundStation = () => {
  const [isConnected, setIsConnected] = useState(false);
  const [flightMode, setFlightMode] = useState('STABILIZE');
  const [isArmed, setIsArmed] = useState(false);
  const [activeTab, setActiveTab] = useState('flight');
  const [telemetry, setTelemetry] = useState({
    altitude: 0,
    speed: 0,
    battery: 87,
    gps: { lat: 37.7749, lng: -122.4194, satellites: 12 },
    attitude: { pitch: 0, roll: 0, yaw: 0 },
    signal: 85
  });
  const [waypoints, setWaypoints] = useState([]);
  const [selectedWaypoint, setSelectedWaypoint] = useState(null);
  const [cameraSettings, setCameraSettings] = useState({
    recording: false,
    gimbalTilt: 0,
    gimbalPan: 0,
    zoom: 1
  });

  const mapRef = useRef(null);
  const videoRef = useRef(null);

  // Simulate telemetry updates
  useEffect(() => {
    if (!isConnected) return;
    
    const interval = setInterval(() => {
      setTelemetry(prev => ({
        ...prev,
        altitude: prev.altitude + (Math.random() - 0.5) * 2,
        speed: Math.max(0, prev.speed + (Math.random() - 0.5) * 5),
        battery: Math.max(0, prev.battery - 0.01),
        attitude: {
          pitch: (Math.random() - 0.5) * 20,
          roll: (Math.random() - 0.5) * 20,
          yaw: (prev.attitude.yaw + (Math.random() - 0.5) * 10) % 360
        }
      }));
    }, 1000);

    return () => clearInterval(interval);
  }, [isConnected]);

  const toggleConnection = () => {
    setIsConnected(!isConnected);
  };

  const toggleArm = () => {
    if (isConnected) {
      setIsArmed(!isArmed);
    }
  };

  const addWaypoint = (e) => {
    if (activeTab === 'mission') {
      const rect = e.currentTarget.getBoundingClientRect();
      const x = (e.clientX - rect.left) / rect.width;
      const y = (e.clientY - rect.top) / rect.height;
      
      const newWaypoint = {
        id: Date.now(),
        x: x * 100,
        y: y * 100,
        lat: 37.7749 + (y - 0.5) * 0.01,
        lng: -122.4194 + (x - 0.5) * 0.01,
        altitude: 50,
        action: 'WAYPOINT'
      };
      
      setWaypoints([...waypoints, newWaypoint]);
    }
  };

  const FlightInstruments = () => (
    <div className="grid grid-cols-2 gap-4">
      {/* Attitude Indicator */}
      <div className="bg-gray-800 rounded-lg p-4">
        <h3 className="text-sm font-medium text-gray-300 mb-2">Attitude</h3>
        <div className="relative w-24 h-24 mx-auto bg-gradient-to-b from-blue-500 to-green-500 rounded-full flex items-center justify-center">
          <div 
            className="absolute w-20 h-20 bg-gray-800 rounded-full flex items-center justify-center"
            style={{ transform: `rotate(${telemetry.attitude.roll}deg)` }}
          >
            <div className="w-8 h-0.5 bg-white"></div>
          </div>
          <Plane className="w-6 h-6 text-white z-10" />
        </div>
        <div className="text-xs text-center mt-2">
          <div>P: {telemetry.attitude.pitch.toFixed(1)}°</div>
          <div>R: {telemetry.attitude.roll.toFixed(1)}°</div>
          <div>Y: {telemetry.attitude.yaw.toFixed(1)}°</div>
        </div>
      </div>

      {/* Compass */}
      <div className="bg-gray-800 rounded-lg p-4">
        <h3 className="text-sm font-medium text-gray-300 mb-2">Heading</h3>
        <div className="relative w-24 h-24 mx-auto">
          <div className="absolute inset-0 rounded-full border-2 border-gray-600"></div>
          <div 
            className="absolute inset-2 rounded-full bg-gray-700 flex items-center justify-center"
            style={{ transform: `rotate(${telemetry.attitude.yaw}deg)` }}
          >
            <Compass className="w-6 h-6 text-red-500" />
          </div>
          <div className="absolute top-0 left-1/2 transform -translate-x-1/2 -translate-y-1 text-xs text-white">N</div>
        </div>
        <div className="text-xs text-center mt-2">
          {telemetry.attitude.yaw.toFixed(0)}°
        </div>
      </div>

      {/* Altitude */}
      <div className="bg-gray-800 rounded-lg p-4">
        <h3 className="text-sm font-medium text-gray-300 mb-2">Altitude</h3>
        <div className="text-2xl font-bold text-white">
          {telemetry.altitude.toFixed(1)}m
        </div>
        <div className="text-xs text-gray-400">AGL</div>
      </div>

      {/* Speed */}
      <div className="bg-gray-800 rounded-lg p-4">
        <h3 className="text-sm font-medium text-gray-300 mb-2">Speed</h3>
        <div className="text-2xl font-bold text-white">
          {telemetry.speed.toFixed(1)} m/s
        </div>
        <div className="text-xs text-gray-400">Ground Speed</div>
      </div>
    </div>
  );

  const StatusBar = () => (
    <div className="bg-gray-800 border-b border-gray-700 px-4 py-2 flex items-center justify-between">
      <div className="flex items-center space-x-4">
        <div className="flex items-center space-x-2">
          <div className={`w-3 h-3 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`}></div>
          <span className="text-sm text-gray-300">
            {isConnected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
        <div className="flex items-center space-x-2">
          <Wifi className="w-4 h-4 text-gray-400" />
          <span className="text-sm text-gray-300">{telemetry.signal}%</span>
        </div>
        <div className="flex items-center space-x-2">
          <Battery className="w-4 h-4 text-gray-400" />
          <span className="text-sm text-gray-300">{telemetry.battery.toFixed(0)}%</span>
        </div>
      </div>
      
      <div className="flex items-center space-x-4">
        <span className="text-sm text-gray-300">GPS: {telemetry.gps.satellites} sats</span>
        <span className="text-sm text-gray-300">Mode: {flightMode}</span>
        <div className={`px-2 py-1 rounded text-xs font-medium ${
          isArmed ? 'bg-red-600 text-white' : 'bg-gray-600 text-gray-300'
        }`}>
          {isArmed ? 'ARMED' : 'DISARMED'}
        </div>
      </div>
    </div>
  );

  const CameraView = () => (
    <div className="bg-gray-900 rounded-lg p-4">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-medium text-white">Camera Feed</h3>
        <div className="flex items-center space-x-2">
          <button
            onClick={() => setCameraSettings({...cameraSettings, recording: !cameraSettings.recording})}
            className={`px-3 py-1 rounded text-sm font-medium ${
              cameraSettings.recording 
                ? 'bg-red-600 text-white' 
                : 'bg-gray-600 text-gray-300'
            }`}
          >
            {cameraSettings.recording ? 'Recording' : 'Record'}
          </button>
        </div>
      </div>
      
      <div className="aspect-video bg-gray-800 rounded-lg mb-4 flex items-center justify-center relative">
        <div className="absolute inset-0 bg-gradient-to-br from-blue-900/20 to-green-900/20 rounded-lg"></div>
        <Camera className="w-16 h-16 text-gray-600" />
        <div className="absolute bottom-2 left-2 text-xs text-white bg-black/50 px-2 py-1 rounded">
          Live Feed - {new Date().toLocaleTimeString()}
        </div>
        <div className="absolute top-2 right-2">
          <Crosshair className="w-6 h-6 text-white/50" />
        </div>
      </div>
      
      <div className="grid grid-cols-2 gap-4">
        <div>
          <label className="block text-sm text-gray-400 mb-1">Gimbal Tilt</label>
          <input
            type="range"
            min="-90"
            max="30"
            value={cameraSettings.gimbalTilt}
            onChange={(e) => setCameraSettings({...cameraSettings, gimbalTilt: e.target.value})}
            className="w-full"
          />
          <span className="text-xs text-gray-500">{cameraSettings.gimbalTilt}°</span>
        </div>
        <div>
          <label className="block text-sm text-gray-400 mb-1">Gimbal Pan</label>
          <input
            type="range"
            min="-180"
            max="180"
            value={cameraSettings.gimbalPan}
            onChange={(e) => setCameraSettings({...cameraSettings, gimbalPan: e.target.value})}
            className="w-full"
          />
          <span className="text-xs text-gray-500">{cameraSettings.gimbalPan}°</span>
        </div>
      </div>
    </div>
  );

  const MapView = () => (
    <div className="bg-gray-900 rounded-lg p-4">
      <div className="flex items-center justify-between mb-4">
        <h3 className="text-lg font-medium text-white">Map</h3>
        <div className="flex items-center space-x-2">
          <button className="px-3 py-1 bg-blue-600 text-white rounded text-sm">
            Center on Drone
          </button>
        </div>
      </div>
      
      <div 
        ref={mapRef}
        className="aspect-video bg-gray-800 rounded-lg relative cursor-crosshair"
        onClick={addWaypoint}
      >
        {/* Simulated map background */}
        <div className="absolute inset-0 bg-gradient-to-br from-green-800/20 to-blue-800/20 rounded-lg"></div>
        
        {/* Grid lines */}
        <svg className="absolute inset-0 w-full h-full">
          <defs>
            <pattern id="grid" width="20" height="20" patternUnits="userSpaceOnUse">
              <path d="M 20 0 L 0 0 0 20" fill="none" stroke="rgba(255,255,255,0.1)" strokeWidth="1"/>
            </pattern>
          </defs>
          <rect width="100%" height="100%" fill="url(#grid)" />
        </svg>
        
        {/* Drone position */}
        <div 
          className="absolute w-4 h-4 bg-red-500 rounded-full transform -translate-x-2 -translate-y-2 z-10"
          style={{
            left: '50%',
            top: '50%'
          }}
        >
          <div className="absolute inset-0 bg-red-500 rounded-full animate-ping opacity-75"></div>
        </div>
        
        {/* Waypoints */}
        {waypoints.map((waypoint, index) => (
          <div
            key={waypoint.id}
            className="absolute w-3 h-3 bg-blue-500 rounded-full transform -translate-x-1.5 -translate-y-1.5 cursor-pointer"
            style={{
              left: `${waypoint.x}%`,
              top: `${waypoint.y}%`
            }}
            onClick={(e) => {
              e.stopPropagation();
              setSelectedWaypoint(waypoint);
            }}
          >
            <span className="absolute -top-6 -left-2 text-xs text-white bg-black/50 px-1 rounded">
              {index + 1}
            </span>
          </div>
        ))}
        
        {/* Flight path */}
        {waypoints.length > 0 && (
          <svg className="absolute inset-0 w-full h-full pointer-events-none">
            <polyline
              points={waypoints.map(w => `${w.x},${w.y}`).join(' ')}
              fill="none"
              stroke="rgba(59, 130, 246, 0.8)"
              strokeWidth="2"
              strokeDasharray="5,5"
            />
          </svg>
        )}
        
        <div className="absolute bottom-2 left-2 text-xs text-white bg-black/50 px-2 py-1 rounded">
          Lat: {telemetry.gps.lat.toFixed(6)}, Lng: {telemetry.gps.lng.toFixed(6)}
        </div>
      </div>
    </div>
  );

  const ControlPanel = () => (
    <div className="space-y-4">
      <div className="bg-gray-800 rounded-lg p-4">
        <h3 className="text-lg font-medium text-white mb-4">Flight Controls</h3>
        
        <div className="grid grid-cols-2 gap-4 mb-4">
          <button
            onClick={toggleConnection}
            className={`px-4 py-2 rounded font-medium ${
              isConnected 
                ? 'bg-red-600 hover:bg-red-700 text-white' 
                : 'bg-green-600 hover:bg-green-700 text-white'
            }`}
          >
            <Radio className="w-4 h-4 inline mr-2" />
            {isConnected ? 'Disconnect' : 'Connect'}
          </button>
          
          <button
            onClick={toggleArm}
            disabled={!isConnected}
            className={`px-4 py-2 rounded font-medium disabled:opacity-50 ${
              isArmed 
                ? 'bg-red-600 hover:bg-red-700 text-white' 
                : 'bg-orange-600 hover:bg-orange-700 text-white'
            }`}
          >
            <AlertTriangle className="w-4 h-4 inline mr-2" />
            {isArmed ? 'Disarm' : 'Arm'}
          </button>
        </div>
        
        <div className="mb-4">
          <label className="block text-sm text-gray-400 mb-2">Flight Mode</label>
          <select
            value={flightMode}
            onChange={(e) => setFlightMode(e.target.value)}
            className="w-full bg-gray-700 text-white rounded px-3 py-2 border border-gray-600"
          >
            <option value="STABILIZE">Stabilize</option>
            <option value="LOITER">Loiter</option>
            <option value="AUTO">Auto</option>
            <option value="RTL">Return to Launch</option>
            <option value="GUIDED">Guided</option>
          </select>
        </div>
        
        <div className="grid grid-cols-3 gap-2">
          <button className="px-3 py-2 bg-blue-600 hover:bg-blue-700 text-white rounded text-sm">
            <Play className="w-4 h-4 inline mr-1" />
            Takeoff
          </button>
          <button className="px-3 py-2 bg-yellow-600 hover:bg-yellow-700 text-white rounded text-sm">
            <Pause className="w-4 h-4 inline mr-1" />
            Pause
          </button>
          <button className="px-3 py-2 bg-red-600 hover:bg-red-700 text-white rounded text-sm">
            <Home className="w-4 h-4 inline mr-1" />
            RTL
          </button>
        </div>
      </div>
      
      <FlightInstruments />
    </div>
  );

  const MissionPlanner = () => (
    <div className="space-y-4">
      <div className="bg-gray-800 rounded-lg p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-lg font-medium text-white">Mission Planner</h3>
          <div className="flex space-x-2">
            <button 
              onClick={() => setWaypoints([])}
              className="px-3 py-1 bg-red-600 hover:bg-red-700 text-white rounded text-sm"
            >
              Clear
            </button>
            <button className="px-3 py-1 bg-green-600 hover:bg-green-700 text-white rounded text-sm">
              Upload
            </button>
          </div>
        </div>
        
        <div className="text-sm text-gray-400 mb-4">
          Click on the map to add waypoints. Total: {waypoints.length} waypoints
        </div>
        
        {waypoints.length > 0 && (
          <div className="space-y-2 max-h-48 overflow-y-auto">
            {waypoints.map((waypoint, index) => (
              <div key={waypoint.id} className="flex items-center justify-between bg-gray-700 p-2 rounded">
                <div className="flex items-center space-x-2">
                  <MapPin className="w-4 h-4 text-blue-400" />
                  <span className="text-sm text-white">WP{index + 1}</span>
                  <span className="text-xs text-gray-400">
                    {waypoint.lat.toFixed(6)}, {waypoint.lng.toFixed(6)}
                  </span>
                </div>
                <button
                  onClick={() => setWaypoints(waypoints.filter(w => w.id !== waypoint.id))}
                  className="text-red-400 hover:text-red-300"
                >
                  ×
                </button>
              </div>
            ))}
          </div>
        )}
      </div>
      
      {selectedWaypoint && (
        <div className="bg-gray-800 rounded-lg p-4">
          <h4 className="text-md font-medium text-white mb-3">Waypoint Settings</h4>
          <div className="space-y-3">
            <div>
              <label className="block text-sm text-gray-400 mb-1">Altitude (m)</label>
              <input
                type="number"
                value={selectedWaypoint.altitude}
                onChange={(e) => setSelectedWaypoint({...selectedWaypoint, altitude: e.target.value})}
                className="w-full bg-gray-700 text-white rounded px-3 py-2 border border-gray-600"
              />
            </div>
            <div>
              <label className="block text-sm text-gray-400 mb-1">Action</label>
              <select
                value={selectedWaypoint.action}
                onChange={(e) => setSelectedWaypoint({...selectedWaypoint, action: e.target.value})}
                className="w-full bg-gray-700 text-white rounded px-3 py-2 border border-gray-600"
              >
                <option value="WAYPOINT">Waypoint</option>
                <option value="LOITER">Loiter</option>
                <option value="LAND">Land</option>
                <option value="TAKEOFF">Takeoff</option>
              </select>
            </div>
          </div>
        </div>
      )}
    </div>
  );

  return (
    <div className="min-h-screen bg-gray-900 text-white">
      <StatusBar />
      
      <div className="flex h-screen">
        {/* Main Content */}
        <div className="flex-1 p-4">
          <div className="grid grid-cols-2 gap-4 h-full">
            {/* Left Column - Map/Camera */}
            <div className="space-y-4">
              <div className="flex space-x-2 mb-4">
                <button
                  onClick={() => setActiveTab('flight')}
                  className={`px-4 py-2 rounded font-medium ${
                    activeTab === 'flight' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
                  }`}
                >
                  <Map className="w-4 h-4 inline mr-2" />
                  Flight
                </button>
                <button
                  onClick={() => setActiveTab('camera')}
                  className={`px-4 py-2 rounded font-medium ${
                    activeTab === 'camera' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
                  }`}
                >
                  <Camera className="w-4 h-4 inline mr-2" />
                  Camera
                </button>
                <button
                  onClick={() => setActiveTab('mission')}
                  className={`px-4 py-2 rounded font-medium ${
                    activeTab === 'mission' ? 'bg-blue-600 text-white' : 'bg-gray-700 text-gray-300'
                  }`}
                >
                  <Route className="w-4 h-4 inline mr-2" />
                  Mission
                </button>
              </div>
              
              {activeTab === 'flight' && <MapView />}
              {activeTab === 'camera' && <CameraView />}
              {activeTab === 'mission' && <MapView />}
            </div>
            
            {/* Right Column - Controls */}
            <div>
              {activeTab === 'mission' ? <MissionPlanner /> : <ControlPanel />}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DroneGroundStation;