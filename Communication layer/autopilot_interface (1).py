#!/usr/bin/env python3
"""
Autopilot Interface for Autonomous Drone

This module interfaces with the drone flight controller (Pixhawk) using the
DroneKit API to provide high-level control functions and telemetry.
"""

import os
import json
import time
import math
import logging
from typing import Dict, List, Any, Callable, Optional, Union, Tuple
from datetime import datetime

# Import DroneKit for drone communication
try:
    from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
    from pymavlink import mavutil
    DRONEKIT_AVAILABLE = True
except ImportError:
    DRONEKIT_AVAILABLE = False
    logging.warning("DroneKit not available. Running in simulation mode.")

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('autopilot')


class MissionWaypoint:
    """Waypoint in a mission."""
    
    def __init__(self, 
                 lat: float, 
                 lon: float, 
                 alt: float, 
                 action: Optional[str] = None,
                 hover_time: float = 0.0,
                 heading: Optional[float] = None):
        """
        Initialize a waypoint.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home position)
            action: Optional action to perform at waypoint (e.g., 'take_photo')
            hover_time: Time to hover at waypoint in seconds
            heading: Optional heading in degrees (0-360, 0=North)
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.action = action
        self.hover_time = hover_time
        self.heading = heading
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert waypoint to dictionary."""
        waypoint_dict = {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt
        }
        
        if self.action:
            waypoint_dict['action'] = self.action
        
        if self.hover_time > 0:
            waypoint_dict['hover_time'] = self.hover_time
            
        if self.heading is not None:
            waypoint_dict['heading'] = self.heading
            
        return waypoint_dict
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'MissionWaypoint':
        """Create waypoint from dictionary."""
        return cls(
            lat=data['lat'],
            lon=data['lon'],
            alt=data['alt'],
            action=data.get('action'),
            hover_time=data.get('hover_time', 0.0),
            heading=data.get('heading')
        )


class Mission:
    """Mission consisting of waypoints."""
    
    def __init__(self, name: str):
        """
        Initialize a mission.
        
        Args:
            name: Mission name
        """
        self.name = name
        self.waypoints: List[MissionWaypoint] = []
        self.created_at = datetime.now().isoformat()
        self.modified_at = self.created_at
    
    def add_waypoint(self, waypoint: MissionWaypoint) -> None:
        """Add a waypoint to the mission."""
        self.waypoints.append(waypoint)
        self.modified_at = datetime.now().isoformat()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert mission to dictionary."""
        return {
            'name': self.name,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'created_at': self.created_at,
            'modified_at': self.modified_at
        }
    
    def save(self, directory: str) -> bool:
        """
        Save mission to a JSON file.
        
        Args:
            directory: Directory to save to
            
        Returns:
            True if successful, False otherwise
        """
        try:
            filename = os.path.join(directory, f"{self.name}.json")
            with open(filename, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
            return True
        except Exception as e:
            logger.error(f"Error saving mission: {str(e)}")
            return False
    
    @classmethod
    def load(cls, filename: str) -> Optional['Mission']:
        """
        Load mission from a JSON file.
        
        Args:
            filename: Path to JSON file
            
        Returns:
            Mission object, or None if loading failed
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            mission = cls(data['name'])
            mission.created_at = data['created_at']
            mission.modified_at = data['modified_at']
            
            for wp_data in data['waypoints']:
                waypoint = MissionWaypoint.from_dict(wp_data)
                mission.add_waypoint(waypoint)
            
            return mission
        
        except Exception as e:
            logger.error(f"Error loading mission: {str(e)}")
            return None


class AutopilotSystem:
    """Interface to the drone's autopilot system."""
    
    def __init__(self, connection_string: str):
        """
        Initialize the autopilot interface.
        
        Args:
            connection_string: Connection string for the vehicle
                (e.g. '/dev/ttyAMA0' for serial, 'tcp:127.0.0.1:5760' for SITL)
        """
        self.connection_string = connection_string
        self.vehicle = None
        self.is_connected = False
        self.current_mission: Optional[Mission] = None
        self.mission_running = False
        self.event_callbacks: Dict[str, List[Callable]] = {}
        
        # Initialize event callback lists
        events = [
            'position_changed', 'attitude_changed', 'mode_changed', 
            'armed_changed', 'mission_started', 'mission_completed',
            'waypoint_reached', 'battery_changed', 'status_changed'
        ]
        for event in events:
            self.event_callbacks[event] = []
        
        # Last values for change detection
        self._last_position: Optional[Tuple[float, float, float]] = None
        self._last_attitude: Optional[Tuple[float, float, float]] = None
        self._last_mode: Optional[str] = None
        self._last_armed: Optional[bool] = None
        self._last_battery: Optional[float] = None
        
        # For simulation mode
        self._sim_home_position = (47.6062, -122.3321, 0)  # Seattle, WA
        self._sim_current_position = self._sim_home_position
        self._sim_attitude = (0.0, 0.0, 0.0)  # Roll, pitch, yaw in radians
        self._sim_armed = False
        self._sim_mode = "STABILIZE"
        self._sim_battery = 100.0
        self._sim_airspeed = 0.0
        self._sim_groundspeed = 0.0
        self._sim_heading = 0
        self._sim_last_heartbeat = time.time()
    
    def register_event_callback(self, event: str, callback: Callable) -> bool:
        """
        Register a callback for an event.
        
        Args:
            event: Event name
            callback: Callback function
            
        Returns:
            True if successful, False otherwise
        """
        if event in self.event_callbacks:
            self.event_callbacks[event].append(callback)
            return True
        return False
    
    def _trigger_event(self, event: str, data: Any = None) -> None:
        """
        Trigger an event.
        
        Args:
            event: Event name
            data: Event data
        """
        if event in self.event_callbacks:
            for callback in self.event_callbacks[event]:
                try:
                    callback(data)
                except Exception as e:
                    logger.error(f"Error in {event} callback: {str(e)}")
    
    def start(self) -> bool:
        """
        Start the autopilot interface.
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if DRONEKIT_AVAILABLE:
                logger.info(f"Connecting to vehicle on {self.connection_string}")
                self.vehicle = connect(self.connection_string, wait_ready=True, baud=57600)
                
                # Set up listeners
                self.vehicle.add_attribute_listener('location.global_relative_frame', 
                                                   self._location_callback)
                self.vehicle.add_attribute_listener('attitude', 
                                                   self._attitude_callback)
                self.vehicle.add_attribute_listener('mode', 
                                                   self._mode_callback)
                self.vehicle.add_attribute_listener('armed', 
                                                   self._armed_callback)
                self.vehicle.add_attribute_listener('battery', 
                                                   self._battery_callback)
                
                self.is_connected = True
                logger.info("Connected to vehicle")
                
                # Trigger initial status
                self._trigger_event('status_changed', self.get_status())
                
                return True
            else:
                # Simulation mode
                logger.info("Starting in simulation mode")
                self.is_connected = True
                return True
        
        except Exception as e:
            logger.error(f"Error connecting to vehicle: {str(e)}")
            return False
    
    def stop(self) -> None:
        """Stop the autopilot interface."""
        try:
            if self.is_connected and DRONEKIT_AVAILABLE and self.vehicle:
                self.vehicle.close()
                logger.info("Disconnected from vehicle")
        except Exception as e:
            logger.error(f"Error disconnecting from vehicle: {str(e)}")
        finally:
            self.is_connected = False
            self.vehicle = None
    
    def _location_callback(self, attr_name, value) -> None:
        """Handle location change events from DroneKit."""
        lat = value.lat
        lon = value.lon
        alt = value.alt
        
        # Check if position has changed significantly
        if (self._last_position is None or
            abs(lat - self._last_position[0]) > 0.0000001 or
            abs(lon - self._last_position[1]) > 0.0000001 or
            abs(alt - self._last_position[2]) > 0.01):
            
            self._last_position = (lat, lon, alt)
            
            # Trigger event
            self._trigger_event('position_changed', {
                'lat': lat,
                'lon': lon,
                'alt': alt
            })
    
    def _attitude_callback(self, attr_name, value) -> None:
        """Handle attitude change events from DroneKit."""
        roll = math.degrees(value.roll)
        pitch = math.degrees(value.pitch)
        yaw = math.degrees(value.yaw)
        
        # Check if attitude has changed significantly
        if (self._last_attitude is None or
            abs(roll - self._last_attitude[0]) > 0.5 or
            abs(pitch - self._last_attitude[1]) > 0.5 or
            abs(yaw - self._last_attitude[2]) > 0.5):
            
            self._last_attitude = (roll, pitch, yaw)
            
            # Trigger event
            self._trigger_event('attitude_changed', {
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'heading': (yaw + 360) % 360  # Convert to 0-360 heading
            })
    
    def _mode_callback(self, attr_name, value) -> None:
        """Handle mode change events from DroneKit."""
        mode = value.name
        
        # Check if mode has changed
        if self._last_mode != mode:
            self._last_mode = mode
            
            # Trigger event
            self._trigger_event('mode_changed', {
                'mode': mode
            })
    
    def _armed_callback(self, attr_name, value) -> None:
        """Handle armed state change events from DroneKit."""
        armed = value
        
        # Check if armed state has changed
        if self._last_armed != armed:
            self._last_armed = armed
            
            # Trigger event
            self._trigger_event('armed_changed', {
                'armed': armed
            })
    
    def _battery_callback(self, attr_name, value) -> None:
        """Handle battery state change events from DroneKit."""
        if value is None:
            return
            
        voltage = value.voltage
        current = value.current if hasattr(value, 'current') else None
        level = value.level if hasattr(value, 'level') else None
        
        # If level is not available, estimate it based on voltage
        if level is None and voltage is not None:
            # Simple estimation based on typical LiPo discharge curve
            # Assumes 3.7V per cell nominal, 4.2V full, 3.0V empty
            # This is a rough estimate and should be calibrated for actual battery
            cell_count = 3  # Assume 3S battery
            max_voltage = 4.2 * cell_count
            min_voltage = 3.0 * cell_count
            
            if voltage >= max_voltage:
                level = 100.0
            elif voltage <= min_voltage:
                level = 0.0
            else:
                level = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0
        
        # Check if battery state has changed significantly
        if (self._last_battery is None or
            level is None or
            abs(level - self._last_battery) > 1.0):
            
            if level is not None:
                self._last_battery = level
            
            # Trigger event
            self._trigger_event('battery_changed', {
                'voltage': voltage,
                'current': current,
                'level': level
            })
    
    def get_status(self) -> Dict[str, Any]:
        """
        Get the current status of the drone.
        
        Returns:
            Dictionary with status information
        """
        if DRONEKIT_AVAILABLE and self.vehicle:
            # Get data from actual vehicle
            position = self.vehicle.location.global_relative_frame
            attitude = self.vehicle.attitude
            battery = self.vehicle.battery
            
            return {
                'connected': self.is_connected,
                'armed': self.vehicle.armed,
                'mode': self.vehicle.mode.name,
                'position': {
                    'lat': position.lat if position else None,
                    'lon': position.lon if position else None,
                    'alt': position.alt if position else None
                },
                'attitude': {
                    'roll': math.degrees(attitude.roll),
                    'pitch': math.degrees(attitude.pitch),
                    'yaw': math.degrees(attitude.yaw),
                    'heading': (math.degrees(attitude.yaw) + 360) % 360
                },
                'speed': {
                    'airspeed': self.vehicle.airspeed,
                    'groundspeed': self.vehicle.groundspeed
                },
                'battery': {
                    'voltage': battery.voltage if battery else None,
                    'current': battery.current if hasattr(battery, 'current') else None,
                    'level': battery.level if hasattr(battery, 'level') else None
                },
                'gps': {
                    'fix': self.vehicle.gps_0.fix_type,
                    'satellites': self.vehicle.gps_0.satellites_visible
                },
                'system': {
                    'last_heartbeat': self.vehicle.last_heartbeat,
                    'ekf_ok': self.vehicle.ekf_ok if hasattr(self.vehicle, 'ekf_ok') else None
                },
                'mission': {
                    'running': self.mission_running,
                    'name': self.current_mission.name if self.current_mission else None,
                    'waypoints': len(self.current_mission.waypoints) if self.current_mission else 0
                }
            }
        else:
            # Return simulated data
            return {
                'connected': self.is_connected,
                'armed': self._sim_armed,
                'mode': self._sim_mode,
                'position': {
                    'lat': self._sim_current_position[0],
                    'lon': self._sim_current_position[1],
                    'alt': self._sim_current_position[2]
                },
                'attitude': {
                    'roll': math.degrees(self._sim_attitude[0]),
                    'pitch': math.degrees(self._sim_attitude[1]),
                    'yaw': math.degrees(self._sim_attitude[2]),
                    'heading': (math.degrees(self._sim_attitude[2]) + 360) % 360
                },
                'speed': {
                    'airspeed': self._sim_airspeed,
                    'groundspeed': self._sim_groundspeed
                },
                'battery': {
                    'voltage': 11.5,  # Simulated 3S LiPo
                    'current': 5.0,  # Simulated current draw
                    'level': self._sim_battery
                },
                'gps': {
                    'fix': 3,  # 3D fix
                    'satellites': 10
                },
                'system': {
                    'last_heartbeat': time.time() - self._sim_last_heartbeat,
                    'ekf_ok': True
                },
                'mission': {
                    'running': self.mission_running,
                    'name': self.current_mission.name if self.current_mission else None,
                    'waypoints': len(self.current_mission.waypoints) if self.current_mission else 0
                }
            }
    
    def arm(self) -> bool:
        """
        Arm the drone.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Check if already armed
                if self.vehicle.armed:
                    return True
                
                # Arm vehicle
                self.vehicle.armed = True
                
                # Wait for commands to clear
                time.sleep(1)
                
                # Add home location
                cmds.add(Command(
                    0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 
                    0, 0, 10))  # Take off to 10m
                
                # Add mission waypoints
                for i, wp in enumerate(mission.waypoints):
                    # Add waypoint command
                    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
                    command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
                    
                    # Parameters
                    param1 = wp.hover_time  # Hold time in seconds
                    param2 = 1.0  # Acceptance radius in meters
                    param3 = 0.0  # Pass through waypoint
                    param4 = wp.heading if wp.heading is not None else float('nan')  # Desired yaw angle
                    
                    cmds.add(Command(
                        0, 0, 0, frame, command, 0, 0, 
                        param1, param2, param3, param4, 
                        wp.lat, wp.lon, wp.alt))
                    
                    # Add do_action command if specified
                    if wp.action:
                        if wp.action == 'take_photo':
                            # Trigger camera
                            cmds.add(Command(
                                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_DO_DIGICAM_CONTROL, 
                                0, 0, 0, 0, 0, 0, 0, 0, 0))
                        elif wp.action == 'start_video':
                            # Start video recording
                            cmds.add(Command(
                                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE, 
                                0, 0, 0, 0, 0, 0, 0, 0, 0))
                        elif wp.action == 'stop_video':
                            # Stop video recording
                            cmds.add(Command(
                                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_VIDEO_STOP_CAPTURE, 
                                0, 0, 0, 0, 0, 0, 0, 0, 0))
                        elif wp.action == 'set_servo':
                            # Set servo (generic)
                            cmds.add(Command(
                                0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
                                0, 0, 10, 1500, 0, 0, 0, 0, 0))  # Servo channel 10, PWM 1500
                
                # Add RTL as final command
                cmds.add(Command(
                    0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 
                    0, 0, 0))
                
                # Upload the commands
                cmds.upload()
                logger.info(f"Uploaded mission with {len(mission.waypoints)} waypoints")
                
                return True
            else:
                # Simulation mode
                logger.info(f"Loaded mission with {len(mission.waypoints)} waypoints (simulated)")
                return True
        
        except Exception as e:
            logger.error(f"Error loading mission: {str(e)}")
            return False
    
    def start_mission(self) -> bool:
        """
        Start the loaded mission.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected or not self.current_mission:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Check if armed
                if not self.vehicle.armed:
                    logger.warning("Cannot start mission: Vehicle not armed")
                    return False
                
                # Set mode to AUTO to start mission
                self.vehicle.mode = VehicleMode("AUTO")
                
                # Wait for mode change
                for _ in range(30):  # Wait up to 3 seconds
                    if self.vehicle.mode.name == "AUTO":
                        break
                    time.sleep(0.1)
                
                if self.vehicle.mode.name != "AUTO":
                    logger.warning("Failed to enter AUTO mode")
                    return False
                
                self.mission_running = True
                logger.info("Mission started")
                
                # Trigger event
                self._trigger_event('mission_started', {
                    'name': self.current_mission.name,
                    'waypoints': len(self.current_mission.waypoints)
                })
                
                return True
            else:
                # Simulation mode
                logger.info("Mission started (simulated)")
                
                # Set mode
                self._sim_mode = "AUTO"
                self._trigger_event('mode_changed', {'mode': "AUTO"})
                
                self.mission_running = True
                
                # Trigger event
                self._trigger_event('mission_started', {
                    'name': self.current_mission.name,
                    'waypoints': len(self.current_mission.waypoints)
                })
                
                # Simulate mission execution
                def sim_mission():
                    # First arm if not armed
                    if not self._sim_armed:
                        self._sim_armed = True
                        self._trigger_event('armed_changed', {'armed': True})
                    
                    # Take off
                    takeoff_alt = 10.0
                    current_alt = self._sim_current_position[2]
                    while current_alt < takeoff_alt:
                        time.sleep(0.1)
                        current_alt = min(takeoff_alt, current_alt + 0.5)
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Visit each waypoint
                    for i, wp in enumerate(self.current_mission.waypoints):
                        # Move to waypoint
                        target_lat, target_lon, target_alt = wp.lat, wp.lon, wp.alt
                        current_lat, current_lon = self._sim_current_position[0], self._sim_current_position[1]
                        
                        # Move to waypoint altitude first
                        while abs(current_alt - target_alt) > 0.5:
                            time.sleep(0.1)
                            if current_alt < target_alt:
                                current_alt = min(target_alt, current_alt + 0.5)
                            else:
                                current_alt = max(target_alt, current_alt - 0.5)
                            
                            self._sim_current_position = (
                                self._sim_current_position[0],
                                self._sim_current_position[1],
                                current_alt
                            )
                            self._trigger_event('position_changed', {
                                'lat': self._sim_current_position[0],
                                'lon': self._sim_current_position[1],
                                'alt': self._sim_current_position[2]
                            })
                        
                        # Move to waypoint position
                        while (abs(current_lat - target_lat) > 0.00001 or 
                               abs(current_lon - target_lon) > 0.00001):
                            time.sleep(0.1)
                            
                            # Calculate direction vector
                            dlat = target_lat - current_lat
                            dlon = target_lon - current_lon
                            
                            # Normalize and scale for speed
                            dist = math.sqrt(dlat**2 + dlon**2)
                            if dist > 0:
                                speed = 0.00001  # Speed in degrees/iteration
                                dlat = dlat / dist * min(speed, dist)
                                dlon = dlon / dist * min(speed, dist)
                            
                            # Update position
                            current_lat += dlat
                            current_lon += dlon
                            
                            self._sim_current_position = (
                                current_lat,
                                current_lon,
                                current_alt
                            )
                            self._trigger_event('position_changed', {
                                'lat': current_lat,
                                'lon': current_lon,
                                'alt': current_alt
                            })
                        
                        # Reached waypoint
                        self._trigger_event('waypoint_reached', {
                            'index': i,
                            'lat': target_lat,
                            'lon': target_lon,
                            'alt': target_alt,
                            'action': wp.action
                        })
                        
                        # Perform action if specified
                        if wp.action:
                            logger.info(f"Performing action: {wp.action}")
                            # Simulate action execution
                            time.sleep(1.0)
                        
                        # Hover if specified
                        if wp.hover_time > 0:
                            time.sleep(wp.hover_time)
                    
                    # Mission completed, return to launch
                    self._trigger_event('mission_completed', {
                        'name': self.current_mission.name,
                        'waypoints': len(self.current_mission.waypoints)
                    })
                    
                    # Return to home
                    home_lat, home_lon = self._sim_home_position[0], self._sim_home_position[1]
                    
                    # First go to safe altitude
                    safe_alt = 15.0
                    while abs(current_alt - safe_alt) > 0.5:
                        time.sleep(0.1)
                        if current_alt < safe_alt:
                            current_alt = min(safe_alt, current_alt + 0.5)
                        else:
                            current_alt = max(safe_alt, current_alt - 0.5)
                        
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Move to home position
                    while (abs(current_lat - home_lat) > 0.00001 or 
                           abs(current_lon - home_lon) > 0.00001):
                        time.sleep(0.1)
                        
                        # Calculate direction vector
                        dlat = home_lat - current_lat
                        dlon = home_lon - current_lon
                        
                        # Normalize and scale for speed
                        dist = math.sqrt(dlat**2 + dlon**2)
                        if dist > 0:
                            speed = 0.00001  # Speed in degrees/iteration
                            dlat = dlat / dist * min(speed, dist)
                            dlon = dlon / dist * min(speed, dist)
                        
                        # Update position
                        current_lat += dlat
                        current_lon += dlon
                        
                        self._sim_current_position = (
                            current_lat,
                            current_lon,
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': current_lat,
                            'lon': current_lon,
                            'alt': current_alt
                        })
                    
                    # Land
                    while current_alt > 0:
                        time.sleep(0.1)
                        current_alt = max(0, current_alt - 0.3)
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Mission completed
                    self.mission_running = False
                    self._sim_armed = False
                    self._sim_mode = "STABILIZE"
                    self._trigger_event('armed_changed', {'armed': False})
                    self._trigger_event('mode_changed', {'mode': "STABILIZE"})
                
                # Start mission simulation in a thread
                import threading
                thread = threading.Thread(target=sim_mission)
                thread.daemon = True
                thread.start()
                
                return True
        
        except Exception as e:
            logger.error(f"Error starting mission: {str(e)}")
            return False
    
    def stop_mission(self) -> bool:
        """
        Stop the current mission.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Set mode to LOITER to stop mission
                self.vehicle.mode = VehicleMode("LOITER")
                
                # Wait for mode change
                for _ in range(30):  # Wait up to 3 seconds
                    if self.vehicle.mode.name == "LOITER":
                        break
                    time.sleep(0.1)
                
                if self.vehicle.mode.name != "LOITER":
                    logger.warning("Failed to enter LOITER mode")
                    return False
                
                self.mission_running = False
                logger.info("Mission stopped")
                return True
            else:
                # Simulation mode
                logger.info("Mission stopped (simulated)")
                self._sim_mode = "LOITER"
                self._trigger_event('mode_changed', {'mode': "LOITER"})
                self.mission_running = False
                return True
        
        except Exception as e:
            logger.error(f"Error stopping mission: {str(e)}")
            return False
 for arming
                for _ in range(30):  # Wait up to 3 seconds
                    if self.vehicle.armed:
                        logger.info("Vehicle armed")
                        return True
                    time.sleep(0.1)
                
                logger.warning("Vehicle failed to arm")
                return False
            else:
                # Simulation mode
                logger.info("Arming vehicle (simulated)")
                self._sim_armed = True
                self._trigger_event('armed_changed', {'armed': True})
                return True
        
        except Exception as e:
            logger.error(f"Error arming vehicle: {str(e)}")
            return False
    
    def disarm(self) -> bool:
        """
        Disarm the drone.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Check if already disarmed
                if not self.vehicle.armed:
                    return True
                
                # Disarm vehicle
                self.vehicle.armed = False
                
                # Wait for disarming
                for _ in range(30):  # Wait up to 3 seconds
                    if not self.vehicle.armed:
                        logger.info("Vehicle disarmed")
                        return True
                    time.sleep(0.1)
                
                logger.warning("Vehicle failed to disarm")
                return False
            else:
                # Simulation mode
                logger.info("Disarming vehicle (simulated)")
                self._sim_armed = False
                self._trigger_event('armed_changed', {'armed': False})
                return True
        
        except Exception as e:
            logger.error(f"Error disarming vehicle: {str(e)}")
            return False
    
    def takeoff(self, altitude: float) -> bool:
        """
        Command the drone to take off.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Check if armed
                if not self.vehicle.armed:
                    logger.warning("Cannot takeoff: Vehicle not armed")
                    return False
                
                # Set GUIDED mode
                self.vehicle.mode = VehicleMode("GUIDED")
                
                # Wait for mode change
                for _ in range(30):  # Wait up to 3 seconds
                    if self.vehicle.mode.name == "GUIDED":
                        break
                    time.sleep(0.1)
                
                if self.vehicle.mode.name != "GUIDED":
                    logger.warning("Failed to enter GUIDED mode")
                    return False
                
                # Command takeoff
                self.vehicle.simple_takeoff(altitude)
                logger.info(f"Taking off to {altitude} meters")
                
                return True
            else:
                # Simulation mode
                logger.info(f"Taking off to {altitude} meters (simulated)")
                
                # Set mode
                self._sim_mode = "GUIDED"
                self._trigger_event('mode_changed', {'mode': "GUIDED"})
                
                # Simulate rising
                def sim_takeoff():
                    current_alt = self._sim_current_position[2]
                    while current_alt < altitude:
                        time.sleep(0.1)
                        current_alt = min(altitude, current_alt + 0.2)  # Rise at 2 m/s
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                
                # Start takeoff simulation in a thread
                import threading
                thread = threading.Thread(target=sim_takeoff)
                thread.daemon = True
                thread.start()
                
                return True
        
        except Exception as e:
            logger.error(f"Error during takeoff: {str(e)}")
            return False
    
    def land(self) -> bool:
        """
        Command the drone to land.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Set LAND mode
                self.vehicle.mode = VehicleMode("LAND")
                logger.info("Landing")
                return True
            else:
                # Simulation mode
                logger.info("Landing (simulated)")
                
                # Set mode
                self._sim_mode = "LAND"
                self._trigger_event('mode_changed', {'mode': "LAND"})
                
                # Simulate descent
                def sim_land():
                    current_alt = self._sim_current_position[2]
                    while current_alt > 0:
                        time.sleep(0.1)
                        current_alt = max(0, current_alt - 0.5)  # Descend at 5 m/s
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Disarm after landing
                    self._sim_armed = False
                    self._trigger_event('armed_changed', {'armed': False})
                
                # Start landing simulation in a thread
                import threading
                thread = threading.Thread(target=sim_land)
                thread.daemon = True
                thread.start()
                
                return True
        
        except Exception as e:
            logger.error(f"Error during landing: {str(e)}")
            return False
    
    def return_to_launch(self) -> bool:
        """
        Command the drone to return to launch position.
        
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Set RTL mode
                self.vehicle.mode = VehicleMode("RTL")
                logger.info("Returning to launch")
                return True
            else:
                # Simulation mode
                logger.info("Returning to launch (simulated)")
                
                # Set mode
                self._sim_mode = "RTL"
                self._trigger_event('mode_changed', {'mode': "RTL"})
                
                # Simulate RTL
                def sim_rtl():
                    # First climb to safe altitude if needed
                    safe_alt = 15.0  # 15 meters safe RTL altitude
                    current_alt = self._sim_current_position[2]
                    
                    # Climb if needed
                    while current_alt < safe_alt:
                        time.sleep(0.1)
                        current_alt = min(safe_alt, current_alt + 0.5)
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Return to home position
                    home_lat, home_lon = self._sim_home_position[0], self._sim_home_position[1]
                    current_lat, current_lon = self._sim_current_position[0], self._sim_current_position[1]
                    
                    while (abs(current_lat - home_lat) > 0.00001 or 
                           abs(current_lon - home_lon) > 0.00001):
                        time.sleep(0.1)
                        
                        # Calculate direction vector
                        dlat = home_lat - current_lat
                        dlon = home_lon - current_lon
                        
                        # Normalize and scale for speed
                        dist = math.sqrt(dlat**2 + dlon**2)
                        if dist > 0:
                            speed = 0.00001  # Speed in degrees/iteration
                            dlat = dlat / dist * min(speed, dist)
                            dlon = dlon / dist * min(speed, dist)
                        
                        # Update position
                        current_lat += dlat
                        current_lon += dlon
                        
                        self._sim_current_position = (
                            current_lat,
                            current_lon,
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': current_lat,
                            'lon': current_lon,
                            'alt': current_alt
                        })
                    
                    # Descend
                    while current_alt > 0:
                        time.sleep(0.1)
                        current_alt = max(0, current_alt - 0.3)
                        self._sim_current_position = (
                            self._sim_current_position[0],
                            self._sim_current_position[1],
                            current_alt
                        )
                        self._trigger_event('position_changed', {
                            'lat': self._sim_current_position[0],
                            'lon': self._sim_current_position[1],
                            'alt': self._sim_current_position[2]
                        })
                    
                    # Disarm after landing
                    self._sim_armed = False
                    self._trigger_event('armed_changed', {'armed': False})
                
                # Start RTL simulation in a thread
                import threading
                thread = threading.Thread(target=sim_rtl)
                thread.daemon = True
                thread.start()
                
                return True
        
        except Exception as e:
            logger.error(f"Error during RTL: {str(e)}")
            return False
    
    def set_mode(self, mode: str) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode: Flight mode (e.g., 'GUIDED', 'LOITER', 'STABILIZE')
            
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        # Validate mode
        valid_modes = [
            'STABILIZE', 'ACRO', 'ALT_HOLD', 'AUTO', 'GUIDED', 'LOITER',
            'RTL', 'CIRCLE', 'LAND', 'DRIFT', 'SPORT', 'FLIP', 'AUTOTUNE',
            'POSHOLD', 'BRAKE', 'THROW', 'AVOID_ADSB', 'GUIDED_NOGPS'
        ]
        
        if mode not in valid_modes:
            logger.warning(f"Invalid mode: {mode}")
            return False
        
        try:
            if DRONEKIT_AVAILABLE and self.vehicle:
                self.vehicle.mode = VehicleMode(mode)
                logger.info(f"Set mode to {mode}")
                return True
            else:
                # Simulation mode
                logger.info(f"Set mode to {mode} (simulated)")
                self._sim_mode = mode
                self._trigger_event('mode_changed', {'mode': mode})
                return True
        
        except Exception as e:
            logger.error(f"Error setting mode: {str(e)}")
            return False
    
    def load_mission(self, mission: Mission) -> bool:
        """
        Load a mission for execution.
        
        Args:
            mission: Mission to load
            
        Returns:
            True if successful, False otherwise
        """
        if not self.is_connected:
            return False
        
        try:
            # Store mission locally
            self.current_mission = mission
            self.mission_running = False
            
            if DRONEKIT_AVAILABLE and self.vehicle:
                # Clear current mission
                cmds = self.vehicle.commands
                cmds.clear()
                cmds.upload()
                
                # Wait