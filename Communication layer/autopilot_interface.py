#!/usr/bin/env python3
"""
Autopilot Interface Module for Autonomous Drone Operations

This module provides a higher-level interface for autonomous drone operations,
using the DroneComm communication layer to interact with the Pixhawk.
"""

import time
import logging
import threading
import math
from typing import Dict, List, Tuple, Optional, Callable, Any
import json
import os

# Import the communication layer
from drone_comm_layer import DroneComm, ConnectionLostError, CommandTimeoutError

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('autopilot')


class MissionWaypoint:
    """Class representing a mission waypoint."""
    
    def __init__(self, lat: float, lon: float, alt: float, 
                 action: Optional[str] = None, 
                 hover_time: float = 0.0,
                 heading: Optional[float] = None):
        """
        Initialize a mission waypoint.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (relative to home position)
            action: Optional action to perform at waypoint (e.g., 'take_photo')
            hover_time: Time to hover at waypoint in seconds
            heading: Optional heading in degrees (0-360)
        """
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.action = action
        self.hover_time = hover_time
        self.heading = heading
    
    def to_dict(self) -> Dict:
        """Convert waypoint to dictionary for serialization."""
        return {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'action': self.action,
            'hover_time': self.hover_time,
            'heading': self.heading
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'MissionWaypoint':
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
    """Class representing a drone mission."""
    
    def __init__(self, name: str, waypoints: Optional[List[MissionWaypoint]] = None):
        """
        Initialize a mission.
        
        Args:
            name: Mission name
            waypoints: List of waypoints
        """
        self.name = name
        self.waypoints = waypoints or []
        self.created_at = time.time()
        self.modified_at = time.time()
    
    def add_waypoint(self, waypoint: MissionWaypoint) -> None:
        """Add a waypoint to the mission."""
        self.waypoints.append(waypoint)
        self.modified_at = time.time()
    
    def remove_waypoint(self, index: int) -> Optional[MissionWaypoint]:
        """Remove a waypoint from the mission."""
        if 0 <= index < len(self.waypoints):
            waypoint = self.waypoints.pop(index)
            self.modified_at = time.time()
            return waypoint
        return None
    
    def to_dict(self) -> Dict:
        """Convert mission to dictionary for serialization."""
        return {
            'name': self.name,
            'waypoints': [wp.to_dict() for wp in self.waypoints],
            'created_at': self.created_at,
            'modified_at': self.modified_at
        }
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Mission':
        """Create mission from dictionary."""
        mission = cls(name=data['name'])
        mission.waypoints = [MissionWaypoint.from_dict(wp) for wp in data['waypoints']]
        mission.created_at = data.get('created_at', time.time())
        mission.modified_at = data.get('modified_at', time.time())
        return mission
    
    def save(self, directory: str = 'missions') -> bool:
        """
        Save mission to file.
        
        Args:
            directory: Directory to save mission file
        
        Returns:
            bool: Success status
        """
        try:
            # Create directory if it doesn't exist
            if not os.path.exists(directory):
                os.makedirs(directory)
            
            # Save mission to JSON file
            filename = os.path.join(directory, f"{self.name}.json")
            with open(filename, 'w') as f:
                json.dump(self.to_dict(), f, indent=2)
            
            logger.info(f"Mission saved to {filename}")
            return True
        
        except Exception as e:
            logger.error(f"Failed to save mission: {str(e)}")
            return False
    
    @classmethod
    def load(cls, filename: str) -> Optional['Mission']:
        """
        Load mission from file.
        
        Args:
            filename: Mission file path
        
        Returns:
            Mission object or None if loading failed
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            mission = cls.from_dict(data)
            logger.info(f"Mission loaded from {filename}")
            return mission
        
        except Exception as e:
            logger.error(f"Failed to load mission: {str(e)}")
            return None


class AutopilotSystem:
    """Main autopilot system for autonomous drone operations."""
    
    def __init__(self, connection_string: str, baudrate: int = 57600):
        """
        Initialize the autopilot system.
        
        Args:
            connection_string: Connection string for the Pixhawk
            baudrate: Baud rate for serial communication
        """
        # Initialize communication layer
        self.comm = DroneComm(connection_string, baudrate)
        
        # Mission related variables
        self.current_mission = None
        self.current_waypoint_index = -1
        self.mission_in_progress = False
        
        # State variables
        self.home_position = None
        self.current_position = None
        self.current_altitude = None
        self.current_attitude = None
        self.is_armed = False
        self.flight_mode = None
        self.battery_level = None
        
        # Event callbacks
        self.event_callbacks = {}
        
        # Control flag
        self.should_stop_mission = False
        
        # Mission thread
        self.mission_thread = None
    
    def start(self) -> bool:
        """
        Start the autopilot system.
        
        Returns:
            bool: Success status
        """
        # Connect to the drone
        if not self.comm.connect():
            logger.error("Failed to connect to drone")
            return False
        
        # Register callbacks for telemetry
        self.comm.register_callback('position', self._handle_position)
        self.comm.register_callback('attitude', self._handle_attitude)
        
        logger.info("Autopilot system started")
        return True
    
    def stop(self) -> None:
        """Stop the autopilot system."""
        # Stop any running mission
        self.stop_mission()
        
        # Disconnect from the drone
        self.comm.disconnect()
        
        logger.info("Autopilot system stopped")
    
    def _handle_position(self, data: Dict) -> None:
        """
        Handle position updates from the drone.
        
        Args:
            data: Position data from the drone
        """
        self.current_position = (data['lat'], data['lon'])
        self.current_altitude = data['relative_alt']
        
        # Store home position if not set
        if self.home_position is None:
            self.home_position = self.current_position
            logger.info(f"Home position set: {self.home_position}")
        
        # Notify position change
        self._notify_event('position_changed', {
            'lat': data['lat'],
            'lon': data['lon'],
            'alt': data['relative_alt']
        })
    
    def _handle_attitude(self, data: Dict) -> None:
        """
        Handle attitude updates from the drone.
        
        Args:
            data: Attitude data from the drone
        """
        self.current_attitude = (
            math.degrees(data['roll']),
            math.degrees(data['pitch']),
            math.degrees(data['yaw'])
        )
        
        # Notify attitude change
        self._notify_event('attitude_changed', {
            'roll': math.degrees(data['roll']),
            'pitch': math.degrees(data['pitch']),
            'yaw': math.degrees(data['yaw'])
        })
    
    def register_event_callback(self, event_type: str, callback: Callable) -> None:
        """
        Register a callback for system events.
        
        Args:
            event_type: Type of event (e.g., 'mission_started', 'waypoint_reached')
            callback: Function to call when event occurs
        """
        if event_type not in self.event_callbacks:
            self.event_callbacks[event_type] = []
        
        self.event_callbacks[event_type].append(callback)
        logger.info(f"Registered callback for event: {event_type}")
    
    def unregister_event_callback(self, event_type: str, callback: Callable) -> None:
        """
        Unregister an event callback.
        
        Args:
            event_type: Type of event
            callback: Function to unregister
        """
        if event_type in self.event_callbacks:
            if callback in self.event_callbacks[event_type]:
                self.event_callbacks[event_type].remove(callback)
                logger.info(f"Unregistered callback for event: {event_type}")
    
    def _notify_event(self, event_type: str, data: Any = None) -> None:
        """
        Notify registered callbacks of an event.
        
        Args:
            event_type: Type of event
            data: Event data
        """
        if event_type in self.event_callbacks:
            for callback in self.event_callbacks[event_type]:
                try:
                    callback(data)
                except Exception as e:
                    logger.error(f"Event callback error: {str(e)}")
    
    def load_mission(self, mission: Mission) -> None:
        """
        Load a mission for execution.
        
        Args:
            mission: Mission to load
        """
        self.current_mission = mission
        self.current_waypoint_index = -1
        logger.info(f"Mission '{mission.name}' loaded with {len(mission.waypoints)} waypoints")
        self._notify_event('mission_loaded', {'name': mission.name})
    
    def start_mission(self) -> bool:
        """
        Start executing the loaded mission.
        
        Returns:
            bool: Success status
        """
        if not self.current_mission or not self.current_mission.waypoints:
            logger.error("No mission loaded or mission has no waypoints")
            return False
        
        if self.mission_in_progress:
            logger.warning("Mission already in progress")
            return False
        
        # Reset control flag
        self.should_stop_mission = False
        
        # Start mission execution in a separate thread
        self.mission_thread = threading.Thread(target=self._execute_mission)
        self.mission_thread.daemon = True
        self.mission_thread.start()
        
        self.mission_in_progress = True
        logger.info(f"Started mission execution: {self.current_mission.name}")
        self._notify_event('mission_started', {'name': self.current_mission.name})
        
        return True
    
    def stop_mission(self) -> None:
        """Stop the current mission execution."""
        if not self.mission_in_progress:
            logger.warning("No mission in progress")
            return
        
        self.should_stop_mission = True
        
        if self.mission_thread:
            self.mission_thread.join(timeout=5.0)
        
        self.mission_in_progress = False
        logger.info("Mission execution stopped")
        self._notify_event('mission_stopped', {'name': self.current_mission.name})
    
    def _execute_mission(self) -> None:
        """Execute the current mission (run in a separate thread)."""
        try:
            # Make sure we're in GUIDED mode
            self.comm.set_mode('GUIDED')
            time.sleep(1)
            
            # Arm the drone if not armed
            if not self.is_armed:
                logger.info("Arming drone...")
                self.comm.arm(True)
                time.sleep(3)  # Wait for arming
            
            # Start with first waypoint
            self.current_waypoint_index = 0
            
            while (self.current_waypoint_index < len(self.current_mission.waypoints) 
                   and not self.should_stop_mission):
                
                waypoint = self.current_mission.waypoints[self.current_waypoint_index]
                
                # Navigate to waypoint
                logger.info(f"Navigating to waypoint {self.current_waypoint_index+1}: "
                           f"({waypoint.lat}, {waypoint.lon}, {waypoint.alt})")
                
                self._notify_event('waypoint_navigation', {
                    'index': self.current_waypoint_index,
                    'lat': waypoint.lat,
                    'lon': waypoint.lon,
                    'alt': waypoint.alt
                })
                
                # Take off if this is the first waypoint and we're on the ground
                if self.current_waypoint_index == 0 and self.current_altitude is not None and self.current_altitude < 1.0:
                    logger.info(f"Taking off to {waypoint.alt} meters")
                    self.comm.takeoff(waypoint.alt)
                    time.sleep(10)  # Allow time for takeoff
                else:
                    # Navigate to the waypoint
                    self.comm.goto(waypoint.lat, waypoint.lon, waypoint.alt)
                    
                    # Wait until we reach the waypoint
                    self._wait_for_waypoint(waypoint)
                
                if self.should_stop_mission:
                    break
                
                # Execute action if specified
                if waypoint.action:
                    self._execute_waypoint_action(waypoint.action)
                
                # Hover for specified time
                if waypoint.hover_time > 0:
                    logger.info(f"Hovering for {waypoint.hover_time} seconds")
                    time.sleep(waypoint.hover_time)
                
                # Mark waypoint as reached
                self._notify_event('waypoint_reached', {
                    'index': self.current_waypoint_index,
                    'lat': waypoint.lat,
                    'lon': waypoint.lon,
                    'alt': waypoint.alt
                })
                
                # Move to next waypoint
                self.current_waypoint_index += 1
            
            # Mission complete or stopped
            if not self.should_stop_mission and self.current_waypoint_index >= len(self.current_mission.waypoints):
                logger.info("Mission completed successfully")
                self._notify_event('mission_completed', {'name': self.current_mission.name})
                
                # Return to launch
                logger.info("Returning to launch")
                self.comm.set_mode('RTL')
            
        except Exception as e:
            logger.error(f"Mission execution error: {str(e)}")
            self._notify_event('mission_error', {'error': str(e)})
        
        finally:
            self.mission_in_progress = False
    
    def _wait_for_waypoint(self, waypoint: MissionWaypoint, timeout: float = 60.0, accuracy: float = 2.0) -> bool:
        """
        Wait until the drone reaches a waypoint.
        
        Args:
            waypoint: Target waypoint
            timeout: Maximum wait time in seconds
            accuracy: Position accuracy in meters
        
        Returns:
            bool: True if waypoint reached, False if timeout
        """
        start_time = time.time()
        check_interval = 0.5  # seconds
        
        while time.time() - start_time < timeout:
            if self.should_stop_mission:
                return False
            
            if self.current_position is None or self.current_altitude is None:
                time.sleep(check_interval)
                continue
            
            # Calculate distance to waypoint
            distance = self._calculate_distance(
                self.current_position[0], self.current_position[1],
                waypoint.lat, waypoint.lon
            )
            
            # Check altitude difference
            alt_diff = abs(self.current_altitude - waypoint.alt)
            
            # Check if we've reached the waypoint
            if distance < accuracy and alt_diff < 1.0:
                logger.info(f"Reached waypoint ({distance:.2f}m, alt diff: {alt_diff:.2f}m)")
                return True
            
            # Log progress
            if (int(time.time()) % 5) == 0:  # Log every 5 seconds
                logger.info(f"Distance to waypoint: {distance:.2f}m, alt diff: {alt_diff:.2f}m")
            
            time.sleep(check_interval)
        
        logger.warning(f"Waypoint timeout after {timeout} seconds")
        return False
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate distance between two coordinates using the Haversine formula.
        
        Args:
            lat1, lon1: First coordinate
            lat2, lon2: Second coordinate
        
        Returns:
            Distance in meters
        """
        # Earth radius in meters
        R = 6371000
        
        # Convert degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Differences
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
    
    def _execute_waypoint_action(self, action: str) -> None:
        """
        Execute an action at a waypoint.
        
        Args:
            action: Action name
        """
        logger.info(f"Executing action: {action}")
        
        # Example actions
        if action == 'take_photo':
            self._take_photo()
        elif action == 'record_video':
            self._record_video()
        elif action == 'scan_area':
            self._scan_area()
        else:
            logger.warning(f"Unknown action: {action}")
    
    def _take_photo(self) -> None:
        """Take a photo (example implementation)."""
        logger.info("Taking photo")
        # Implement camera control here
        self._notify_event('photo_taken', {
            'position': self.current_position,
            'altitude': self.current_altitude,
            'timestamp': time.time()
        })
    
    def _record_video(self, duration: float = 5.0) -> None:
        """
        Record a video (example implementation).
        
        Args:
            duration: Recording duration in seconds
        """
        logger.info(f"Recording video for {duration} seconds")
        # Implement camera control here
        self._notify_event('video_started', {
            'position': self.current_position,
            'altitude': self.current_altitude,
            'timestamp': time.time()
        })
        
        time.sleep(duration)
        
        self._notify_event('video_stopped', {
            'position': self.current_position,
            'altitude': self.current_altitude,
            'timestamp': time.time()
        })
    
    def _scan_area(self, radius: float = 5.0) -> None:
        """
        Scan area around current position (example implementation).
        
        Args:
            radius: Scan radius in meters
        """
        logger.info(f"Scanning area with radius {radius} meters")
        # Implement area scanning logic here
        self._notify_event('area_scanned', {
            'position': self.current_position,
            'altitude': self.current_altitude,
            'radius': radius,
            'timestamp': time.time()
        })
    
    # Convenience methods for direct control
    
    def arm(self) -> bool:
        """
        Arm the drone.
        
        Returns:
            bool: Success status
        """
        result = self.comm.arm(True)
        if result:
            self.is_armed = True
            logger.info("Drone armed")
        return result
    
    def disarm(self) -> bool:
        """
        Disarm the drone.
        
        Returns:
            bool: Success status
        """
        result = self.comm.arm(False)
        if result:
            self.is_armed = False
            logger.info("Drone disarmed")
        return result
    
    def takeoff(self, altitude: float) -> bool:
        """
        Command the drone to take off.
        
        Args:
            altitude: Target altitude in meters
        
        Returns:
            bool: Success status
        """
        return self.comm.takeoff(altitude)
    
    def land(self) -> bool:
        """
        Command the drone to land.
        
        Returns:
            bool: Success status
        """
        return self.comm.land()
    
    def return_to_launch(self) -> bool:
        """
        Command the drone to return to launch position.
        
        Returns:
            bool: Success status
        """
        return self.comm.set_mode('RTL')
    
    def set_mode(self, mode: str) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode: Flight mode
        
        Returns:
            bool: Success status
        """
        return self.comm.set_mode(mode)
    
    def get_status(self) -> Dict:
        """
        Get the current drone status.
        
        Returns:
            Dict with drone status information
        """
        return {
            'position': self.current_position,
            'altitude': self.current_altitude,
            'attitude': self.current_attitude,
            'armed': self.is_armed,
            'mode': self.flight_mode,
            'battery': self.battery_level,
            'mission_in_progress': self.mission_in_progress,
            'current_waypoint': self.current_waypoint_index if self.mission_in_progress else -1
        }


# Example usage of the AutopilotSystem class
if __name__ == "__main__":
    # Connection string for Pixhawk over serial
    connection_string = '/dev/ttyAMA0'  # Change this to match your setup
    
    # Create autopilot system
    autopilot = AutopilotSystem(connection_string, baudrate=57600)
    
    # Example status callback
    def on_position_changed(data):
        print(f"Position: lat={data['lat']:.7f}, lon={data['lon']:.7f}, alt={data['alt']:.2f}m")
    
    def on_mission_event(data):
        print(f"Mission event: {data}")
    
    try:
        # Start the autopilot system
        if autopilot.start():
            print("Autopilot system started!")
            
            # Register callbacks
            autopilot.register_event_callback('position_changed', on_position_changed)
            autopilot.register_event_callback('mission_started', on_mission_event)
            autopilot.register_event_callback('mission_completed', on_mission_event)
            autopilot.register_event_callback('waypoint_reached', on_mission_event)
            
            # Create a simple mission
            mission = Mission("Simple Survey")
            
            # Add waypoints (using example coordinates - replace with real ones)
            mission.add_waypoint(MissionWaypoint(47.3977419, 8.5455938, 5.0))
            mission.add_waypoint(MissionWaypoint(47.3977419, 8.5457938, 10.0, action='take_photo'))
            mission.add_waypoint(MissionWaypoint(47.3979419, 8.5457938, 10.0, hover_time=5.0))
            mission.add_waypoint(MissionWaypoint(47.3979419, 8.5455938, 5.0, action='take_photo'))
            
            # Save the mission
            mission.save()
            
            # Load and execute the mission
            autopilot.load_mission(mission)
            
            # Wait for a while to receive data before starting mission
            print("Waiting for telemetry data...")
            time.sleep(10)
            
            # Start the mission
            autopilot.start_mission()
            
            # Wait for mission completion
            while autopilot.mission_in_progress:
                time.sleep(1)
            
            print("Mission complete or stopped")
            
        else:
            print("Failed to start autopilot system!")
    
    except KeyboardInterrupt:
        print("Program interrupted by user")
    
    finally:
        # Clean up
        autopilot.stop()
        print("Autopilot system stopped")