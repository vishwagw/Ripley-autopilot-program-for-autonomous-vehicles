#!/usr/bin/env python3
"""
Drone Communication Layer for Pixhawk-Raspberry Pi Integration

This module provides a communication interface between a Raspberry Pi 
and Pixhawk flight controller using MAVLink protocol for autonomous drone operations.
"""

import time
import logging
import threading
from typing import Callable, Dict, List, Optional, Union, Any
import queue

# Import MAVLink library for Pixhawk communication
from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('drone_comm')

class CommunicationError(Exception):
    """Base exception for communication errors."""
    pass

class ConnectionLostError(CommunicationError):
    """Raised when connection with the flight controller is lost."""
    pass

class CommandTimeoutError(CommunicationError):
    """Raised when a command times out."""
    pass


class DroneComm:
    """Main communication interface for autonomous drone operations."""

    def __init__(self, connection_string: str, baudrate: int = 57600):
        """
        Initialize the communication interface.
        
        Args:
            connection_string: Connection string for the Pixhawk (e.g., '/dev/ttyAMA0')
            baudrate: Baud rate for serial communication
        """
        self.connection_string = connection_string
        self.baudrate = baudrate
        self.vehicle = None
        self.is_connected = False
        self._heartbeat_timeout = 3  # seconds
        self._last_heartbeat = 0
        self._command_queue = queue.Queue()
        self._telemetry_callbacks = {}
        
        # Control flags
        self.should_exit = False
        
        # Initialize threads
        self._heartbeat_thread = None
        self._receive_thread = None
        self._command_thread = None
        
    def connect(self) -> bool:
        """
        Establish connection with the Pixhawk flight controller.
        
        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            logger.info(f"Connecting to Pixhawk on {self.connection_string}")
            # Connect to the Vehicle
            self.vehicle = mavutil.mavlink_connection(
                self.connection_string,
                baud=self.baudrate,
            )
            
            # Wait for the first heartbeat
            logger.info("Waiting for first heartbeat...")
            self.vehicle.wait_heartbeat()
            logger.info("Heartbeat received!")
            
            self.is_connected = True
            self._last_heartbeat = time.time()
            
            # Start threads
            self._start_threads()
            
            logger.info("Connected to Pixhawk!")
            return True
        
        except Exception as e:
            logger.error(f"Connection failed: {str(e)}")
            return False
            
    def disconnect(self) -> None:
        """Close the connection with the flight controller."""
        self.should_exit = True
        
        if self._heartbeat_thread:
            self._heartbeat_thread.join(timeout=2.0)
        if self._receive_thread:
            self._receive_thread.join(timeout=2.0)
        if self._command_thread:
            self._command_thread.join(timeout=2.0)
        
        if self.vehicle:
            self.vehicle.close()
            
        self.is_connected = False
        logger.info("Disconnected from Pixhawk")
    
    def _start_threads(self) -> None:
        """Start the communication threads."""
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop)
        self._heartbeat_thread.daemon = True
        self._heartbeat_thread.start()
        
        self._receive_thread = threading.Thread(target=self._receive_loop)
        self._receive_thread.daemon = True
        self._receive_thread.start()
        
        self._command_thread = threading.Thread(target=self._command_loop)
        self._command_thread.daemon = True
        self._command_thread.start()
    
    def _heartbeat_loop(self) -> None:
        """Send heartbeat messages to maintain connection."""
        while not self.should_exit:
            try:
                # Send heartbeat message
                if self.is_connected:
                    self.vehicle.mav.heartbeat_send(
                        mavlink.MAV_TYPE_GCS,  # GCS type
                        mavlink.MAV_AUTOPILOT_INVALID,  # Autopilot type
                        0,  # Base mode
                        0,  # Custom mode
                        mavlink.MAV_STATE_ACTIVE  # System status
                    )
                time.sleep(1)  # Send heartbeat every second
            except Exception as e:
                logger.error(f"Heartbeat error: {str(e)}")
                time.sleep(1)
    
    def _receive_loop(self) -> None:
        """Receive and process incoming messages."""
        while not self.should_exit:
            try:
                if not self.is_connected:
                    time.sleep(0.1)
                    continue
                
                # Check for timeout
                if time.time() - self._last_heartbeat > self._heartbeat_timeout:
                    logger.warning("Heartbeat timeout!")
                    self.is_connected = False
                    continue
                
                # Process incoming messages
                msg = self.vehicle.recv_match(blocking=True, timeout=0.5)
                if msg:
                    self._process_message(msg)
            
            except Exception as e:
                logger.error(f"Receive error: {str(e)}")
                time.sleep(0.1)
    
    def _command_loop(self) -> None:
        """Process and send commands from the queue."""
        while not self.should_exit:
            try:
                if not self.is_connected:
                    time.sleep(0.1)
                    continue
                
                # Get command from queue
                try:
                    cmd, args, kwargs = self._command_queue.get(timeout=0.5)
                    cmd(*args, **kwargs)
                    self._command_queue.task_done()
                except queue.Empty:
                    continue
                
            except Exception as e:
                logger.error(f"Command error: {str(e)}")
                time.sleep(0.1)
    
    def _process_message(self, msg) -> None:
        """
        Process incoming MAVLink messages.
        
        Args:
            msg: MAVLink message
        """
        msg_type = msg.get_type()
        
        # Update heartbeat time
        if msg_type == 'HEARTBEAT':
            self._last_heartbeat = time.time()
            # Extract flight mode and status
            if hasattr(msg, 'custom_mode'):
                # Process flight mode info
                pass
        
        # Process other message types
        elif msg_type == 'GLOBAL_POSITION_INT':
            # Process position data
            lat = msg.lat / 1e7  # Convert to degrees
            lon = msg.lon / 1e7
            alt = msg.alt / 1000  # Convert to meters
            relative_alt = msg.relative_alt / 1000
            
            # Call registered callbacks
            self._call_callbacks('position', {
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'relative_alt': relative_alt
            })
            
        elif msg_type == 'ATTITUDE':
            # Process attitude data
            self._call_callbacks('attitude', {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,
                'yawspeed': msg.yawspeed
            })
        
        # Add more message handlers as needed
    
    def _call_callbacks(self, event_type: str, data: Dict[str, Any]) -> None:
        """
        Call registered callbacks for a specific event type.
        
        Args:
            event_type: Type of event
            data: Event data
        """
        if event_type in self._telemetry_callbacks:
            for callback in self._telemetry_callbacks[event_type]:
                try:
                    callback(data)
                except Exception as e:
                    logger.error(f"Callback error: {str(e)}")
    
    def register_callback(self, event_type: str, callback: Callable) -> None:
        """
        Register a callback for a specific event type.
        
        Args:
            event_type: Type of event to listen for (e.g., 'position', 'attitude')
            callback: Function to call when event occurs
        """
        if event_type not in self._telemetry_callbacks:
            self._telemetry_callbacks[event_type] = []
        
        self._telemetry_callbacks[event_type].append(callback)
        logger.info(f"Registered callback for event type: {event_type}")
    
    def unregister_callback(self, event_type: str, callback: Callable) -> None:
        """
        Unregister a callback.
        
        Args:
            event_type: Type of event
            callback: Function to unregister
        """
        if event_type in self._telemetry_callbacks:
            if callback in self._telemetry_callbacks[event_type]:
                self._telemetry_callbacks[event_type].remove(callback)
                logger.info(f"Unregistered callback for event type: {event_type}")
    
    def arm(self, value: bool = True) -> bool:
        """
        Arm or disarm the drone.
        
        Args:
            value: True to arm, False to disarm
        
        Returns:
            bool: Success status
        """
        if not self.is_connected:
            logger.error("Cannot arm: Not connected")
            return False
        
        arm_val = 1 if value else 0
        
        # Send arm command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            arm_val,  # Param1: 1 to arm, 0 to disarm
            0, 0, 0, 0, 0, 0  # Other params (not used)
        )
        
        logger.info(f"{'Arming' if value else 'Disarming'} command sent")
        return True
    
    def takeoff(self, altitude: float) -> bool:
        """
        Command the drone to take off to a specified altitude.
        
        Args:
            altitude: Target altitude in meters
        
        Returns:
            bool: Success status
        """
        if not self.is_connected:
            logger.error("Cannot takeoff: Not connected")
            return False
        
        # Send takeoff command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0,  # Params 1-6 (not used)
            altitude  # Param7: Altitude
        )
        
        logger.info(f"Takeoff command sent: {altitude}m")
        return True
    
    def land(self) -> bool:
        """
        Command the drone to land.
        
        Returns:
            bool: Success status
        """
        if not self.is_connected:
            logger.error("Cannot land: Not connected")
            return False
        
        # Send land command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmation
            0, 0, 0, 0, 0, 0, 0  # Params (not used)
        )
        
        logger.info("Land command sent")
        return True
    
    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Command the drone to go to a specified position.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters (absolute)
        
        Returns:
            bool: Success status
        """
        if not self.is_connected:
            logger.error("Cannot goto: Not connected")
            return False
        
        # Send goto command
        self.vehicle.mav.mission_item_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            0,  # Sequence
            mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
            mavlink.MAV_CMD_NAV_WAYPOINT,  # Command
            2,  # Current (2 means guided mode)
            1,  # Autocontinue
            0, 0, 0, 0,  # Params 1-4
            lat, lon, alt  # Position
        )
        
        logger.info(f"Goto command sent: lat={lat}, lon={lon}, alt={alt}")
        return True
    
    def set_velocity(self, vx: float, vy: float, vz: float) -> bool:
        """
        Set the drone's velocity in the NED frame.
        
        Args:
            vx: Velocity in North direction (m/s)
            vy: Velocity in East direction (m/s)
            vz: Velocity in Down direction (m/s)
        
        Returns:
            bool: Success status
        """
        if not self.is_connected:
            logger.error("Cannot set velocity: Not connected")
            return False
        
        # Set velocity
        self.vehicle.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_FRAME_LOCAL_NED,  # Frame
            0b0000111111000111,  # Type mask (only speeds enabled)
            0, 0, 0,  # Position
            vx, vy, vz,  # Velocity
            0, 0, 0,  # Acceleration
            0, 0  # Yaw, yaw rate
        )
        
        logger.info(f"Velocity command sent: vx={vx}, vy={vy}, vz={vz}")
        return True
    
    def set_mode(self, mode: str) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode: Flight mode (e.g., 'GUIDED', 'RTL', 'LOITER')
        
        Returns:
            bool: Success status
        """
        # Mode mapping
        mode_map = {
            'STABILIZE': 0,
            'GUIDED': 4,
            'LOITER': 5,
            'RTL': 6,
            'AUTO': 3,
            'LAND': 9,
        }
        
        if mode not in mode_map:
            logger.error(f"Unknown mode: {mode}")
            return False
        
        if not self.is_connected:
            logger.error("Cannot set mode: Not connected")
            return False
        
        # Send mode change command
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_CMD_DO_SET_MODE,
            0,  # Confirmation
            mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_map[mode],
            0, 0, 0, 0, 0  # Params (not used)
        )
        
        logger.info(f"Mode change command sent: {mode}")
        return True
        
    def get_battery_status(self) -> Optional[Dict[str, float]]:
        """
        Request battery status.
        
        Returns:
            Dict with battery info or None if not available
        """
        if not self.is_connected:
            logger.error("Cannot get battery status: Not connected")
            return None
        
        # Request battery info
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,  # Confirmation
            mavlink.MAVLINK_MSG_ID_BATTERY_STATUS,
            0, 0, 0, 0, 0, 0  # Params (not used)
        )
        
        # Battery info will be received asynchronously
        # and processed in the _process_message method
        logger.info("Battery status requested")
        return None


# Example usage of the DroneComm class
if __name__ == "__main__":
    # Connection string for Pixhawk over serial
    connection_string = '/dev/ttyAMA0'  # Change this to match your setup
    
    # Create communication interface
    drone = DroneComm(connection_string, baudrate=57600)
    
    # Example position callback
    def position_callback(data):
        print(f"Position: lat={data['lat']:.7f}, lon={data['lon']:.7f}, alt={data['alt']:.2f}m")
    
    # Example attitude callback
    def attitude_callback(data):
        print(f"Attitude: roll={data['roll']:.2f}, pitch={data['pitch']:.2f}, yaw={data['yaw']:.2f}")
    
    try:
        # Connect to the drone
        if drone.connect():
            print("Connected to drone!")
            
            # Register callbacks
            drone.register_callback('position', position_callback)
            drone.register_callback('attitude', attitude_callback)
            
            # Wait for a while to receive data
            time.sleep(10)
            
            # Example command sequence
            drone.set_mode('GUIDED')
            time.sleep(1)
            
            drone.arm(True)
            time.sleep(2)
            
            drone.takeoff(5.0)  # Take off to 5 meters
            time.sleep(20)  # Wait for takeoff
            
            # Fly to a position
            drone.goto(47.3977419, 8.5455938, 10)
            time.sleep(30)
            
            # Return to launch
            drone.set_mode('RTL')
            time.sleep(30)
            
            # Disarm
            drone.arm(False)
            
        else:
            print("Failed to connect!")
    
    except KeyboardInterrupt:
        print("Program interrupted by user")
    
    finally:
        # Clean up
        drone.disconnect()
        print("Drone communication terminated")
