#!/usr/bin/env python3
"""
Drone Autopilot Software
A comprehensive flight control system for autonomous drone operations
"""

import time
import math
import threading
import json
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from enum import Enum
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class FlightMode(Enum):
    MANUAL = "manual"
    STABILIZE = "stabilize"
    ALTITUDE_HOLD = "altitude_hold"
    POSITION_HOLD = "position_hold"
    AUTO = "auto"
    RTL = "return_to_launch"
    LAND = "land"
    EMERGENCY = "emergency"

@dataclass
class Position:
    """3D position with latitude, longitude, and altitude"""
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0

@dataclass
class Attitude:
    """Drone attitude (orientation)"""
    roll: float = 0.0   # degrees
    pitch: float = 0.0  # degrees
    yaw: float = 0.0    # degrees

@dataclass
class Velocity:
    """3D velocity vector"""
    vx: float = 0.0  # m/s
    vy: float = 0.0  # m/s
    vz: float = 0.0  # m/s

@dataclass
class SensorData:
    """Consolidated sensor readings"""
    position: Position = field(default_factory=Position)
    attitude: Attitude = field(default_factory=Attitude)
    velocity: Velocity = field(default_factory=Velocity)
    battery_voltage: float = 12.0
    gps_fix: bool = False
    timestamp: float = field(default_factory=time.time)

@dataclass
class ControlInputs:
    """Motor control outputs"""
    motor1: float = 0.0  # 0-100%
    motor2: float = 0.0
    motor3: float = 0.0
    motor4: float = 0.0

class PIDController:
    """PID controller for flight stabilization"""
    
    def __init__(self, kp: float, ki: float, kd: float, output_limits: Tuple[float, float] = (-100, 100)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
    
    def update(self, setpoint: float, measurement: float) -> float:
        """Calculate PID output"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
        
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        # Update for next iteration
        self.prev_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

class NavigationController:
    """Handles waypoint navigation and path planning"""
    
    def __init__(self):
        self.waypoints: List[Position] = []
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 2.0  # meters
    
    def add_waypoint(self, position: Position):
        """Add a waypoint to the mission"""
        self.waypoints.append(position)
    
    def get_current_target(self) -> Optional[Position]:
        """Get current target waypoint"""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def update_navigation(self, current_pos: Position) -> Optional[Position]:
        """Update navigation and return next target"""
        target = self.get_current_target()
        if not target:
            return None
        
        # Calculate distance to current waypoint
        distance = self.calculate_distance(current_pos, target)
        
        if distance < self.waypoint_tolerance:
            self.current_waypoint_index += 1
            logger.info(f"Waypoint {self.current_waypoint_index} reached")
            return self.get_current_target()
        
        return target
    
    def calculate_distance(self, pos1: Position, pos2: Position) -> float:
        """Calculate 3D distance between two positions"""
        # Simplified distance calculation (assumes local coordinates)
        dx = (pos2.lat - pos1.lat) * 111000  # rough conversion to meters
        dy = (pos2.lon - pos1.lon) * 111000 * math.cos(math.radians(pos1.lat))
        dz = pos2.alt - pos1.alt
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_bearing(self, from_pos: Position, to_pos: Position) -> float:
        """Calculate bearing from one position to another"""
        dx = (to_pos.lat - from_pos.lat) * 111000
        dy = (to_pos.lon - from_pos.lon) * 111000 * math.cos(math.radians(from_pos.lat))
        return math.degrees(math.atan2(dy, dx))

class SafetyMonitor:
    """Monitors flight safety and triggers emergency procedures"""
    
    def __init__(self):
        self.min_battery_voltage = 10.5
        self.max_altitude = 120.0  # meters
        self.max_distance_from_home = 500.0  # meters
        self.home_position: Optional[Position] = None
        self.emergency_triggered = False
    
    def set_home_position(self, position: Position):
        """Set home position for safety checks"""
        self.home_position = position
        logger.info(f"Home position set: {position.lat}, {position.lon}")
    
    def check_safety(self, sensor_data: SensorData) -> bool:
        """Check if flight conditions are safe"""
        if self.emergency_triggered:
            return False
        
        # Battery check
        if sensor_data.battery_voltage < self.min_battery_voltage:
            logger.warning("Low battery detected!")
            self.emergency_triggered = True
            return False
        
        # Altitude check
        if sensor_data.position.alt > self.max_altitude:
            logger.warning("Maximum altitude exceeded!")
            return False
        
        # Distance from home check
        if self.home_position:
            distance = self.calculate_distance_from_home(sensor_data.position)
            if distance > self.max_distance_from_home:
                logger.warning("Maximum distance from home exceeded!")
                return False
        
        return True
    
    def calculate_distance_from_home(self, current_pos: Position) -> float:
        """Calculate distance from home position"""
        if not self.home_position:
            return 0.0
        
        dx = (current_pos.lat - self.home_position.lat) * 111000
        dy = (current_pos.lon - self.home_position.lon) * 111000 * math.cos(math.radians(self.home_position.lat))
        return math.sqrt(dx*dx + dy*dy)

class DroneAutopilot:
    """Main autopilot system"""
    
    def __init__(self):
        self.flight_mode = FlightMode.MANUAL
        self.is_armed = False
        self.is_running = False
        
        # Initialize controllers
        self.roll_pid = PIDController(1.0, 0.1, 0.05, (-30, 30))
        self.pitch_pid = PIDController(1.0, 0.1, 0.05, (-30, 30))
        self.yaw_pid = PIDController(1.0, 0.05, 0.02, (-30, 30))
        self.altitude_pid = PIDController(2.0, 0.5, 0.1, (-50, 50))
        
        self.navigation = NavigationController()
        self.safety = SafetyMonitor()
        
        # Current state
        self.sensor_data = SensorData()
        self.target_position = Position()
        self.target_attitude = Attitude()
        
        # Control thread
        self.control_thread = None
        self.control_rate = 50  # Hz
    
    def arm(self) -> bool:
        """Arm the drone for flight"""
        if not self.safety.check_safety(self.sensor_data):
            logger.error("Cannot arm: Safety check failed")
            return False
        
        self.is_armed = True
        logger.info("Drone armed")
        return True
    
    def disarm(self):
        """Disarm the drone"""
        self.is_armed = False
        logger.info("Drone disarmed")
    
    def set_flight_mode(self, mode: FlightMode):
        """Change flight mode"""
        self.flight_mode = mode
        logger.info(f"Flight mode changed to: {mode.value}")
        
        # Reset PID controllers when changing modes
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()
        self.altitude_pid.reset()
    
    def update_sensors(self, sensor_data: SensorData):
        """Update sensor data"""
        self.sensor_data = sensor_data
    
    def calculate_control_outputs(self) -> ControlInputs:
        """Calculate motor control outputs based on current flight mode"""
        if not self.is_armed:
            return ControlInputs()
        
        controls = ControlInputs()
        
        if self.flight_mode == FlightMode.STABILIZE:
            controls = self.stabilize_mode()
        elif self.flight_mode == FlightMode.ALTITUDE_HOLD:
            controls = self.altitude_hold_mode()
        elif self.flight_mode == FlightMode.POSITION_HOLD:
            controls = self.position_hold_mode()
        elif self.flight_mode == FlightMode.AUTO:
            controls = self.auto_mode()
        elif self.flight_mode == FlightMode.RTL:
            controls = self.return_to_launch_mode()
        elif self.flight_mode == FlightMode.LAND:
            controls = self.land_mode()
        
        return controls
    
    def stabilize_mode(self) -> ControlInputs:
        """Stabilize mode - maintain level attitude"""
        roll_output = self.roll_pid.update(self.target_attitude.roll, self.sensor_data.attitude.roll)
        pitch_output = self.pitch_pid.update(self.target_attitude.pitch, self.sensor_data.attitude.pitch)
        yaw_output = self.yaw_pid.update(self.target_attitude.yaw, self.sensor_data.attitude.yaw)
        
        # Convert to motor outputs (simplified quadcopter mixing)
        base_throttle = 50.0  # Base hover throttle
        
        controls = ControlInputs()
        controls.motor1 = base_throttle + pitch_output + yaw_output
        controls.motor2 = base_throttle - roll_output - yaw_output
        controls.motor3 = base_throttle - pitch_output + yaw_output
        controls.motor4 = base_throttle + roll_output - yaw_output
        
        # Ensure outputs are within limits
        controls.motor1 = max(0, min(100, controls.motor1))
        controls.motor2 = max(0, min(100, controls.motor2))
        controls.motor3 = max(0, min(100, controls.motor3))
        controls.motor4 = max(0, min(100, controls.motor4))
        
        return controls
    
    def altitude_hold_mode(self) -> ControlInputs:
        """Altitude hold mode"""
        altitude_output = self.altitude_pid.update(self.target_position.alt, self.sensor_data.position.alt)
        
        # Get stabilize outputs and add altitude control
        controls = self.stabilize_mode()
        
        # Add altitude control to all motors
        controls.motor1 += altitude_output
        controls.motor2 += altitude_output
        controls.motor3 += altitude_output
        controls.motor4 += altitude_output
        
        # Ensure outputs are within limits
        controls.motor1 = max(0, min(100, controls.motor1))
        controls.motor2 = max(0, min(100, controls.motor2))
        controls.motor3 = max(0, min(100, controls.motor3))
        controls.motor4 = max(0, min(100, controls.motor4))
        
        return controls
    
    def position_hold_mode(self) -> ControlInputs:
        """Position hold mode"""
        # Calculate position errors and convert to attitude targets
        pos_error_x = self.target_position.lat - self.sensor_data.position.lat
        pos_error_y = self.target_position.lon - self.sensor_data.position.lon
        
        # Convert position errors to attitude targets (simplified)
        self.target_attitude.pitch = max(-15, min(15, pos_error_x * 1000))
        self.target_attitude.roll = max(-15, min(15, pos_error_y * 1000))
        
        return self.altitude_hold_mode()
    
    def auto_mode(self) -> ControlInputs:
        """Autonomous flight mode following waypoints"""
        target = self.navigation.update_navigation(self.sensor_data.position)
        if target:
            self.target_position = target
        
        return self.position_hold_mode()
    
    def return_to_launch_mode(self) -> ControlInputs:
        """Return to launch mode"""
        if self.safety.home_position:
            self.target_position = self.safety.home_position
        
        return self.position_hold_mode()
    
    def land_mode(self) -> ControlInputs:
        """Landing mode"""
        # Gradually reduce altitude
        self.target_position.alt = max(0, self.target_position.alt - 0.5)
        
        if self.sensor_data.position.alt < 0.5:
            # Close to ground, reduce throttle
            return ControlInputs()
        
        return self.altitude_hold_mode()
    
    def start_control_loop(self):
        """Start the main control loop"""
        if self.is_running:
            return
        
        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        logger.info("Control loop started")
    
    def stop_control_loop(self):
        """Stop the main control loop"""
        self.is_running = False
        if self.control_thread:
            self.control_thread.join()
        logger.info("Control loop stopped")
    
    def _control_loop(self):
        """Main control loop"""
        loop_time = 1.0 / self.control_rate
        
        while self.is_running:
            start_time = time.time()
            
            # Check safety
            if not self.safety.check_safety(self.sensor_data):
                if self.flight_mode != FlightMode.EMERGENCY:
                    logger.warning("Safety check failed, entering emergency mode")
                    self.set_flight_mode(FlightMode.LAND)
            
            # Calculate control outputs
            controls = self.calculate_control_outputs()
            
            # Send controls to motors (this would interface with hardware)
            self.send_motor_commands(controls)
            
            # Log telemetry
            self.log_telemetry()
            
            # Maintain loop rate
            elapsed = time.time() - start_time
            if elapsed < loop_time:
                time.sleep(loop_time - elapsed)
    
    def send_motor_commands(self, controls: ControlInputs):
        """Send motor commands to hardware (placeholder)"""
        # This would interface with actual motor controllers
        pass
    
    def log_telemetry(self):
        """Log telemetry data"""
        if time.time() % 5 < 0.02:  # Log every 5 seconds
            logger.info(f"Mode: {self.flight_mode.value}, "
                       f"Alt: {self.sensor_data.position.alt:.1f}m, "
                       f"Batt: {self.sensor_data.battery_voltage:.1f}V, "
                       f"Armed: {self.is_armed}")

# Example usage and testing
def main():
    """Example usage of the drone autopilot"""
    autopilot = DroneAutopilot()
    
    # Set home position
    home = Position(lat=40.7128, lon=-74.0060, alt=0)
    autopilot.safety.set_home_position(home)
    
    # Add some waypoints for auto mode
    autopilot.navigation.add_waypoint(Position(lat=40.7130, lon=-74.0058, alt=10))
    autopilot.navigation.add_waypoint(Position(lat=40.7132, lon=-74.0056, alt=15))
    autopilot.navigation.add_waypoint(Position(lat=40.7130, lon=-74.0060, alt=10))
    
    # Simulate sensor data
    sensor_data = SensorData()
    sensor_data.position = Position(lat=40.7128, lon=-74.0060, alt=0)
    sensor_data.attitude = Attitude(roll=0, pitch=0, yaw=0)
    sensor_data.battery_voltage = 12.0
    sensor_data.gps_fix = True
    
    autopilot.update_sensors(sensor_data)
    
    # Start autopilot
    autopilot.start_control_loop()
    
    # Arm and set flight mode
    if autopilot.arm():
        autopilot.set_flight_mode(FlightMode.STABILIZE)
        
        # Run simulation for a few seconds
        try:
            time.sleep(10)
        except KeyboardInterrupt:
            pass
        
        autopilot.disarm()
    
    autopilot.stop_control_loop()

if __name__ == "__main__":
    main()