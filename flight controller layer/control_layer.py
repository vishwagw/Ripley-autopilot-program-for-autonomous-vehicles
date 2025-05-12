"""
Flight Controller Module for Custom Autopilot System

This module implements a comprehensive flight controller that can be used as part 
of a custom autopilot system. It provides functionality for:
1. Flight state management
2. PID control loops for various control surfaces
3. Navigation algorithms
4. Safety monitoring
5. Interface to hardware sensors and control surfaces
"""

import time
import math
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple, Union


class FlightMode(Enum):
    """Flight modes supported by the autopilot system."""
    MANUAL = auto()        # Manual control, autopilot inactive
    STABILIZE = auto()      # Basic attitude stabilization
    ALT_HOLD = auto()       # Maintain current altitude
    LOITER = auto()         # Maintain position and altitude
    RTL = auto()            # Return to launch
    AUTO = auto()           # Follow pre-programmed waypoints
    GUIDED = auto()         # Accept external navigation commands
    LAND = auto()           # Autonomous landing


@dataclass
class Vector3:
    """3D vector representation for position, velocity, acceleration, etc."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def magnitude(self) -> float:
        """Calculate the magnitude of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalize(self) -> 'Vector3':
        """Return a normalized version of this vector."""
        mag = self.magnitude()
        if mag > 0:
            return Vector3(self.x/mag, self.y/mag, self.z/mag)
        return Vector3()


@dataclass
class Attitude:
    """Aircraft attitude representation using Euler angles."""
    roll: float = 0.0       # Roll angle in radians
    pitch: float = 0.0      # Pitch angle in radians
    yaw: float = 0.0        # Yaw angle in radians (heading)


@dataclass
class FlightState:
    """Current state of the aircraft."""
    timestamp: float = 0.0                     # System timestamp
    position: Vector3 = None                   # Position in NED coordinates (m)
    velocity: Vector3 = None                   # Velocity vector (m/s)
    acceleration: Vector3 = None               # Acceleration vector (m/s²)
    attitude: Attitude = None                  # Current attitude
    angular_velocity: Vector3 = None           # Angular velocity (rad/s)
    airspeed: float = 0.0                      # Airspeed (m/s)
    groundspeed: float = 0.0                   # Groundspeed (m/s)
    altitude_agl: float = 0.0                  # Altitude above ground level (m)
    altitude_amsl: float = 0.0                 # Altitude above mean sea level (m)
    vertical_speed: float = 0.0                # Vertical speed (m/s)
    battery_voltage: float = 0.0               # Battery voltage (V)
    battery_current: float = 0.0               # Battery current (A)
    battery_remaining: float = 0.0             # Battery remaining (%)
    mode: FlightMode = FlightMode.MANUAL       # Current flight mode
    armed: bool = False                        # Whether the aircraft is armed

    def __post_init__(self):
        """Initialize default values for nested objects."""
        if self.position is None:
            self.position = Vector3()
        if self.velocity is None:
            self.velocity = Vector3()
        if self.acceleration is None:
            self.acceleration = Vector3()
        if self.attitude is None:
            self.attitude = Attitude()
        if self.angular_velocity is None:
            self.angular_velocity = Vector3()


@dataclass
class Waypoint:
    """Navigation waypoint."""
    lat: float              # Latitude in degrees
    lon: float              # Longitude in degrees
    alt: float              # Altitude in meters
    speed: float = 0.0      # Target speed in m/s (0 = use default)
    heading: float = None   # Target heading in degrees (None = unrestricted)
    loiter_time: float = 0  # Time to loiter at waypoint (seconds)
    acceptance_radius: float = 10.0  # Acceptance radius in meters


class PIDController:
    """PID controller implementation for flight control loops."""
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: float = -1.0, output_max: float = 1.0,
                 integral_min: float = -1.0, integral_max: float = 1.0,
                 derivative_filter_hz: float = 20.0):
        """
        Initialize a PID controller with tuning parameters.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_min: Minimum controller output
            output_max: Maximum controller output
            integral_min: Minimum integral term
            integral_max: Maximum integral term
            derivative_filter_hz: Derivative low-pass filter cutoff frequency
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_min = integral_min
        self.integral_max = integral_max
        self.derivative_filter_hz = derivative_filter_hz
        
        # State variables
        self.last_error = 0.0
        self.integral = 0.0
        self.last_derivative = 0.0
        self.last_time = time.time()
        self.filtered_derivative = 0.0
        
    def reset(self):
        """Reset the controller state."""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_derivative = 0.0
        self.last_time = time.time()
        self.filtered_derivative = 0.0
        
    def update(self, error: float, current_time: float = None) -> float:
        """
        Update the PID controller with a new error value.
        
        Args:
            error: Current error (setpoint - measurement)
            current_time: Current time in seconds, defaults to time.time()
            
        Returns:
            float: Controller output
        """
        if current_time is None:
            current_time = time.time()
            
        # Time delta
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.01  # Fallback to prevent division by zero
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))
        i_term = self.ki * self.integral
        
        # Derivative term with filtering
        derivative = (error - self.last_error) / dt
        
        # Apply low-pass filter to derivative
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.derivative_filter_hz))
        self.filtered_derivative = self.filtered_derivative * (1.0 - alpha) + derivative * alpha
        d_term = self.kd * self.filtered_derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(self.output_min, min(output, self.output_max))
        
        # Update state
        self.last_error = error
        self.last_time = current_time
        
        return output


class SensorInterface(ABC):
    """Abstract base class for sensor interfaces."""
    
    @abstractmethod
    def read(self) -> Dict:
        """Read sensor data and return as a dictionary."""
        pass
    
    @abstractmethod
    def update(self) -> None:
        """Update sensor readings."""
        pass


class ActuatorInterface(ABC):
    """Abstract base class for actuator interfaces."""
    
    @abstractmethod
    def write(self, channel: int, value: float) -> None:
        """
        Write a value to an actuator channel.
        
        Args:
            channel: Actuator channel number
            value: Normalized control value (-1.0 to 1.0)
        """
        pass
    
    @abstractmethod
    def arm(self) -> bool:
        """Arm the actuators and return success status."""
        pass
    
    @abstractmethod
    def disarm(self) -> bool:
        """Disarm the actuators and return success status."""
        pass


class NavigationSystem:
    """Navigation system for waypoint following and path planning."""
    
    def __init__(self):
        """Initialize the navigation system."""
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.home_position = None
        
    def set_home_position(self, lat: float, lon: float, alt: float) -> None:
        """Set the home position for RTL mode."""
        self.home_position = Waypoint(lat, lon, alt)
        
    def add_waypoint(self, waypoint: Waypoint) -> None:
        """Add a waypoint to the mission."""
        self.waypoints.append(waypoint)
        
    def clear_waypoints(self) -> None:
        """Clear all waypoints."""
        self.waypoints = []
        self.current_waypoint_index = 0
        
    def get_current_waypoint(self) -> Optional[Waypoint]:
        """Get the current target waypoint."""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            return None
        return self.waypoints[self.current_waypoint_index]
    
    def next_waypoint(self) -> Optional[Waypoint]:
        """Advance to the next waypoint and return it."""
        if self.current_waypoint_index < len(self.waypoints) - 1:
            self.current_waypoint_index += 1
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def has_reached_waypoint(self, current_position: Tuple[float, float, float], 
                             waypoint: Waypoint) -> bool:
        """
        Check if the aircraft has reached the waypoint.
        
        Args:
            current_position: Current position as (lat, lon, alt)
            waypoint: Target waypoint
            
        Returns:
            bool: True if waypoint has been reached
        """
        lat, lon, alt = current_position
        
        # Calculate great-circle distance
        lat1, lon1 = math.radians(lat), math.radians(lon)
        lat2, lon2 = math.radians(waypoint.lat), math.radians(waypoint.lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters * central angle
        
        # Check if within acceptance radius
        if distance <= waypoint.acceptance_radius:
            # Also check altitude if specified
            if abs(alt - waypoint.alt) <= waypoint.acceptance_radius:
                return True
        
        return False
    
    def calculate_bearing(self, from_position: Tuple[float, float], 
                          to_position: Tuple[float, float]) -> float:
        """
        Calculate bearing from one position to another.
        
        Args:
            from_position: Starting position as (lat, lon)
            to_position: Target position as (lat, lon)
            
        Returns:
            float: Bearing in degrees (0-360)
        """
        lat1, lon1 = math.radians(from_position[0]), math.radians(from_position[1])
        lat2, lon2 = math.radians(to_position[0]), math.radians(to_position[1])
        
        y = math.sin(lon2 - lon1) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1)
        bearing = math.atan2(y, x)
        
        # Convert to degrees and normalize to 0-360
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        
        return bearing


class AttitudeController:
    """
    Attitude controller using PID loops for roll, pitch, and yaw control.
    This controller generates actuator commands to achieve desired attitudes.
    """
    
    def __init__(self):
        """Initialize the attitude controller with PID controllers for each axis."""
        # Default PID values - these would typically be tuned for a specific aircraft
        self.roll_controller = PIDController(kp=0.5, ki=0.1, kd=0.05)
        self.pitch_controller = PIDController(kp=0.5, ki=0.1, kd=0.05)
        self.yaw_controller = PIDController(kp=0.7, ki=0.0, kd=0.1)
        
        # Desired attitude
        self.desired_attitude = Attitude()
        
    def set_desired_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        """Set the desired attitude angles in radians."""
        self.desired_attitude.roll = roll
        self.desired_attitude.pitch = pitch
        self.desired_attitude.yaw = yaw
        
    def update(self, current_attitude: Attitude) -> Dict[str, float]:
        """
        Update attitude controller and get control surface commands.
        
        Args:
            current_attitude: Current aircraft attitude
            
        Returns:
            Dict[str, float]: Control surface commands (-1.0 to 1.0) for aileron, elevator, rudder
        """
        # Calculate errors (shortest angular distance)
        roll_error = self._wrap_pi(self.desired_attitude.roll - current_attitude.roll)
        pitch_error = self._wrap_pi(self.desired_attitude.pitch - current_attitude.pitch)
        
        # For yaw, we need to handle the -pi to pi transition carefully
        yaw_error = self._wrap_pi(self.desired_attitude.yaw - current_attitude.yaw)
        
        # Update PID controllers
        aileron_cmd = self.roll_controller.update(roll_error)
        elevator_cmd = self.pitch_controller.update(pitch_error)
        rudder_cmd = self.yaw_controller.update(yaw_error)
        
        return {
            "aileron": aileron_cmd,
            "elevator": elevator_cmd,
            "rudder": rudder_cmd
        }
    
    @staticmethod
    def _wrap_pi(angle: float) -> float:
        """Wrap angle to -pi..pi range."""
        return ((angle + math.pi) % (2 * math.pi)) - math.pi


class AltitudeController:
    """
    Altitude controller for maintaining desired altitude.
    This controller works with the attitude controller to achieve and maintain altitude.
    """
    
    def __init__(self):
        """Initialize the altitude controller."""
        # PID controller for vertical speed
        self.altitude_pid = PIDController(kp=0.3, ki=0.05, kd=0.1)
        self.vertical_speed_pid = PIDController(kp=0.5, ki=0.1, kd=0.05)
        
        # Desired altitude and climb rate
        self.desired_altitude = 0.0
        self.desired_vertical_speed = 0.0
        self.max_climb_rate = 3.0  # m/s
        self.max_descent_rate = -2.0  # m/s
        
    def set_desired_altitude(self, altitude: float) -> None:
        """Set the desired altitude in meters."""
        self.desired_altitude = altitude
        
    def set_climb_rate(self, climb_rate: float) -> None:
        """
        Set the desired climb/descent rate in m/s.
        Positive values for climb, negative for descent.
        """
        self.desired_vertical_speed = max(self.max_descent_rate, 
                                         min(climb_rate, self.max_climb_rate))
        
    def update(self, current_altitude: float, current_vertical_speed: float) -> float:
        """
        Update altitude controller and get pitch command.
        
        Args:
            current_altitude: Current altitude in meters
            current_vertical_speed: Current vertical speed in m/s
            
        Returns:
            float: Pitch command to achieve/maintain the desired altitude
        """
        if self.desired_vertical_speed != 0:
            # We're in climb/descent mode - control vertical speed directly
            vs_error = self.desired_vertical_speed - current_vertical_speed
            pitch_cmd = self.vertical_speed_pid.update(vs_error)
        else:
            # We're in altitude hold mode - first get desired vertical speed
            alt_error = self.desired_altitude - current_altitude
            target_vs = self.altitude_pid.update(alt_error)
            
            # Limit the target vertical speed
            target_vs = max(self.max_descent_rate, min(target_vs, self.max_climb_rate))
            
            # Then control to achieve that vertical speed
            vs_error = target_vs - current_vertical_speed
            pitch_cmd = self.vertical_speed_pid.update(vs_error)
            
        return pitch_cmd


class SpeedController:
    """
    Speed controller for maintaining desired airspeed.
    This controller outputs throttle commands to achieve the desired speed.
    """
    
    def __init__(self):
        """Initialize the speed controller."""
        self.speed_pid = PIDController(kp=0.1, ki=0.05, kd=0.01)
        self.desired_airspeed = 0.0
        
    def set_desired_airspeed(self, airspeed: float) -> None:
        """Set the desired airspeed in m/s."""
        self.desired_airspeed = airspeed
        
    def update(self, current_airspeed: float) -> float:
        """
        Update speed controller and get throttle command.
        
        Args:
            current_airspeed: Current airspeed in m/s
            
        Returns:
            float: Throttle command (0.0 to 1.0)
        """
        # Calculate error and update PID
        speed_error = self.desired_airspeed - current_airspeed
        throttle_cmd = self.speed_pid.update(speed_error)
        
        # Ensure throttle is always positive (0 to 1 range)
        throttle_cmd = max(0.0, min(throttle_cmd + 0.5, 1.0))
        
        return throttle_cmd


class FlightController:
    """
    Main flight controller class that integrates sensors, actuators,
    and control algorithms to implement autopilot functionality.
    """
    
    def __init__(self, sensor_interface: SensorInterface, actuator_interface: ActuatorInterface):
        """
        Initialize the flight controller.
        
        Args:
            sensor_interface: Interface to aircraft sensors
            actuator_interface: Interface to aircraft actuators
        """
        self.sensors = sensor_interface
        self.actuators = actuator_interface
        
        # Initialize controllers
        self.attitude_controller = AttitudeController()
        self.altitude_controller = AltitudeController()
        self.speed_controller = SpeedController()
        self.navigation = NavigationSystem()
        
        # Flight state
        self.state = FlightState()
        
        # Control parameters
        self.control_frequency = 50  # Hz
        self.control_period = 1.0 / self.control_frequency
        self.last_control_time = 0.0
        
        # Safety parameters
        self.max_bank_angle = math.radians(45)  # Maximum roll angle
        self.max_pitch_angle = math.radians(30)  # Maximum pitch angle
        
        # Failsafe flags
        self.failsafe_triggered = False
        self.low_battery_warning = False
        
    def update_flight_state(self) -> None:
        """Update the flight state from sensor readings."""
        # Read sensor data
        self.sensors.update()
        sensor_data = self.sensors.read()
        
        # Update state with sensor readings
        # (This mapping would be specific to the sensor interface implementation)
        self.state.timestamp = time.time()
        
        # Update position data if available
        if 'position' in sensor_data:
            pos = sensor_data['position']
            self.state.position.x = pos.get('x', self.state.position.x)
            self.state.position.y = pos.get('y', self.state.position.y)
            self.state.position.z = pos.get('z', self.state.position.z)
        
        # Update velocity data if available
        if 'velocity' in sensor_data:
            vel = sensor_data['velocity']
            self.state.velocity.x = vel.get('x', self.state.velocity.x)
            self.state.velocity.y = vel.get('y', self.state.velocity.y)
            self.state.velocity.z = vel.get('z', self.state.velocity.z)
            
            # Calculate groundspeed
            self.state.groundspeed = math.sqrt(vel.get('x', 0)**2 + vel.get('y', 0)**2)
            
            # Update vertical speed
            self.state.vertical_speed = -vel.get('z', 0)  # Negative because NED coordinates
        
        # Update attitude data
        if 'attitude' in sensor_data:
            att = sensor_data['attitude']
            self.state.attitude.roll = att.get('roll', self.state.attitude.roll)
            self.state.attitude.pitch = att.get('pitch', self.state.attitude.pitch)
            self.state.attitude.yaw = att.get('yaw', self.state.attitude.yaw)
        
        # Update angular velocity data
        if 'angular_velocity' in sensor_data:
            ang_vel = sensor_data['angular_velocity']
            self.state.angular_velocity.x = ang_vel.get('x', self.state.angular_velocity.x)
            self.state.angular_velocity.y = ang_vel.get('y', self.state.angular_velocity.y)
            self.state.angular_velocity.z = ang_vel.get('z', self.state.angular_velocity.z)
        
        # Update airspeed data
        if 'airspeed' in sensor_data:
            self.state.airspeed = sensor_data['airspeed']
            
        # Update altitude data
        if 'altitude_agl' in sensor_data:
            self.state.altitude_agl = sensor_data['altitude_agl']
        if 'altitude_amsl' in sensor_data:
            self.state.altitude_amsl = sensor_data['altitude_amsl']
            
        # Update battery data
        if 'battery' in sensor_data:
            bat = sensor_data['battery']
            self.state.battery_voltage = bat.get('voltage', self.state.battery_voltage)
            self.state.battery_current = bat.get('current', self.state.battery_current)
            self.state.battery_remaining = bat.get('remaining', self.state.battery_remaining)
            
            # Check for low battery
            if self.state.battery_remaining < 20.0:
                self.low_battery_warning = True
        
    def set_mode(self, mode: FlightMode) -> bool:
        """
        Set the flight mode.
        
        Args:
            mode: The desired flight mode
            
        Returns:
            bool: True if the mode was set successfully
        """
        # Check if mode change is allowed
        if self.failsafe_triggered and mode != FlightMode.RTL and mode != FlightMode.LAND:
            # Failsafe active - only allow RTL or LAND modes
            return False
            
        # Set the new mode
        self.state.mode = mode
        
        # Reset relevant controllers based on the new mode
        if mode == FlightMode.ALT_HOLD or mode == FlightMode.LOITER:
            # When entering altitude hold, set the desired altitude to current
            self.altitude_controller.set_desired_altitude(self.state.altitude_amsl)
            
        if mode == FlightMode.STABILIZE:
            # Reset rate controllers
            self.attitude_controller.roll_controller.reset()
            self.attitude_controller.pitch_controller.reset()
            self.attitude_controller.yaw_controller.reset()
            
        return True
        
    def arm(self) -> bool:
        """
        Arm the aircraft if safe to do so.
        
        Returns:
            bool: True if armed successfully
        """
        # Check safety conditions
        if self.failsafe_triggered:
            return False
            
        # Try to arm the actuators
        if self.actuators.arm():
            self.state.armed = True
            return True
        return False
        
    def disarm(self) -> bool:
        """
        Disarm the aircraft.
        
        Returns:
            bool: True if disarmed successfully
        """
        if self.actuators.disarm():
            self.state.armed = False
            return True
        return False
        
    def check_safety(self) -> bool:
        """
        Check for any safety issues.
        
        Returns:
            bool: True if safe to continue, False if a failsafe has been triggered
        """
        # Check battery level
        if self.state.battery_remaining < 10.0:
            # Critical battery level
            self.failsafe_triggered = True
            self.set_mode(FlightMode.RTL)  # Return to launch
            return False
            
        # Check for excessive attitudes
        if abs(self.state.attitude.roll) > self.max_bank_angle or \
           abs(self.state.attitude.pitch) > self.max_pitch_angle:
            # Aircraft is in an unusual attitude
            # For now, just note this, but could trigger a failsafe
            pass
            
        return not self.failsafe_triggered
        
    def run_control_loop(self) -> None:
        """Run the main control loop at the specified frequency."""
        current_time = time.time()
        
        # Check if it's time to run the control loop
        if current_time - self.last_control_time < self.control_period:
            return
            
        # Update our state from sensors
        self.update_flight_state()
        
        # Check safety conditions
        if not self.check_safety():
            # A failsafe has been triggered
            pass
        
        # Execute control logic based on the current mode
        if self.state.armed:
            control_outputs = self._execute_flight_mode()
            
            # Apply control outputs to actuators
            for channel, value in control_outputs.items():
                if channel == "throttle":
                    # Map throttle from 0-1 to channel range
                    self.actuators.write(0, value)
                elif channel == "aileron":
                    self.actuators.write(1, value)
                elif channel == "elevator":
                    self.actuators.write(2, value)
                elif channel == "rudder":
                    self.actuators.write(3, value)
                    
        # Update last control time
        self.last_control_time = current_time
        
    def _execute_flight_mode(self) -> Dict[str, float]:
        """
        Execute control logic for the current flight mode.
        
        Returns:
            Dict[str, float]: Control outputs for throttle, aileron, elevator, rudder
        """
        # Default control outputs
        control_outputs = {
            "throttle": 0.0,  # 0 to 1.0
            "aileron": 0.0,   # -1.0 to 1.0
            "elevator": 0.0,  # -1.0 to 1.0
            "rudder": 0.0     # -1.0 to 1.0
        }
        
        # Handle different flight modes
        if self.state.mode == FlightMode.MANUAL:
            # In manual mode, don't apply any control (outputs will come from RC)
            pass
            
        elif self.state.mode == FlightMode.STABILIZE:
            # Basic attitude stabilization - keep wings level
            self.attitude_controller.set_desired_attitude(0.0, 0.0, self.state.attitude.yaw)
            attitude_outputs = self.attitude_controller.update(self.state.attitude)
            
            # Apply attitude controller outputs
            control_outputs["aileron"] = attitude_outputs["aileron"]
            control_outputs["elevator"] = attitude_outputs["elevator"]
            control_outputs["rudder"] = attitude_outputs["rudder"]
            
            # In stabilize mode, throttle is manual
            
        elif self.state.mode == FlightMode.ALT_HOLD:
            # Keep wings level and maintain altitude
            self.attitude_controller.set_desired_attitude(0.0, 0.0, self.state.attitude.yaw)
            attitude_outputs = self.attitude_controller.update(self.state.attitude)
            
            # Get pitch command from altitude controller
            pitch_cmd = self.altitude_controller.update(
                self.state.altitude_amsl, self.state.vertical_speed)
                
            # Blend pitch command with attitude stabilization
            control_outputs["aileron"] = attitude_outputs["aileron"]
            control_outputs["elevator"] = pitch_cmd
            control_outputs["rudder"] = attitude_outputs["rudder"]
            
            # Maintain airspeed with throttle
            control_outputs["throttle"] = self.speed_controller.update(self.state.airspeed)
            
        elif self.state.mode == FlightMode.LOITER:
            # Maintain position and altitude
            # This would typically use GPS position control (not fully implemented here)
            
            # For now, just maintain altitude and attitude
            self.attitude_controller.set_desired_attitude(0.0, 0.0, self.state.attitude.yaw)
            attitude_outputs = self.attitude_controller.update(self.state.attitude)
            
            pitch_cmd = self.altitude_controller.update(
                self.state.altitude_amsl, self.state.vertical_speed)
                
            control_outputs["aileron"] = attitude_outputs["aileron"]
            control_outputs["elevator"] = pitch_cmd
            control_outputs["rudder"] = attitude_outputs["rudder"]
            control_outputs["throttle"] = self.speed_controller.update(self.state.airspeed)
            
        elif self.state.mode == FlightMode.RTL:
            # Return to launch
            if self.navigation.home_position:
                # Get current lat/lon from position (implementation would depend on coordinate system)
                current_lat = 0.0  # This would be derived from self.state.position
                current_lon = 0.0  # This would be derived from self.state.position
                
                # Calculate bearing to home
                bearing = self.navigation.calculate_bearing(
                    (current_lat, current_lon),
                    (self.navigation.home_position.lat, self.navigation.home_position.lon)
                )
                
                # Convert bearing to yaw angle (radians)
                target_yaw = math.radians(bearing)
                
                # Set desired attitude to turn towards home
                # Bank angle proportional to heading error (max 30 degrees)
                heading_error = self._wrap_angle(target_yaw - self.state.attitude.yaw)
                bank_angle = min(math.radians(30), max(-math.radians(30), heading_error * 0.5))
                
                self.attitude_controller.set_desired_attitude(bank_angle, 0.0, target_yaw)
                attitude_outputs = self.attitude_controller.update(self.state.attitude)
                
                # Set altitude to home altitude or safe RTL altitude (whichever is higher)
                safe_rtl_altitude = self.navigation.home_position.alt + 50  # 50m above home
                self.altitude_controller.set_desired_altitude(
                    max(safe_rtl_altitude, self.state.altitude_amsl)
                )
                pitch_cmd = self.altitude_controller.update(
                    self.state.altitude_amsl, self.state.vertical_speed
                )
                
                control_outputs["aileron"] = attitude_outputs["aileron"]
                control_outputs["elevator"] = pitch_cmd
                control_outputs["rudder"] = attitude_outputs["rudder"]
                control_outputs["throttle"] = self.speed_controller.update(self.state.airspeed)
            
        elif self.state.mode == FlightMode.AUTO:
            # Follow waypoints
            waypoint = self.navigation.get_current_waypoint()
            if waypoint:
                # Convert position to lat/lon (would depend on coordinate system)
                current_lat = 0.0  # This would be derived from self.state.position
                current_lon = 0.0  # This would be derived from self.state.position
                current_alt = self.state.altitude_amsl
                
                # Check if we've reached the waypoint
                if self.navigation.has_reached_waypoint(
                    (current_lat, current_lon, current_alt), waypoint
                ):
                    # Move to next waypoint
                    waypoint = self.navigation.next_waypoint()
                    if not waypoint:
                        # End of mission, switch to LOITER
                        self.set_mode(FlightMode.LOITER)
                        return self._execute_flight_mode()
                
                # Calculate bearing to waypoint
                bearing = self.navigation.calculate_bearing(
                    (current_lat, current_lon),
                    (waypoint.lat, waypoint.lon)
                )
                
                # Convert bearing to yaw angle (radians)
                target_yaw = math.radians(bearing)
                
                # Calculate bank angle for turn (proportional to heading error)
                heading_error = self._wrap_angle(target_yaw - self.state.attitude.yaw)
                bank_angle = min(math.radians(30), max(-math.radians(30), heading_error * 0.5))
                
                # Set desired attitude
                self.attitude_controller.set_desired_attitude(bank_angle, 0.0, target_yaw)
                attitude_outputs = self.attitude_controller.update(self.state.attitude)
                
                # Set altitude to waypoint altitude
                self.altitude_controller.set_desired_altitude(waypoint.alt)
                pitch_cmd = self.altitude_controller.update(
                    self.state.altitude_amsl, self.state.vertical_speed
                )
                
                # Set airspeed based on waypoint speed
                if waypoint.speed > 0:
                    self.speed_controller.set_desired_airspeed(waypoint.speed)
                
                control_outputs["aileron"] = attitude_outputs["aileron"]
                control_outputs["elevator"] = pitch_cmd
                control_outputs["rudder"] = attitude_outputs["rudder"]
                control_outputs["throttle"] = self.speed_controller.update(self.state.airspeed)
            else:
                # No waypoints, switch to LOITER
                self.set_mode(FlightMode.LOITER)
                return self._execute_flight_mode()
                
        elif self.state.mode == FlightMode.LAND:
            # Autonomous landing
            # Maintain wings level
            self.attitude_controller.set_desired_attitude(0.0, 0.0, self.state.attitude.yaw)
            attitude_outputs = self.attitude_controller.update(self.state.attitude)
            
            # Set descent rate based on altitude
            if self.state.altitude_agl > 50:
                # Higher altitude - descend at moderate rate
                descent_rate = -2.0
            elif self.state.altitude_agl > 10:
                # Getting closer - slow descent
                descent_rate = -1.0
            elif self.state.altitude_agl > 3:
                # Final approach - very slow descent
                descent_rate = -0.5
            else:
                # About to touch down - minimal descent
                descent_rate = -0.2
                
            # Set desired vertical speed
            self.altitude_controller.set_climb_rate(descent_rate)
            pitch_cmd = self.altitude_controller.update(
                self.state.altitude_amsl, self.state.vertical_speed
            )
            
            # Reduce airspeed as we get closer to the ground
            approach_speed = max(1.3 * self.min_airspeed, self.nominal_airspeed * 0.8)
            if self.state.altitude_agl < 10:
                approach_speed = max(1.2 * self.min_airspeed, self.nominal_airspeed * 0.7)
            
            self.speed_controller.set_desired_airspeed(approach_speed)
            
            # Apply control outputs
            control_outputs["aileron"] = attitude_outputs["aileron"]
            control_outputs["elevator"] = pitch_cmd
            control_outputs["rudder"] = attitude_outputs["rudder"]
            
            # Cut throttle when very close to the ground
            if self.state.altitude_agl < 1.0:
                control_outputs["throttle"] = 0.0
            else:
                control_outputs["throttle"] = self.speed_controller.update(self.state.airspeed)
                
        elif self.state.mode == FlightMode.GUIDED:
            # Similar to AUTO but takes dynamic waypoints from external source
            # Implementation would be similar to AUTO mode
            pass
            
        # Apply safety limits to all control outputs
        control_outputs = self._apply_safety_limits(control_outputs)
        
        return control_outputs
        
    def _apply_safety_limits(self, control_outputs: Dict[str, float]) -> Dict[str, float]:
        """
        Apply safety limits to control outputs.
        
        Args:
            control_outputs: Raw control outputs
            
        Returns:
            Dict[str, float]: Limited control outputs
        """
        # Ensure all values are within valid ranges
        for channel, value in control_outputs.items():
            if channel == "throttle":
                # Throttle is 0.0 to 1.0
                control_outputs[channel] = max(0.0, min(value, 1.0))
            else:
                # Other controls are -1.0 to 1.0
                control_outputs[channel] = max(-1.0, min(value, 1.0))
                
        return control_outputs
        
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to -pi..pi range."""
        return ((angle + math.pi) % (2 * math.pi)) - math.pi
        
    def set_waypoints(self, waypoints: List[Waypoint]) -> None:
        """
        Set mission waypoints.
        
        Args:
            waypoints: List of waypoints to follow
        """
        self.navigation.clear_waypoints()
        for waypoint in waypoints:
            self.navigation.add_waypoint(waypoint)
            
    def set_home_position(self, lat: float, lon: float, alt: float) -> None:
        """
        Set home position for RTL mode.
        
        Args:
            lat: Latitude in degrees
            lon: Longitude in degrees
            alt: Altitude in meters
        """
        self.navigation.set_home_position(lat, lon, alt)
        
    def get_telemetry(self) -> Dict:
        """
        Get current telemetry data.
        
        Returns:
            Dict: Current flight state data
        """
        return {
            "timestamp": self.state.timestamp,
            "position": {
                "x": self.state.position.x,
                "y": self.state.position.y,
                "z": self.state.position.z
            },
            "velocity": {
                "x": self.state.velocity.x,
                "y": self.state.velocity.y,
                "z": self.state.velocity.z,
                "airspeed": self.state.airspeed,
                "groundspeed": self.state.groundspeed,
                "vertical_speed": self.state.vertical_speed
            },
            "attitude": {
                "roll": self.state.attitude.roll,
                "pitch": self.state.attitude.pitch,
                "yaw": self.state.attitude.yaw
            },
            "angular_velocity": {
                "x": self.state.angular_velocity.x,
                "y": self.state.angular_velocity.y,
                "z": self.state.angular_velocity.z
            },
            "altitude": {
                "agl": self.state.altitude_agl,
                "amsl": self.state.altitude_amsl
            },
            "battery": {
                "voltage": self.state.battery_voltage,
                "current": self.state.battery_current,
                "remaining": self.state.battery_remaining
            },
            "armed": self.state.armed,
            "mode": self.state.mode.name,
            "failsafe": self.failsafe_triggered,
            "low_battery": self.low_battery_warning
        }


class MockSensorInterface(SensorInterface):
    """Mock sensor interface for testing."""
    
    def __init__(self):
        """Initialize the mock sensor interface."""
        self.position = Vector3(0, 0, -100)  # 100m altitude
        self.velocity = Vector3(10, 0, 0)    # 10 m/s forward
        self.attitude = Attitude(0, 0, 0)    # Level flight
        self.angular_velocity = Vector3(0, 0, 0)
        self.airspeed = 10.0
        self.altitude_agl = 100.0
        self.altitude_amsl = 500.0
        self.battery_voltage = 12.6
        self.battery_current = 5.0
        self.battery_remaining = 80.0
        
    def read(self) -> Dict:
        """Read sensor data."""
        return {
            "position": {
                "x": self.position.x,
                "y": self.position.y,
                "z": self.position.z
            },
            "velocity": {
                "x": self.velocity.x,
                "y": self.velocity.y,
                "z": self.velocity.z
            },
            "attitude": {
                "roll": self.attitude.roll,
                "pitch": self.attitude.pitch,
                "yaw": self.attitude.yaw
            },
            "angular_velocity": {
                "x": self.angular_velocity.x,
                "y": self.angular_velocity.y,
                "z": self.angular_velocity.z
            },
            "airspeed": self.airspeed,
            "altitude_agl": self.altitude_agl,
            "altitude_amsl": self.altitude_amsl,
            "battery": {
                "voltage": self.battery_voltage,
                "current": self.battery_current,
                "remaining": self.battery_remaining
            }
        }
        
    def update(self) -> None:
        """Update sensor readings (simulation)."""
        # Simple simulation logic
        dt = 0.02  # 50Hz
        
        # Update position based on velocity
        self.position.x += self.velocity.x * dt
        self.position.y += self.velocity.y * dt
        self.position.z += self.velocity.z * dt
        
        # Simulate battery drain
        self.battery_remaining -= 0.001


class MockActuatorInterface(ActuatorInterface):
    """Mock actuator interface for testing."""
    
    def __init__(self):
        """Initialize mock actuator interface."""
        self.channels = [0.0] * 8  # 8 PWM channels
        self.armed = False
        
    def write(self, channel: int, value: float) -> None:
        """Write value to actuator channel."""
        if 0 <= channel < len(self.channels):
            self.channels[channel] = value
            
    def arm(self) -> bool:
        """Arm the actuators."""
        self.armed = True
        return True
        
    def disarm(self) -> bool:
        """Disarm the actuators."""
        self.armed = False
        return True


def create_sample_mission() -> List[Waypoint]:
    """Create a sample mission with waypoints."""
    # Example waypoints for a simple rectangular pattern
    waypoints = [
        Waypoint(lat=47.123, lon=-122.456, alt=100.0, speed=15.0),
        Waypoint(lat=47.124, lon=-122.456, alt=100.0, speed=15.0),
        Waypoint(lat=47.124, lon=-122.457, alt=100.0, speed=15.0),
        Waypoint(lat=47.123, lon=-122.457, alt=100.0, speed=15.0),
        Waypoint(lat=47.123, lon=-122.456, alt=100.0, speed=15.0)  # Return to start
    ]
    return waypoints


def main():
    """Main function to demonstrate the flight controller."""
    # Create sensor and actuator interfaces
    sensors = MockSensorInterface()
    actuators = MockActuatorInterface()
    
    # Create flight controller
    controller = FlightController(sensors, actuators)
    
    # Set home position
    controller.set_home_position(47.123, -122.456, 50.0)
    
    # Create and set a mission
    mission = create_sample_mission()
    controller.set_waypoints(mission)
    
    # Arm the aircraft
    if controller.arm():
        print("Aircraft armed successfully")
    else:
        print("Failed to arm aircraft")
        return
    
    # Set initial mode
    controller.set_mode(FlightMode.STABILIZE)
    print(f"Mode set to {controller.state.mode.name}")
    
    # Run a simple simulation loop
    try:
        print("Starting control loop simulation...")
        for _ in range(100):  # Run for 100 iterations
            controller.run_control_loop()
            
            # Print some telemetry
            telem = controller.get_telemetry()
            print(f"Mode: {telem['mode']}, "
                  f"Alt: {telem['altitude']['amsl']:.1f}m, "
                  f"Speed: {telem['velocity']['airspeed']:.1f}m/s, "
                  f"Attitude: [{telem['attitude']['roll']:.2f}, "
                  f"{telem['attitude']['pitch']:.2f}, "
                  f"{telem['attitude']['yaw']:.2f}]")
                  
            # Change mode after 20 iterations
            if _ == 20:
                controller.set_mode(FlightMode.ALT_HOLD)
                print(f"Mode changed to {controller.state.mode.name}")
                
            # Change mode after 40 iterations
            if _ == 40:
                controller.set_mode(FlightMode.AUTO)
                print(f"Mode changed to {controller.state.mode.name}")
                
            # Change mode after 80 iterations
            if _ == 80:
                controller.set_mode(FlightMode.RTL)
                print(f"Mode changed to {controller.state.mode.name}")
                
            time.sleep(0.02)  # 50Hz
            
        # Disarm at the end
        controller.disarm()
        print("Aircraft disarmed")
        
    except KeyboardInterrupt:
        print("Simulation stopped by user")
        controller.disarm()


if __name__ == "__main__":
    main()