# version 1.3: 
# added primary flight display (PFD) unit
import tkinter as tk
from tkinter import ttk
import math
import time
import threading
import random
import cv2
from PIL import Image, ImageTk

class PrimaryFlightDisplay:
    """Class to handle drawing a Primary Flight Display"""
    def __init__(self, canvas, x, y, width, height):
        self.canvas = canvas
        self.x = x  # top-left corner X
        self.y = y  # top-left corner Y
        self.width = width
        self.height = height
        
        # Colors
        self.sky_color = "#3870B0"  # Blue
        self.ground_color = "#8B4513"  # Brown
        self.line_color = "white"
        self.text_color = "white"
        self.speed_tape_color = "#222222"
        self.alt_tape_color = "#222222"
        
        # Initialize values
        self.roll = 0  # degrees
        self.pitch = 0  # degrees
        self.altitude = 0  # meters
        self.speed = 0  # m/s
        self.heading = 0  # degrees
        self.vertical_speed = 0  # m/s
        
    def update(self, roll, pitch, altitude, speed, heading, vertical_speed=0):
        """Update flight parameters"""
        self.roll = roll
        self.pitch = pitch
        self.altitude = altitude
        self.speed = speed
        self.heading = heading
        self.vertical_speed = vertical_speed
        
    def draw(self):
        """Draw the entire PFD"""
        self.canvas.delete("pfd")  # Clear previous PFD
        
        # Draw background
        self.canvas.create_rectangle(
            self.x, self.y, self.x + self.width, self.y + self.height,
            fill="#111111", outline="#444444", width=2, tags="pfd"
        )
        
        # Draw title
        self.canvas.create_text(
            self.x + self.width // 2, self.y + 12,
            text="PRIMARY FLIGHT DISPLAY",
            fill=self.text_color,
            font=("Arial", 8, "bold"),
            tags="pfd"
        )
        
        # Draw artificial horizon
        self._draw_attitude_indicator()
        
        # Draw speed tape
        self._draw_speed_tape()
        
        # Draw altitude tape
        self._draw_altitude_tape()
        
        # Draw heading indicator
        self._draw_heading_indicator()
        
        # Draw vertical speed indicator
        self._draw_vertical_speed()
        
    def _draw_attitude_indicator(self):
        """Draw artificial horizon (attitude indicator)"""
        # Define center and radius for attitude indicator
        center_x = self.x + self.width // 2
        center_y = self.y + self.height // 2
        radius = min(self.width, self.height) // 3
        
        # Calculate pitch offset (10 pixels per 10 degrees)
        pitch_offset = self.pitch * (radius / 30)
        
        # Save current state
        self.canvas.save()
        
        # Set clipping region to circle
        self.canvas.create_oval(
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            outline=self.line_color, width=2, tags="pfd"
        )
        
        # Rotate canvas for roll
        self.canvas.rotate(center_x, center_y, -self.roll)
        
        # Draw sky and ground
        horizon_y = center_y + pitch_offset
        
        # Sky
        self.canvas.create_rectangle(
            center_x - radius, center_y - radius,
            center_x + radius, horizon_y,
            fill=self.sky_color, outline="", tags="pfd"
        )
        
        # Ground
        self.canvas.create_rectangle(
            center_x - radius, horizon_y,
            center_x + radius, center_y + radius,
            fill=self.ground_color, outline="", tags="pfd"
        )
        
        # Draw horizon line
        self.canvas.create_line(
            center_x - radius, horizon_y,
            center_x + radius, horizon_y,
            fill=self.line_color, width=2, tags="pfd"
        )
        
        # Draw pitch lines (every 10 degrees)
        for i in range(-30, 31, 10):
            if i == 0:
                continue  # Skip horizon line as it's already drawn
                
            line_y = horizon_y - i * (radius / 30)
            line_width = radius * (0.8 if i % 20 == 0 else 0.5)
            
            # Draw pitch line
            self.canvas.create_line(
                center_x - line_width/2, line_y,
                center_x + line_width/2, line_y,
                fill=self.line_color, width=2, tags="pfd"
            )
            
            # Draw pitch value
            if i != 0:
                self.canvas.create_text(
                    center_x - line_width/2 - 10, line_y,
                    text=str(abs(i)),
                    fill=self.text_color,
                    font=("Arial", 8),
                    tags="pfd"
                )
                self.canvas.create_text(
                    center_x + line_width/2 + 10, line_y,
                    text=str(abs(i)),
                    fill=self.text_color,
                    font=("Arial", 8),
                    tags="pfd"
                )
        
        # Restore canvas state
        self.canvas.restore()
        
        # Draw roll indicator (fixed)
        self.canvas.create_arc(
            center_x - radius - 10, center_y - radius - 10,
            center_x + radius + 10, center_y + radius + 10,
            start=30, extent=120,
            style=tk.ARC,
            outline=self.line_color,
            tags="pfd"
        )
        
        # Draw roll markers
        for angle in [-60, -45, -30, -15, 0, 15, 30, 45, 60]:
            marker_angle = 90 - angle
            marker_radius = radius + 5
            
            x = center_x + marker_radius * math.cos(math.radians(marker_angle))
            y = center_y - marker_radius * math.sin(math.radians(marker_angle))
            
            marker_len = 7 if angle % 30 == 0 else 5
            
            end_x = center_x + (marker_radius - marker_len) * math.cos(math.radians(marker_angle))
            end_y = center_y - (marker_radius - marker_len) * math.sin(math.radians(marker_angle))
            
            self.canvas.create_line(
                x, y, end_x, end_y,
                fill=self.line_color, width=2, tags="pfd"
            )
            
            if angle % 30 == 0:
                text_x = center_x + (marker_radius + 10) * math.cos(math.radians(marker_angle))
                text_y = center_y - (marker_radius + 10) * math.sin(math.radians(marker_angle))
                
                self.canvas.create_text(
                    text_x, text_y,
                    text=str(abs(angle)),
                    fill=self.text_color,
                    font=("Arial", 8),
                    tags="pfd"
                )
        
        # Draw roll indicator
        roll_angle = 90 - self.roll
        roll_x = center_x + (radius + 5) * math.cos(math.radians(roll_angle))
        roll_y = center_y - (radius + 5) * math.sin(math.radians(roll_angle))
        
        self.canvas.create_polygon(
            roll_x - 7, roll_y + 7,
            roll_x, roll_y - 7,
            roll_x + 7, roll_y + 7,
            fill=self.line_color, tags="pfd"
        )
        
        # Draw fixed aircraft symbol
        self.canvas.create_line(
            center_x - 30, center_y,
            center_x - 10, center_y,
            fill=self.line_color, width=3, tags="pfd"
        )
        self.canvas.create_line(
            center_x + 10, center_y,
            center_x + 30, center_y,
            fill=self.line_color, width=3, tags="pfd"
        )
        self.canvas.create_rectangle(
            center_x - 5, center_y - 5,
            center_x + 5, center_y + 5,
            fill=self.line_color, tags="pfd"
        )
        
    def _draw_speed_tape(self):
        """Draw speed tape on the left side"""
        # Speed tape position and dimensions
        tape_x = self.x + 30
        tape_y = self.y + self.height // 2
        tape_width = 40
        tape_height = 120
        
        # Draw speed tape background
        self.canvas.create_rectangle(
            tape_x - tape_width//2, tape_y - tape_height//2,
            tape_x + tape_width//2, tape_y + tape_height//2,
            fill=self.speed_tape_color, outline=self.line_color, tags="pfd"
        )
        
        # Draw speed range (current speed +/- 20)
        min_speed = max(0, int(self.speed) - 20)
        max_speed = int(self.speed) + 20
        
        # Speed tick marks
        for spd in range(min_speed, max_speed + 1, 5):
            # Calculate vertical position
            y_pos = tape_y + (self.speed - spd) * (tape_height / 40)
            
            if y_pos < tape_y - tape_height//2 or y_pos > tape_y + tape_height//2:
                continue
                
            # Draw tick mark
            tick_width = 10 if spd % 10 == 0 else 5
            self.canvas.create_line(
                tape_x - tick_width, y_pos,
                tape_x, y_pos,
                fill=self.line_color, tags="pfd"
            )
            
            # Draw speed label (only for round numbers)
            if spd % 10 == 0:
                self.canvas.create_text(
                    tape_x - 20, y_pos,
                    text=str(spd),
                    fill=self.text_color,
                    font=("Arial", 8),
                    anchor=tk.E,
                    tags="pfd"
                )
        
        # Draw current speed box
        self.canvas.create_rectangle(
            tape_x - tape_width//2 - 5, tape_y - 10,
            tape_x + tape_width//2, tape_y + 10,
            fill="black", outline=self.line_color, tags="pfd"
        )
        
        # Draw current speed value
        self.canvas.create_text(
            tape_x - 5, tape_y,
            text=f"{self.speed:.1f}",
            fill=self.text_color,
            font=("Arial", 10, "bold"),
            anchor=tk.E,
            tags="pfd"
        )
        
        # Label
        self.canvas.create_text(
            tape_x, tape_y - tape_height//2 - 10,
            text="SPD",
            fill=self.text_color,
            font=("Arial", 8),
            tags="pfd"
        )
        
    def _draw_altitude_tape(self):
        """Draw altitude tape on the right side"""
        # Altitude tape position and dimensions
        tape_x = self.x + self.width - 30
        tape_y = self.y + self.height // 2
        tape_width = 40
        tape_height = 120
        
        # Draw altitude tape background
        self.canvas.create_rectangle(
            tape_x - tape_width//2, tape_y - tape_height//2,
            tape_x + tape_width//2, tape_y + tape_height//2,
            fill=self.alt_tape_color, outline=self.line_color, tags="pfd"
        )
        
        # Draw altitude range (current altitude +/- 100)
        min_alt = max(0, int(self.altitude) - 100)
        max_alt = int(self.altitude) + 100
        
        # Make min_alt and max_alt round to nearest 20
        min_alt = min_alt - (min_alt % 20)
        max_alt = max_alt + (20 - max_alt % 20)
        
        # Altitude tick marks
        for alt in range(min_alt, max_alt + 1, 20):
            # Calculate vertical position
            y_pos = tape_y + (self.altitude - alt) * (tape_height / 200)
            
            if y_pos < tape_y - tape_height//2 or y_pos > tape_y + tape_height//2:
                continue
                
            # Draw tick mark
            tick_width = 10 if alt % 100 == 0 else 5
            self.canvas.create_line(
                tape_x, y_pos,
                tape_x + tick_width, y_pos,
                fill=self.line_color, tags="pfd"
            )
            
            # Draw altitude label (only for round numbers)
            if alt % 100 == 0:
                self.canvas.create_text(
                    tape_x + 20, y_pos,
                    text=str(alt),
                    fill=self.text_color,
                    font=("Arial", 8),
                    anchor=tk.W,
                    tags="pfd"
                )
        
        # Draw current altitude box
        self.canvas.create_rectangle(
            tape_x - tape_width//2, tape_y - 10,
            tape_x + tape_width//2 + 5, tape_y + 10,
            fill="black", outline=self.line_color, tags="pfd"
        )
        
        # Draw current altitude value
        self.canvas.create_text(
            tape_x + 5, tape_y,
            text=f"{int(self.altitude)}",
            fill=self.text_color,
            font=("Arial", 10, "bold"),
            anchor=tk.W,
            tags="pfd"
        )
        
        # Label
        self.canvas.create_text(
            tape_x, tape_y - tape_height//2 - 10,
            text="ALT",
            fill=self.text_color,
            font=("Arial", 8),
            tags="pfd"
        )
        
    def _draw_heading_indicator(self):
        """Draw heading indicator at the top"""
        # Heading indicator position and dimensions
        hdg_x = self.x + self.width // 2
        hdg_y = self.y + 35
        hdg_width = 100
        hdg_height = 20
        
        # Draw heading background
        self.canvas.create_rectangle(
            hdg_x - hdg_width//2, hdg_y - hdg_height//2,
            hdg_x + hdg_width//2, hdg_y + hdg_height//2,
            fill="#222222", outline=self.line_color, tags="pfd"
        )
        
        # Calculate the visible range of headings
        hdg_range = 60  # degrees visible
        min_hdg = (self.heading - hdg_range//2) % 360
        
        # Draw heading tick marks
        for i in range(hdg_range + 1):
            hdg = (min_hdg + i) % 360
            x_pos = hdg_x - hdg_width//2 + i * (hdg_width / hdg_range)
            
            # Draw tick mark
            if hdg % 10 == 0:
                tick_height = 10 if hdg % 30 == 0 else 5
                self.canvas.create_line(
                    x_pos, hdg_y - tick_height,
                    x_pos, hdg_y,
                    fill=self.line_color, tags="pfd"
                )
                
                # Draw heading label (only for round numbers)
                if hdg % 30 == 0:
                    label = str(hdg // 10)
                    self.canvas.create_text(
                        x_pos, hdg_y - 10,
                        text=label,
                        fill=self.text_color,
                        font=("Arial", 8),
                        tags="pfd"
                    )
        
        # Draw current heading pointer
        self.canvas.create_polygon(
            hdg_x - 7, hdg_y - hdg_height//2,
            hdg_x, hdg_y - hdg_height//2 - 7,
            hdg_x + 7, hdg_y - hdg_height//2,
            fill=self.line_color, tags="pfd"
        )
        
        # Draw current heading value
        self.canvas.create_rectangle(
            hdg_x - 15, hdg_y + hdg_height//2,
            hdg_x + 15, hdg_y + hdg_height//2 + 15,
            fill="black", outline=self.line_color, tags="pfd"
        )
        
        self.canvas.create_text(
            hdg_x, hdg_y + hdg_height//2 + 7,
            text=f"{int(self.heading):03d}",
            fill=self.text_color,
            font=("Arial", 8, "bold"),
            tags="pfd"
        )
        
    def _draw_vertical_speed(self):
        """Draw vertical speed indicator"""
        # VS indicator position and dimensions
        vs_x = self.x + self.width - 15
        vs_y = self.y + self.height // 2
        vs_width = 10
        vs_height = 80
        
        # Draw VS scale background
        self.canvas.create_rectangle(
            vs_x - vs_width//2, vs_y - vs_height//2,
            vs_x + vs_width//2, vs_y + vs_height//2,
            fill="#222222", outline=self.line_color, tags="pfd"
        )
        
        # Draw center line
        self.canvas.create_line(
            vs_x - vs_width//2, vs_y,
            vs_x + vs_width//2, vs_y,
            fill=self.line_color, tags="pfd"
        )
        
        # Draw tick marks
        for rate in [-2, -1, 0, 1, 2]:
            y_pos = vs_y - rate * (vs_height / 4)
            
            # Draw tick
            self.canvas.create_line(
                vs_x - vs_width//2, y_pos,
                vs_x - vs_width//2 + 5, y_pos,
                fill=self.line_color, tags="pfd"
            )
        
        # Draw VS indicator
        vs_indicator_height = self.vertical_speed * (vs_height / 4)
        vs_indicator_height = max(-vs_height/2, min(vs_height/2, vs_indicator_height))
        
        self.canvas.create_rectangle(
            vs_x - vs_width//2, vs_y,
            vs_x, vs_y - vs_indicator_height,
            fill="green" if self.vertical_speed >= 0 else "red",
            outline=self.line_color,
            tags="pfd"
        )
        
        # Draw VS value
        self.canvas.create_text(
            vs_x, vs_y - vs_height//2 - 10,
            text="VS",
            fill=self.text_color,
            font=("Arial", 8),
            tags="pfd"
        )


class DroneAutopilotGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Autopilot Control System")
        self.root.geometry("1200x800")
        self.root.configure(bg="#1e1e1e")
        
        # Set dark theme
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure("TFrame", background="#1e1e1e")
        self.style.configure("TLabel", background="#1e1e1e", foreground="white")
        self.style.configure("TButton", background="#333333", foreground="white")
        self.style.configure("TScale", background="#1e1e1e")
        self.style.configure("TCombobox", fieldbackground="#333333", background="#333333", foreground="white")
        
        # Initialize drone state variables
        self.battery_level = 85.0
        self.altitude = 120.0
        self.speed = 15.0
        self.heading = 45
        self.connected = True
        self.flight_mode = "Manual"
        self.is_flying = False
        self.lat = 37.7749
        self.lon = -122.4194
        self.roll = 0
        self.pitch = 2
        self.rc_signal = 98
        self.data_signal = 100
        self.vertical_speed = 0  # Added vertical speed
        
        # Camera/map view variables
        self.current_view = "map"  # Default to map view
        self.camera_active = False
        self.cap = None  # Will hold the webcam capture object
        
        # Create main frames
        self.create_layout()
        
        # Start simulation thread
        self.simulation_running = True
        self.sim_thread = threading.Thread(target=self.simulate_drone_data)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
        # Schedule UI updates
        self.update_ui()
        
    def create_layout(self):
        # Main container
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Header frame
        header_frame = ttk.Frame(main_container)
        header_frame.pack(fill=tk.X, pady=(0, 10))
        
        ttk.Label(header_frame, text="Drone Autopilot", font=("Arial", 16, "bold")).pack(side=tk.LEFT)
        
        # Connection status
        conn_frame = ttk.Frame(header_frame)
        conn_frame.pack(side=tk.RIGHT)
        
        self.conn_status = ttk.Label(conn_frame, text="Connected" if self.connected else "Disconnected", 
                                    foreground="green" if self.connected else "red")
        self.conn_status.pack(side=tk.RIGHT, padx=(0, 10))
        
        self.battery_indicator = ttk.Label(conn_frame, text=f"Battery: {self.battery_level:.1f}%", 
                                          foreground="green" if self.battery_level > 20 else "red")
        self.battery_indicator.pack(side=tk.RIGHT, padx=(0, 20))
        
        # Content area
        content = ttk.Frame(main_container)
        content.pack(fill=tk.BOTH, expand=True)
        
        # Left sidebar - Controls
        controls_frame = ttk.Frame(content, width=250)
        controls_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        controls_frame.pack_propagate(False)
        
        ttk.Label(controls_frame, text="Flight Controls", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=(0, 10))
        
        # Flight mode selection
        ttk.Label(controls_frame, text="Flight Mode").pack(anchor=tk.W, pady=(10, 5))
        
        self.flight_mode_var = tk.StringVar(value=self.flight_mode)
        flight_mode_options = ["Manual", "Follow Me", "Waypoint", "Return Home", "Orbit"]
        flight_mode_combo = ttk.Combobox(controls_frame, textvariable=self.flight_mode_var, values=flight_mode_options)
        flight_mode_combo.pack(fill=tk.X, pady=(0, 10))
        flight_mode_combo.bind("<<ComboboxSelected>>", self.on_flight_mode_changed)
        
        # Altitude control
        ttk.Label(controls_frame, text=f"Target Altitude (m): {self.altitude:.1f}").pack(anchor=tk.W, pady=(10, 5))
        
        self.altitude_scale = ttk.Scale(controls_frame, from_=0, to=400, value=self.altitude, 
                                       command=self.on_altitude_changed)
        self.altitude_scale.pack(fill=tk.X)
        
        altitude_labels = ttk.Frame(controls_frame)
        altitude_labels.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(altitude_labels, text="0m").pack(side=tk.LEFT)
        self.altitude_value = ttk.Label(altitude_labels, text=f"{self.altitude:.1f}m")
        self.altitude_value.pack(side=tk.LEFT, expand=True)
        ttk.Label(altitude_labels, text="400m").pack(side=tk.RIGHT)
        
        # Speed control
        ttk.Label(controls_frame, text=f"Speed (m/s): {self.speed:.1f}").pack(anchor=tk.W, pady=(10, 5))
        
        self.speed_scale = ttk.Scale(controls_frame, from_=0, to=30, value=self.speed, 
                                    command=self.on_speed_changed)
        self.speed_scale.pack(fill=tk.X)
        
        speed_labels = ttk.Frame(controls_frame)
        speed_labels.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(speed_labels, text="0m/s").pack(side=tk.LEFT)
        self.speed_value = ttk.Label(speed_labels, text=f"{self.speed:.1f}m/s")
        self.speed_value.pack(side=tk.LEFT, expand=True)
        ttk.Label(speed_labels, text="30m/s").pack(side=tk.RIGHT)
        
        # View toggle button
        view_toggle_frame = ttk.Frame(controls_frame)
        view_toggle_frame.pack(fill=tk.X, pady=(20, 0))
        
        self.view_button = ttk.Button(view_toggle_frame, text="Switch to Camera", 
                                     command=self.toggle_view)
        self.view_button.pack(fill=tk.X)
        
        # Control buttons
        button_frame = ttk.Frame(controls_frame)
        button_frame.pack(fill=tk.X, pady=(20, 0))
        
        self.start_button = ttk.Button(button_frame, text="Start" if not self.is_flying else "Stop", 
                                      command=self.toggle_flight)
        self.start_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        self.rth_button = ttk.Button(button_frame, text="Return Home", 
                                    command=self.return_home)
        self.rth_button.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5, 0))
        
        # Center - Map/Camera view
        self.center_frame = ttk.Frame(content)
        self.center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)
        
        # Map view
        self.map_frame = ttk.Frame(self.center_frame)
        self.map_frame.pack(fill=tk.BOTH, expand=True)
        
        self.map_canvas = tk.Canvas(self.map_frame, bg="#333333", highlightthickness=0)
        self.map_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Extend the Canvas class to add save/restore and rotate methods for the PFD
        self._extend_canvas_functions()
        
        # Create the Primary Flight Display
        self.pfd = PrimaryFlightDisplay(
            self.map_canvas, 
            10,  # X position (bottom left corner)
            self.map_canvas.winfo_reqheight() - 10 - 200,  # Y position
            250,  # Width
            200   # Height
        )
        
        # Draw placeholder map elements
        self.map_canvas.create_text(self.map_canvas.winfo_reqwidth() // 2, 
                                   self.map_canvas.winfo_reqheight() // 2,
                                   text="Map View", fill="gray", font=("Arial", 16))
        
        # Camera view (initially hidden)
        self.camera_frame = ttk.Frame(self.center_frame)
        self.camera_label = ttk.Label(self.camera_frame)
        self.camera_label.pack(fill=tk.BOTH, expand=True)
        
        # Compass overlay
        self.compass_size = 100
        self.compass_x = 80
        self.compass_y = 80
        
        # Status bar
        status_bar = ttk.Frame(self.center_frame)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM, pady=(10, 0))
        
        self.status_label = ttk.Label(status_bar, text=f"Status: {'Flying' if self.is_flying else 'Idle'}")
        self.status_label.pack(side=tk.LEFT)
        
        self.flight_info = ttk.Label(status_bar, 
                                    text=f"Mode: {self.flight_mode} | Alt: {self.altitude:.1f}m | Speed: {self.speed:.1f}m/s | Heading: {self.heading}°")
        self.flight_info.pack(side=tk.RIGHT)
        
        # Right sidebar - Telemetry
        telemetry_frame = ttk.Frame(content, width=250)
        telemetry_frame.pack(side=tk.RIGHT, fill=tk.Y)
        telemetry_frame.pack_propagate(False)
        
        ttk.Label(telemetry_frame, text="Telemetry Data", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=(0, 10))
        
        # Position info
        pos_frame = ttk.Frame(telemetry_frame, padding=10)
        pos_frame.pack(fill=tk.X, pady=5)
        pos_frame.configure(style="Card.TFrame")
        self.style.configure("Card.TFrame", background="#333333")
        
        ttk.Label(pos_frame, text="Position", font=("Arial", 10, "bold"), 
                 foreground="#aaaaaa").pack(anchor=tk.W)
        
        pos_grid = ttk.Frame(pos_frame)
        pos_grid.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(pos_grid, text="Lat:").grid(row=0, column=0, sticky=tk.W)
        self.lat_value = ttk.Label(pos_grid, text=f"{self.lat:.4f}°")
        self.lat_value.grid(row=0, column=1, sticky=tk.E)
        
        ttk.Label(pos_grid, text="Lon:").grid(row=1, column=0, sticky=tk.W)
        self.lon_value = ttk.Label(pos_grid, text=f"{self.lon:.4f}°")
        self.lon_value.grid(row=1, column=1, sticky=tk.E)
        
        # Add vertical speed to telemetry
        ttk.Label(pos_grid, text="V.Speed:").grid(row=2, column=0, sticky=tk.W)
        self.vs_value = ttk.Label(pos_grid, text=f"{self.vertical_speed:.1f} m/s")
        self.vs_value.grid(row=2, column=1, sticky=tk.E)
        
        # Orientation info
        ori_frame = ttk.Frame(telemetry_frame, padding=10)
        ori_frame.pack(fill=tk.X, pady=5)
        ori_frame.configure(style="Card.TFrame")
        