# version 1.2: 
# added camera/map toggle feature
import tkinter as tk
from tkinter import ttk
import math
import time
import threading
import random
import cv2
from PIL import Image, ImageTk

# building the user interface 
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
        
        ttk.Label(header_frame, text="Ripley UAV Autopilot", font=("Arial", 16, "bold")).pack(side=tk.LEFT)
        
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
        
        ttk.Label(telemetry_frame, text="Telemetry", font=("Arial", 12, "bold")).pack(anchor=tk.W, pady=(0, 10))
        
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
        
        # Orientation info
        ori_frame = ttk.Frame(telemetry_frame, padding=10)
        ori_frame.pack(fill=tk.X, pady=5)
        ori_frame.configure(style="Card.TFrame")
        
        ttk.Label(ori_frame, text="Orientation", font=("Arial", 10, "bold"), 
                 foreground="#aaaaaa").pack(anchor=tk.W)
        
        ori_grid = ttk.Frame(ori_frame)
        ori_grid.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(ori_grid, text="Roll:").grid(row=0, column=0, sticky=tk.W)
        self.roll_value = ttk.Label(ori_grid, text=f"{self.roll}°")
        self.roll_value.grid(row=0, column=1, sticky=tk.E)
        
        ttk.Label(ori_grid, text="Pitch:").grid(row=1, column=0, sticky=tk.W)
        self.pitch_value = ttk.Label(ori_grid, text=f"{self.pitch}°")
        self.pitch_value.grid(row=1, column=1, sticky=tk.E)
        
        ttk.Label(ori_grid, text="Yaw:").grid(row=2, column=0, sticky=tk.W)
        self.yaw_value = ttk.Label(ori_grid, text=f"{self.heading}°")
        self.yaw_value.grid(row=2, column=1, sticky=tk.E)
        
        # Signal info
        signal_frame = ttk.Frame(telemetry_frame, padding=10)
        signal_frame.pack(fill=tk.X, pady=5)
        signal_frame.configure(style="Card.TFrame")
        
        ttk.Label(signal_frame, text="Signal", font=("Arial", 10, "bold"), 
                 foreground="#aaaaaa").pack(anchor=tk.W)
        
        signal_grid = ttk.Frame(signal_frame)
        signal_grid.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(signal_grid, text="RC Link:").grid(row=0, column=0, sticky=tk.W)
        self.rc_value = ttk.Label(signal_grid, text=f"{self.rc_signal}%", foreground="green")
        self.rc_value.grid(row=0, column=1, sticky=tk.E)
        
        ttk.Label(signal_grid, text="Data Link:").grid(row=1, column=0, sticky=tk.W)
        self.data_value = ttk.Label(signal_grid, text=f"{self.data_signal}%", foreground="green")
        self.data_value.grid(row=1, column=1, sticky=tk.E)
    
    def draw_compass(self):
        # Clear previous compass
        self.map_canvas.delete("compass")
        
        # Draw compass circle
        self.map_canvas.create_oval(
            self.compass_x - self.compass_size//2,
            self.compass_y - self.compass_size//2,
            self.compass_x + self.compass_size//2,
            self.compass_y + self.compass_size//2,
            outline="#4a86e8", fill="#333333", width=2, tags="compass"
        )
        
        # Draw heading indicator
        angle_rad = math.radians(self.heading)
        indicator_length = self.compass_size//2 - 10
        end_x = self.compass_x + indicator_length * math.sin(angle_rad)
        end_y = self.compass_y - indicator_length * math.cos(angle_rad)
        
        self.map_canvas.create_line(
            self.compass_x, self.compass_y, end_x, end_y,
            fill="red", width=2, tags="compass"
        )
        
        # Draw heading text
        self.map_canvas.create_text(
            self.compass_x, self.compass_y - self.compass_size//2 - 15,
            text=f"{self.heading}°", fill="white", tags="compass"
        )
        
        # Draw cardinal directions
        for i, direction in enumerate(["N", "E", "S", "W"]):
            angle = math.radians(i * 90)
            text_x = self.compass_x + (self.compass_size//2 - 5) * math.sin(angle)
            text_y = self.compass_y - (self.compass_size//2 - 5) * math.cos(angle)
            self.map_canvas.create_text(text_x, text_y, text=direction, fill="white", tags="compass")
    
    def toggle_flight(self):
        self.is_flying = not self.is_flying
        button_text = "Stop" if self.is_flying else "Start"
        self.start_button.configure(text=button_text)
    
    def return_home(self):
        self.flight_mode = "Return Home"
        self.flight_mode_var.set("Return Home")
    
    def on_flight_mode_changed(self, event):
        self.flight_mode = self.flight_mode_var.get()
    
    def on_altitude_changed(self, value):
        self.altitude = float(value)
        self.altitude_value.configure(text=f"{self.altitude:.1f}m")
    
    def on_speed_changed(self, value):
        self.speed = float(value)
        self.speed_value.configure(text=f"{self.speed:.1f}m/s")
    
    def toggle_view(self):
        """Toggle between map view and camera view"""
        if self.current_view == "map":
            # Switch to camera view
            self.current_view = "camera"
            self.view_button.configure(text="Switch to Map")
            self.map_frame.pack_forget()
            self.camera_frame.pack(fill=tk.BOTH, expand=True)
            self.start_camera()
        else:
            # Switch to map view
            self.current_view = "map"
            self.view_button.configure(text="Switch to Camera")
            self.camera_frame.pack_forget()
            self.map_frame.pack(fill=tk.BOTH, expand=True)
            self.stop_camera()
    
    def start_camera(self):
        """Start webcam capture"""
        if not self.camera_active:
            self.camera_active = True
            self.cap = cv2.VideoCapture(0)  # 0 is usually the default webcam
            if not self.cap.isOpened():
                # If webcam can't be opened, show an error message
                self.camera_label.configure(text="Error: Could not open webcam")
                return
            
            # Start updating camera feed
            self.update_camera()
    
    def update_camera(self):
        """Update camera feed"""
        if self.camera_active and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert frame from BGR to RGB color space
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Add compass overlay to camera view
                self.add_camera_overlay(frame)
                
                # Convert to PhotoImage for Tkinter
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)
                
                # Update the label
                self.camera_label.imgtk = imgtk
                self.camera_label.configure(image=imgtk)
                
            # Schedule the next update
            if self.camera_active:
                self.root.after(30, self.update_camera)
    
    def add_camera_overlay(self, frame):
        """Add compass overlay and telemetry data to camera frame"""
        height, width = frame.shape[:2]
        
        # Add compass
        center_x = 80
        center_y = 80
        radius = 40
        
        # Draw compass circle
        cv2.circle(frame, (center_x, center_y), radius, (74, 134, 232), 2)
        
        # Draw heading indicator
        angle_rad = math.radians(self.heading)
        end_x = int(center_x + (radius - 10) * math.sin(angle_rad))
        end_y = int(center_y - (radius - 10) * math.cos(angle_rad))
        cv2.line(frame, (center_x, center_y), (end_x, end_y), (0, 0, 255), 2)
        
        # Draw cardinal directions
        directions = ["N", "E", "S", "W"]
        for i, direction in enumerate(directions):
            angle = math.radians(i * 90)
            text_x = int(center_x + (radius - 5) * math.sin(angle))
            text_y = int(center_y - (radius - 5) * math.cos(angle))
            cv2.putText(frame, direction, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw telemetry data overlay
        cv2.putText(frame, f"Alt: {self.altitude:.1f}m", (width - 150, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Spd: {self.speed:.1f}m/s", (width - 150, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Hdg: {self.heading}°", (width - 150, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Add battery indicator
        battery_color = (0, 255, 0) if self.battery_level > 20 else (0, 0, 255)
        cv2.putText(frame, f"Batt: {self.battery_level:.1f}%", (width - 150, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, battery_color, 1)
    
    def stop_camera(self):
        """Stop webcam capture"""
        self.camera_active = False
        if self.cap is not None:
            self.cap.release()
            self.cap = None
    
    def simulate_drone_data(self):
        """Simulate drone data changes in a separate thread"""
        while self.simulation_running:
            if self.is_flying:
                # Update battery
                self.battery_level = max(0, self.battery_level - 0.05)
                
                # Update heading
                self.heading = (self.heading + 1) % 360
                
                # Update altitude with some randomness
                self.altitude += (random.random() - 0.5) * 2
                self.altitude = max(0, min(400, self.altitude))
                
                # Update speed with some randomness
                self.speed += (random.random() - 0.5)
                self.speed = max(0, min(30, self.speed))
                
                # Update orientation
                self.roll = (self.roll + (random.random() - 0.5)) % 5
                self.pitch = (self.pitch + (random.random() - 0.5)) % 5
                
                # Small position changes
                self.lat += (random.random() - 0.5) * 0.0001
                self.lon += (random.random() - 0.5) * 0.0001
            
            time.sleep(0.5)
    
    def update_ui(self):
        """Update UI with current drone state"""
        # Update battery indicator
        battery_color = "green" if self.battery_level > 20 else "red"
        self.battery_indicator.configure(text=f"Battery: {self.battery_level:.1f}%", foreground=battery_color)
        
        # Update status
        self.status_label.configure(text=f"Status: {'Flying' if self.is_flying else 'Idle'}")
        
        # Update flight info
        self.flight_info.configure(
            text=f"Mode: {self.flight_mode} | Alt: {self.altitude:.1f}m | Speed: {self.speed:.1f}m/s | Heading: {self.heading}°"
        )
        
        # Update telemetry values
        self.lat_value.configure(text=f"{self.lat:.4f}°")
        self.lon_value.configure(text=f"{self.lon:.4f}°")
        self.roll_value.configure(text=f"{self.roll:.1f}°")
        self.pitch_value.configure(text=f"{self.pitch:.1f}°")
        self.yaw_value.configure(text=f"{self.heading}°")
        
        # Update scales if changed externally
        self.altitude_scale.set(self.altitude)
        self.speed_scale.set(self.speed)
        
        # Draw compass if in map view
        if self.current_view == "map":
            self.draw_compass()
        
        # Schedule next update
        self.root.after(100, self.update_ui)
    
    def on_closing(self):
        """Clean up resources before closing"""
        self.simulation_running = False
        self.stop_camera()
        if self.sim_thread.is_alive():
            self.sim_thread.join(timeout=1.0)
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = DroneAutopilotGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
