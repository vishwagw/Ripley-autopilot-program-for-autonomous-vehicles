# other part of enhanced version:
def _extend_canvas_functions(self):
    """Add save/restore and rotate methods to Canvas for the PFD"""
    # Add a state stack to canvas
    self.map_canvas._state_stack = []
    
    # Add save method to canvas
    def save(canvas):
        # Save current transformation matrix
        canvas._state_stack.append(canvas.gettags("all"))
    
    # Add restore method to canvas
    def restore(canvas):
        if canvas._state_stack:
            # Restore last transformation
            canvas._state_stack.pop()
    
    # Add rotate method to canvas
    def rotate(canvas, cx, cy, angle):
        # Apply rotation transformation around center point
        # For PFD, we'll simulate this by storing the rotation angle
        canvas._rotation_angle = angle
        canvas._rotation_center = (cx, cy)
    
    # Attach methods to canvas
    self.map_canvas.save = lambda: save(self.map_canvas)
    self.map_canvas.restore = lambda: restore(self.map_canvas)
    self.map_canvas.rotate = lambda cx, cy, angle: rotate(self.map_canvas, cx, cy, angle)

def update_ui(self):
    """Update the UI elements with current drone data"""
    # Update battery indicator
    self.battery_indicator.config(text=f"Battery: {self.battery_level:.1f}%", 
                                 foreground="green" if self.battery_level > 20 else "red")
    
    # Update altitude value
    self.altitude_value.config(text=f"{self.altitude:.1f}m")
    
    # Update speed value
    self.speed_value.config(text=f"{self.speed:.1f}m/s")
    
    # Update status label
    self.status_label.config(text=f"Status: {'Flying' if self.is_flying else 'Idle'}")
    
    # Update flight info
    self.flight_info.config(text=f"Mode: {self.flight_mode} | Alt: {self.altitude:.1f}m | " +
                          f"Speed: {self.speed:.1f}m/s | Heading: {self.heading}°")
    
    # Update position info
    self.lat_value.config(text=f"{self.lat:.4f}°")
    self.lon_value.config(text=f"{self.lon:.4f}°")
    self.vs_value.config(text=f"{self.vertical_speed:.1f} m/s")
    
    # Draw the compass on map view if using map view
    if self.current_view == "map":
        self.draw_compass()
        
        # Update and draw the Primary Flight Display
        self.update_pfd()
    
    # Update camera view if active
    if self.camera_active and self.cap is not None:
        self.update_camera()
    
    # Schedule next UI update
    self.root.after(100, self.update_ui)

def update_pfd(self):
    """Update and draw the Primary Flight Display"""
    # Update PFD position based on canvas size
    self.pfd.x = 10
    self.pfd.y = self.map_canvas.winfo_height() - 210
    
    # Update PFD with current drone data
    self.pfd.update(
        roll=self.roll,
        pitch=self.pitch,
        altitude=self.altitude,
        speed=self.speed,
        heading=self.heading,
        vertical_speed=self.vertical_speed
    )
    
    # Draw the PFD
    self.pfd.draw()

def simulate_drone_data(self):
    """Simulate changing drone data for demonstration"""
    while self.simulation_running:
        # Simulate battery drain
        if self.is_flying:
            self.battery_level -= 0.01
            if self.battery_level < 0:
                self.battery_level = 0
                self.is_flying = False
        
        # Simulate movement if flying
        if self.is_flying:
            # Random altitude changes
            altitude_change = random.uniform(-0.5, 0.5)
            self.altitude += altitude_change
            self.altitude = max(0, min(400, self.altitude))
            
            # Set vertical speed based on altitude change
            self.vertical_speed = altitude_change * 2
            
            # Random heading changes
            heading_change = random.uniform(-1, 1)
            self.heading = (self.heading + heading_change) % 360
            
            # Random pitch and roll changes
            self.pitch += random.uniform(-0.2, 0.2)
            self.pitch = max(-20, min(20, self.pitch))
            
            self.roll += random.uniform(-0.3, 0.3)
            self.roll = max(-30, min(30, self.roll))
            
            # Random position changes
            self.lat += random.uniform(-0.0001, 0.0001)
            self.lon += random.uniform(-0.0001, 0.0001)
        else:
            # Reset values when not flying
            self.vertical_speed = 0
            self.pitch = 0
            self.roll = 0
        
        # Simulate signal strength fluctuations
        self.rc_signal = min(100, max(0, self.rc_signal + random.uniform(-1, 1)))
        self.data_signal = min(100, max(0, self.data_signal + random.uniform(-0.5, 0.5)))
        
        # Sleep to reduce CPU usage
        time.sleep(0.1)

def toggle_view(self):
    """Toggle between map and camera view"""
    if self.current_view == "map":
        self.current_view = "camera"
        self.view_button.config(text="Switch to Map")
        
        # Hide map view, show camera view
        self.map_frame.pack_forget()
        self.camera_frame.pack(fill=tk.BOTH, expand=True)
        
        # Activate camera if not already active
        if not self.camera_active:
            self.camera_active = True
            try:
                self.cap = cv2.VideoCapture(0)  # Try to open default camera
            except:
                self.camera_label.config(text="Camera not available")
    else:
        self.current_view = "map"
        self.view_button.config(text="Switch to Camera")
        
        # Hide camera view, show map view
        self.camera_frame.pack_forget()
        self.map_frame.pack(fill=tk.BOTH, expand=True)

def update_camera(self):
    """Update the camera feed"""
    ret, frame = self.cap.read()
    if ret:
        # Convert from BGR to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Convert to PIL format and then to Tkinter image
        img = Image.fromarray(frame)
        
        # Resize to fit the frame
        img = img.resize((self.camera_frame.winfo_width(), self.camera_frame.winfo_height()), 
                        Image.LANCZOS)
        
        img_tk = ImageTk.PhotoImage(image=img)
        
        # Keep a reference, otherwise it gets garbage collected
        self.camera_label.img_tk = img_tk
        self.camera_label.config(image=img_tk)
    else:
        self.camera_label.config(text="Camera feed error")