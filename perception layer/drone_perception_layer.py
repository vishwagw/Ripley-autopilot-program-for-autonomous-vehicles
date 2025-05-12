"""
Drone Perception Layer - Autonomous Control System
-------------------------------------------------
This module implements the perception layer for an autonomous drone control system.
It processes various sensor inputs to create a comprehensive environmental model,
enabling safe and efficient autonomous flight.

Key components:
1. Sensor data acquisition and preprocessing
2. Object detection and tracking
3. Environmental mapping and obstacle detection
4. Sensor fusion for robust state estimation
5. Situation awareness module
"""

import numpy as np
import cv2
import time
from dataclasses import dataclass
from typing import List, Dict, Tuple, Optional
from enum import Enum


# ===== DATA STRUCTURES =====

class ObjectClass(Enum):
    """Classes of objects that can be detected."""
    UNKNOWN = 0
    PERSON = 1
    VEHICLE = 2
    BUILDING = 3
    TREE = 4
    POWERLINE = 5
    ANIMAL = 6
    DRONE = 7
    BIRD = 8


@dataclass
class BoundingBox:
    """Represents a 3D bounding box in space."""
    x_min: float
    y_min: float
    z_min: float
    x_max: float
    y_max: float
    z_max: float
    
    @property
    def center(self) -> Tuple[float, float, float]:
        """Return the center point of the bounding box."""
        return (
            (self.x_min + self.x_max) / 2,
            (self.y_min + self.y_max) / 2,
            (self.z_min + self.z_max) / 2
        )
    
    @property
    def dimensions(self) -> Tuple[float, float, float]:
        """Return the dimensions of the bounding box (width, height, depth)."""
        return (
            self.x_max - self.x_min,
            self.y_max - self.y_min,
            self.z_max - self.z_min
        )


@dataclass
class DetectedObject:
    """Represents a detected object in the environment."""
    object_id: int
    object_class: ObjectClass
    confidence: float
    bbox: BoundingBox
    velocity: Optional[Tuple[float, float, float]] = None
    timestamp: float = 0.0


@dataclass
class Obstacle:
    """Represents an obstacle in the environment."""
    position: Tuple[float, float, float]
    dimensions: Tuple[float, float, float]
    is_static: bool
    velocity: Optional[Tuple[float, float, float]] = None
    type: str = "unknown"


@dataclass
class DroneState:
    """Represents the current state of the drone."""
    position: Tuple[float, float, float]  # x, y, z in meters
    orientation: Tuple[float, float, float, float]  # quaternion
    velocity: Tuple[float, float, float]  # m/s
    angular_velocity: Tuple[float, float, float]  # rad/s
    acceleration: Tuple[float, float, float]  # m/s^2
    timestamp: float  # in seconds


class SensorType(Enum):
    """Types of sensors that can be mounted on the drone."""
    CAMERA = 1
    LIDAR = 2
    RADAR = 3
    ULTRASONIC = 4
    IMU = 5
    GPS = 6
    BAROMETER = 7
    MAGNETOMETER = 8


@dataclass
class SensorConfig:
    """Configuration for a single sensor."""
    sensor_id: int
    sensor_type: SensorType
    position: Tuple[float, float, float]  # relative to drone center
    orientation: Tuple[float, float, float]  # Euler angles in radians
    field_of_view: Optional[float] = None  # in radians
    range: Optional[float] = None  # in meters
    resolution: Optional[Tuple[int, int]] = None  # for cameras


# ===== SENSOR DATA ACQUISITION =====

class SensorDataAcquisition:
    """Handles acquisition and preprocessing of sensor data."""
    
    def __init__(self, sensor_configs: List[SensorConfig]):
        """Initialize with a list of sensor configurations."""
        self.sensor_configs = sensor_configs
        self.sensor_handlers = {}
        self._init_sensor_handlers()
        
    def _init_sensor_handlers(self):
        """Initialize handlers for each type of sensor."""
        for config in self.sensor_configs:
            if config.sensor_type == SensorType.CAMERA:
                self.sensor_handlers[config.sensor_id] = CameraHandler(config)
            elif config.sensor_type == SensorType.LIDAR:
                self.sensor_handlers[config.sensor_id] = LidarHandler(config)
            elif config.sensor_type == SensorType.RADAR:
                self.sensor_handlers[config.sensor_id] = RadarHandler(config)
            elif config.sensor_type == SensorType.ULTRASONIC:
                self.sensor_handlers[config.sensor_id] = UltrasonicHandler(config)
            elif config.sensor_type == SensorType.IMU:
                self.sensor_handlers[config.sensor_id] = IMUHandler(config)
            elif config.sensor_type == SensorType.GPS:
                self.sensor_handlers[config.sensor_id] = GPSHandler(config)
            # Add more handlers as needed
    
    def get_sensor_data(self):
        """Collect and preprocess data from all sensors."""
        sensor_data = {}
        current_time = time.time()
        
        for sensor_id, handler in self.sensor_handlers.items():
            try:
                data = handler.get_data()
                if data is not None:
                    sensor_data[sensor_id] = {
                        'data': data,
                        'timestamp': current_time,
                        'config': self.sensor_configs[sensor_id]
                    }
            except Exception as e:
                print(f"Error reading sensor {sensor_id}: {e}")
                
        return sensor_data


class CameraHandler:
    """Handles camera sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        self.camera = None
        # Initialize camera connection
        # self.camera = cv2.VideoCapture(0)  # This would be replaced with actual camera initialization
        
    def get_data(self):
        """Read and preprocess camera frame."""
        if self.camera is None:
            # Simulate camera data for development
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            return self._preprocess_frame(frame)
        
        # In a real implementation:
        # ret, frame = self.camera.read()
        # if ret:
        #     return self._preprocess_frame(frame)
        # return None
        
    def _preprocess_frame(self, frame):
        """Preprocess camera frame for further analysis."""
        # Resize if needed
        if self.config.resolution is not None:
            frame = cv2.resize(frame, self.config.resolution)
        
        # Apply any necessary preprocessing
        # - Distortion correction
        # - Color normalization
        # - Noise reduction
        
        return frame


class LidarHandler:
    """Handles LiDAR sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        # Initialize LiDAR connection
        
    def get_data(self):
        """Read and preprocess LiDAR data."""
        # Simulate LiDAR point cloud for development
        # Generate random 3D points within sensor range
        num_points = 1000
        point_cloud = np.random.uniform(
            low=[-self.config.range, -self.config.range, 0],
            high=[self.config.range, self.config.range, self.config.range],
            size=(num_points, 3)
        )
        
        # Add intensity values (simulated)
        intensities = np.random.uniform(0, 1, size=(num_points, 1))
        point_cloud_with_intensity = np.hstack((point_cloud, intensities))
        
        return self._preprocess_point_cloud(point_cloud_with_intensity)
        
    def _preprocess_point_cloud(self, point_cloud):
        """Preprocess LiDAR point cloud."""
        # Ground plane removal
        # Noise filtering
        # Downsampling if needed
        return point_cloud


class RadarHandler:
    """Handles radar sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        # Initialize radar connection
        
    def get_data(self):
        """Read and preprocess radar data."""
        # Simulate radar data for development
        num_targets = 5
        # Each target has range, azimuth, elevation, and radial velocity
        targets = np.random.uniform(
            low=[0, -np.pi/2, -np.pi/4, -10],
            high=[self.config.range, np.pi/2, np.pi/4, 10],
            size=(num_targets, 4)
        )
        
        return self._preprocess_radar_data(targets)
        
    def _preprocess_radar_data(self, radar_data):
        """Preprocess radar data."""
        # Filter out noise
        # Cluster reflections
        return radar_data


class UltrasonicHandler:
    """Handles ultrasonic sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        # Initialize ultrasonic sensor connection
        
    def get_data(self):
        """Read ultrasonic sensor data."""
        # Simulate ultrasonic data - distance in meters
        distance = np.random.uniform(0.1, self.config.range)
        return distance


class IMUHandler:
    """Handles IMU sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        # Initialize IMU connection
        
    def get_data(self):
        """Read IMU data."""
        # Simulate IMU data
        # Linear accelerations (m/s^2)
        accel = np.random.normal(0, 0.2, 3)
        # Angular velocities (rad/s)
        gyro = np.random.normal(0, 0.05, 3)
        # Magnetic field (normalized)
        mag = np.random.normal(0, 1, 3)
        mag = mag / np.linalg.norm(mag)
        
        return {
            'acceleration': accel,
            'angular_velocity': gyro,
            'magnetic_field': mag
        }


class GPSHandler:
    """Handles GPS sensor data acquisition and preprocessing."""
    
    def __init__(self, config: SensorConfig):
        self.config = config
        # Initialize GPS connection
        
    def get_data(self):
        """Read GPS data."""
        # Simulate GPS data (latitude, longitude, altitude, accuracy)
        return {
            'latitude': 37.7749 + np.random.normal(0, 0.0001),
            'longitude': -122.4194 + np.random.normal(0, 0.0001),
            'altitude': 100 + np.random.normal(0, 1),
            'accuracy': 2.5 + np.random.uniform(0, 1)
        }


# ===== OBJECT DETECTION AND TRACKING =====

class ObjectDetector:
    """Detects objects from camera images using computer vision."""
    
    def __init__(self, model_path: str = None):
        """Initialize with optional path to a pre-trained model."""
        self.model = None
        if model_path:
            self._load_model(model_path)
            
    def _load_model(self, model_path: str):
        """Load a pre-trained object detection model."""
        # In a real implementation, this would load a model using frameworks like
        # TensorFlow, PyTorch, or OpenCV's DNN module
        print(f"Loading object detection model from {model_path}")
        # Placeholder for model loading
        
    def detect(self, image):
        """Detect objects in the given image."""
        # For demonstration, we'll generate some synthetic detections
        height, width = image.shape[:2]
        num_objects = np.random.randint(1, 5)
        
        detections = []
        for i in range(num_objects):
            # Create a random bounding box
            x1 = np.random.randint(0, width // 2)
            y1 = np.random.randint(0, height // 2)
            x2 = np.random.randint(x1 + width // 4, width)
            y2 = np.random.randint(y1 + height // 4, height)
            
            # Assign a random class
            class_id = np.random.randint(1, len(ObjectClass))
            confidence = np.random.uniform(0.7, 1.0)
            
            # Create 3D bounding box (extending the 2D box)
            z1 = np.random.uniform(5, 10)  # Distance from camera
            z2 = z1 + np.random.uniform(1, 5)  # Depth of object
            
            bbox = BoundingBox(
                x_min=float(x1), y_min=float(y1), z_min=float(z1),
                x_max=float(x2), y_max=float(y2), z_max=float(z2)
            )
            
            detections.append(DetectedObject(
                object_id=i,
                object_class=ObjectClass(class_id),
                confidence=confidence,
                bbox=bbox,
                timestamp=time.time()
            ))
        
        return detections


class ObjectTracker:
    """Tracks detected objects across frames to maintain object persistence."""
    
    def __init__(self, max_age: int = 10, min_hits: int = 3):
        """Initialize object tracker with parameters.
        
        Args:
            max_age: Maximum number of frames an object can be missing before track is deleted
            min_hits: Minimum number of detections needed to establish a track
        """
        self.max_age = max_age
        self.min_hits = min_hits
        self.tracks = {}  # Map of track_id to track data
        self.next_id = 0
        
    def update(self, detections: List[DetectedObject]):
        """Update tracks with new detections."""
        # Predict new locations of existing tracks
        self._predict()
        
        # Match detections to existing tracks
        matched_tracks, unmatched_detections, unmatched_tracks = self._match_detections(detections)
        
        # Update matched tracks
        for track_id, detection_idx in matched_tracks:
            self._update_track(track_id, detections[detection_idx])
        
        # Create new tracks for unmatched detections
        for detection_idx in unmatched_detections:
            self._create_track(detections[detection_idx])
        
        # Mark unmatched tracks as missing
        for track_id in unmatched_tracks:
            self.tracks[track_id]['age'] += 1
        
        # Remove old tracks
        self._remove_old_tracks()
        
        return self.get_active_tracks()
    
    def _predict(self):
        """Predict new locations of all tracks based on their motion model."""
        for track_id, track in self.tracks.items():
            if track['velocity'] is not None:
                # Apply simple linear motion model
                obj = track['object']
                dt = time.time() - obj.timestamp
                
                # Update position based on velocity
                vx, vy, vz = track['velocity']
                center_x, center_y, center_z = obj.bbox.center
                
                new_center_x = center_x + vx * dt
                new_center_y = center_y + vy * dt
                new_center_z = center_z + vz * dt
                
                # Get current dimensions
                width, height, depth = obj.bbox.dimensions
                
                # Create new bounding box
                new_bbox = BoundingBox(
                    x_min=new_center_x - width/2,
                    y_min=new_center_y - height/2,
                    z_min=new_center_z - depth/2,
                    x_max=new_center_x + width/2,
                    y_max=new_center_y + height/2,
                    z_max=new_center_z + depth/2
                )
                
                # Update the track's predicted position
                obj.bbox = new_bbox
                obj.timestamp = time.time()
    
    def _match_detections(self, detections):
        """Match current detections to existing tracks using IoU."""
        if not self.tracks or not detections:
            return [], list(range(len(detections))), list(self.tracks.keys())
        
        # Compute IoU matrix
        iou_matrix = np.zeros((len(self.tracks), len(detections)))
        for i, (track_id, track) in enumerate(self.tracks.items()):
            for j, detection in enumerate(detections):
                iou_matrix[i, j] = self._calculate_iou(track['object'].bbox, detection.bbox)
        
        # Hungarian algorithm to find optimal matching
        # In a real implementation, use scipy.optimize.linear_sum_assignment
        # For simplicity, we'll use a greedy approach here
        matched_tracks = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(self.tracks.keys())
        
        # Find matches above threshold
        iou_threshold = 0.3
        while True:
            # Find highest IoU
            if iou_matrix.size == 0:
                break
                
            i, j = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
            if iou_matrix[i, j] < iou_threshold:
                break
                
            track_id = list(self.tracks.keys())[i]
            matched_tracks.append((track_id, j))
            unmatched_detections.remove(j)
            unmatched_tracks.remove(track_id)
            
            # Remove the matched track and detection from consideration
            iou_matrix = np.delete(iou_matrix, i, axis=0)
            iou_matrix = np.delete(iou_matrix, j, axis=1)
        
        return matched_tracks, unmatched_detections, unmatched_tracks
    
    def _calculate_iou(self, bbox1, bbox2):
        """Calculate 3D IoU between two bounding boxes."""
        # Calculate intersection volume
        x_left = max(bbox1.x_min, bbox2.x_min)
        y_top = max(bbox1.y_min, bbox2.y_min)
        z_near = max(bbox1.z_min, bbox2.z_min)
        
        x_right = min(bbox1.x_max, bbox2.x_max)
        y_bottom = min(bbox1.y_max, bbox2.y_max)
        z_far = min(bbox1.z_max, bbox2.z_max)
        
        # Check if there is an intersection
        if x_right < x_left or y_bottom < y_top or z_far < z_near:
            return 0.0
        
        intersection_volume = (x_right - x_left) * (y_bottom - y_top) * (z_far - z_near)
        
        # Calculate the volumes of both bounding boxes
        bbox1_volume = (bbox1.x_max - bbox1.x_min) * (bbox1.y_max - bbox1.y_min) * (bbox1.z_max - bbox1.z_min)
        bbox2_volume = (bbox2.x_max - bbox2.x_min) * (bbox2.y_max - bbox2.y_min) * (bbox2.z_max - bbox2.z_min)
        
        # Calculate IoU
        union_volume = bbox1_volume + bbox2_volume - intersection_volume
        iou = intersection_volume / union_volume if union_volume > 0 else 0.0
        
        return iou
    
    def _create_track(self, detection):
        """Create a new track from a detection."""
        self.tracks[self.next_id] = {
            'object': detection,
            'hits': 1,
            'age': 0,
            'velocity': None,
            'active': False  # Track becomes active after min_hits
        }
        self.next_id += 1
    
    def _update_track(self, track_id, detection):
        """Update an existing track with new detection."""
        track = self.tracks[track_id]
        
        # Calculate velocity based on previous position
        old_center = track['object'].bbox.center
        new_center = detection.bbox.center
        dt = detection.timestamp - track['object'].timestamp
        
        if dt > 0:
            velocity = (
                (new_center[0] - old_center[0]) / dt,
                (new_center[1] - old_center[1]) / dt,
                (new_center[2] - old_center[2]) / dt
            )
        else:
            velocity = track['velocity']  # Keep previous velocity
        
        # Update detection with track ID and velocity
        detection.object_id = track_id
        detection.velocity = velocity
        
        # Update track data
        track['object'] = detection
        track['hits'] += 1
        track['age'] = 0
        track['velocity'] = velocity
        
        # Mark as active if it has enough hits
        if track['hits'] >= self.min_hits:
            track['active'] = True
    
    def _remove_old_tracks(self):
        """Remove tracks that have been missing for too long."""
        track_ids_to_remove = []
        for track_id, track in self.tracks.items():
            if track['age'] > self.max_age:
                track_ids_to_remove.append(track_id)
                
        for track_id in track_ids_to_remove:
            del self.tracks[track_id]
    
    def get_active_tracks(self):
        """Return currently active tracks."""
        active_tracks = []
        for track_id, track in self.tracks.items():
            if track['active']:
                active_tracks.append(track['object'])
        
        return active_tracks


# ===== ENVIRONMENTAL MAPPING =====

class OccupancyGrid:
    """Represents a 3D occupancy grid for mapping the environment."""
    
    def __init__(self, resolution: float = 0.2, size: Tuple[float, float, float] = (50, 50, 10)):
        """Initialize the occupancy grid.
        
        Args:
            resolution: Cell size in meters
            size: Size of the grid in meters (x, y, z)
        """
        self.resolution = resolution
        self.size = size
        
        # Calculate grid dimensions
        self.grid_dims = (
            int(size[0] / resolution),
            int(size[1] / resolution),
            int(size[2] / resolution)
        )
        
        # Initialize grid with unknown state (-1)
        # 0: Free space, 1: Occupied, -1: Unknown
        self.grid = np.full(self.grid_dims, -1, dtype=np.float32)
        
        # Grid origin (center of the grid in world coordinates)
        self.origin = (size[0]/2, size[1]/2, size[2]/2)
    
    def update_from_point_cloud(self, point_cloud, sensor_position):
        """Update the occupancy grid using a LiDAR point cloud."""
        # Convert sensor position to grid coordinates
        sensor_grid_pos = self.world_to_grid(sensor_position)
        
        for point in point_cloud:
            # Convert point to grid coordinates
            point_grid_pos = self.world_to_grid(point[:3])
            
            # Skip if point is outside grid
            if not self.is_in_grid(point_grid_pos):
                continue
            
            # Mark the cell as occupied
            self.grid[point_grid_pos] = 1.0
            
            # Perform ray tracing from sensor to point to mark free space
            self._ray_trace(sensor_grid_pos, point_grid_pos)
    
    def world_to_grid(self, world_pos):
        """Convert world coordinates to grid indices."""
        # Shift by grid origin and divide by resolution
        grid_x = int((world_pos[0] + self.origin[0]) / self.resolution)
        grid_y = int((world_pos[1] + self.origin[1]) / self.resolution)
        grid_z = int((world_pos[2] + self.origin[2]) / self.resolution)
        
        return (grid_x, grid_y, grid_z)
    
    def grid_to_world(self, grid_pos):
        """Convert grid indices to world coordinates."""
        # Multiply by resolution and shift by grid origin
        world_x = grid_pos[0] * self.resolution - self.origin[0]
        world_y = grid_pos[1] * self.resolution - self.origin[1]
        world_z = grid_pos[2] * self.resolution - self.origin[2]
        
        return (world_x, world_y, world_z)
    
    def is_in_grid(self, grid_pos):
        """Check if the given grid indices are within bounds."""
        x, y, z = grid_pos
        return (0 <= x < self.grid_dims[0] and 
                0 <= y < self.grid_dims[1] and 
                0 <= z < self.grid_dims[2])
    
    def _ray_trace(self, start, end):
        """Perform ray tracing to mark free space between start and end."""
        # Bresenham's 3D line algorithm
        x0, y0, z0 = start
        x1, y1, z1 = end
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        dz = abs(z1 - z0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        sz = 1 if z0 < z1 else -1
        
        if dx >= dy and dx >= dz:
            err_y = dx / 2
            err_z = dx / 2
            
            x, y, z = x0, y0, z0
            while x != x1:
                if self.is_in_grid((x, y, z)):
                    # Don't overwrite occupied cells
                    if self.grid[x, y, z] != 1.0:
                        self.grid[x, y, z] = 0.0  # Mark as free space
                
                err_y -= dy
                if err_y < 0:
                    y += sy
                    err_y += dx
                
                err_z -= dz
                if err_z < 0:
                    z += sz
                    err_z += dx
                
                x += sx
        
        elif dy >= dx and dy >= dz:
            err_x = dy / 2
            err_z = dy / 2
            
            x, y, z = x0, y0, z0
            while y != y1:
                if self.is_in_grid((x, y, z)):
                    if self.grid[x, y, z] != 1.0:
                        self.grid[x, y, z] = 0.0
                
                err_x -= dx
                if err_x < 0:
                    x += sx
                    err_x += dy
                
                err_z -= dz
                if err_z < 0:
                    z += sz
                    err_z += dy
                
                y += sy
        
        else:  # dz >= dx and dz >= dy
            err_x = dz / 2
            err_y = dz / 2
            
            x, y, z = x0, y0, z0
            while z != z1:
                if self.is_in_grid((x, y, z)):
                    if self.grid[x, y, z] != 1.0:
                        self.grid[x, y, z] = 0.0
                
                err_x -= dx
                if err_x < 0:
                    x += sx
                    err_x += dz
                
                err_y -= dy
                if err_y < 0:
                    y += sy
                    err_y += dz
                
                z += sz
    
    def get_obstacles(self, threshold: float = 0.5):
        """Extract obstacles from the occupancy grid."""
        obstacles = []
        
        # Find occupied cells
        occupied = np.where(self.grid > threshold)
        
        # Simple clustering by connectivity
        visited = np.zeros_like(self.grid, dtype=bool)
        
        for i in range(len(occupied[0])):
            x, y, z = occupied[0][i], occupied[1][i], occupied[2][i]
            
            if visited[x, y, z]:
                continue
            
            # Perform BFS to find connected cells
            cluster = []
            queue = [(x, y, z)]
            visited[x, y, z] = True
            
            while queue:
                cx, cy, cz = queue.pop(0)
                cluster.append((cx, cy, cz))
                
                # Check 6-connectivity
                neighbors = [
                    (cx+1, cy, cz), (cx-1, cy, cz),
                    (cx, cy+1, cz), (cx, cy-1, cz),
                    (cx, cy, cz+1), (cx, cy, cz-1)
                ]
                
                for nx, ny, nz in neighbors:
                    if (self.is_in_grid((nx, ny, nz)) and 
                        not visited[nx, ny, nz] and 
                        self.grid[nx, ny, nz] > threshold):
                        visited[nx, ny, nz] = True
                        queue.append((nx, ny, nz))
            
            # Convert cluster to obstacle
            if cluster:
                # Calculate bounding box of the cluster
                cluster_array = np.array(cluster)
                min_x, min_y, min_z = np.min(cluster_array, axis=0)
                max_x, max_y, max_z = np.max(cluster_array, axis=0)
                
                # Convert to world coordinates
                min_world = self.grid_to_world((min_x, min_y, min_z))
                max_world = self.grid_to_world((max_x, max_y, max_z))
                
                # Calculate center position and dimensions
                position = (
                    (min_world[0] + max_world[0]) / 2,
                    (min_world[1] + max_world[1]) / 2,
                    (min_world[2] + max_world[2]) / 2
                )
                
                dimensions = (
                    max_world[0] - min_world[0],
                    max_world[1] - min_world[1],
                    max_world[2] - min_world[2]
                )
                
                obstacles.append(Obstacle(
                    position=position,
                    dimensions=dimensions,
                    is_static=True,  # Assume static by default
                    type="unknown"
                ))
        
        return obstacles


# ===== SENSOR FUSION =====

class KalmanFilter:
    """Implements a Kalman filter for state estimation."""
    
    def __init__(self, state_dim, measurement_dim):
        """Initialize the Kalman filter.
        
        Args:
            state_dim: Dimension of the state vector
            measurement_dim: Dimension of the measurement vector
        """
        # State vector and covariance
        self.x = np.zeros((state_dim, 1))
        self.P = np.eye(state_dim)
        
        # Process and measurement models
        self.F = np.eye(state_dim)  # State transition model
        self.H = np.zeros((measurement_dim, state_dim))  # Observation model
        
        # Process and measurement noise
        self.Q = np.eye(state_dim) * 0.01  # Process noise
        self.R = np.eye(measurement_dim) * 0.1  # Measurement noise
        
        # Identity matrix for updates
        self.I = np.eye(state_dim)
    
    def predict(self, dt=1.0):
        """Predict step of the Kalman filter."""
        # Update state transition matrix for current dt
        if self.x.shape[0] == 9:  # Position, velocity, acceleration model
            # x, y, z, vx, vy, vz, ax, ay, az
            self.F[0, 3] = dt
            self.F[1, 4] = dt
            self.F[2, 5] = dt
            self.F[3, 6] = dt
            self.F[4, 7] = dt
            self.F[5, 8] = dt
            self.F[0, 6] = 0.5 * dt * dt
            self.F[1, 7] = 0.5 * dt * dt
            self.F[2, 8] = 0.5 * dt * dt
        
        # Predict state and covariance
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x
    
    def update(self, z):
        """Update step of the Kalman filter with measurement z."""
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (self.I - K @ self.H) @ self.P
        
        return self.x


class SensorFusion:
    """Fuses data from multiple sensors to create a unified state estimate."""
    
    def __init__(self):
        """Initialize the sensor fusion module."""
        # State: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw]
        self.state_dim = 12
        self.kf = KalmanFilter(self.state_dim, 6)  # 6 is for position and orientation measurements
        
        # Set observation model for GPS and IMU
        self.kf.H[:3, :3] = np.eye(3)  # Position measurements
        self.kf.H[3:6, 3:6] = np.eye(3)  # Orientation measurements
        
        # Last update timestamp
        self.last_timestamp = None
    
    def update(self, sensor_data):
        """Update state estimate with new sensor data."""
        current_time = time.time()
        
        # Initialize timestamp if first update
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            return DroneState(
                position=(0, 0, 0),
                orientation=(0, 0, 0, 1),  # Quaternion (w, x, y, z)
                velocity=(0, 0, 0),
                angular_velocity=(0, 0, 0),
                acceleration=(0, 0, 0),
                timestamp=current_time
            )
        
        # Calculate time delta
        dt = current_time - self.last_timestamp
        self.last_timestamp = current_time
        
        # Predict step
        self.kf.predict(dt)
        
        # Process GPS data if available
        if any(SensorConfig.sensor_type == SensorType.GPS for _, data in sensor_data.items() if 'config' in data):
            for sensor_id, data in sensor_data.items():
                if 'config' in data and data['config'].sensor_type == SensorType.GPS:
                    gps_data = data['data']
                    # Convert GPS to local coordinates (simplified)
                    # In a real system, this would use proper coordinate transformations
                    x = gps_data['longitude'] * 111320.0  # Rough conversion to meters
                    y = gps_data['latitude'] * 110540.0  # Rough conversion to meters
                    z = gps_data['altitude']
                    
                    # Create measurement vector
                    z = np.array([x, y, z, 0, 0, 0]).reshape(-1, 1)
                    
                    # Update filter with position only
                    measurement_mask = np.zeros((6, self.state_dim))
                    measurement_mask[:3, :3] = np.eye(3)
                    original_H = self.kf.H.copy()
                    self.kf.H = measurement_mask
                    self.kf.update(z)
                    self.kf.H = original_H
        
        # Process IMU data if available
        if any(SensorConfig.sensor_type == SensorType.IMU for _, data in sensor_data.items() if 'config' in data):
            for sensor_id, data in sensor_data.items():
                if 'config' in data and data['config'].sensor_type == SensorType.IMU:
                    imu_data = data['data']
                    
                    # Extract orientation (simplified - would use proper fusion in real system)
                    # In a real system, would use quaternion-based orientation
                    accel = imu_data['acceleration']
                    
                    # Estimate roll and pitch from accelerometer
                    # This is a simplification; real systems use more sophisticated methods
                    roll = np.arctan2(accel[1], np.sqrt(accel[0]**2 + accel[2]**2))
                    pitch = np.arctan2(-accel[0], accel[2])
                    
                    # Yaw cannot be reliably estimated from accelerometer alone
                    # Would use magnetometer and/or gyroscope integration
                    yaw = 0.0
                    
                    # Create measurement vector for orientation
                    z = np.array([0, 0, 0, roll, pitch, yaw]).reshape(-1, 1)
                    
                    # Update filter with orientation only
                    measurement_mask = np.zeros((6, self.state_dim))
                    measurement_mask[3:, 3:6] = np.eye(3)
                    original_H = self.kf.H.copy()
                    self.kf.H = measurement_mask
                    self.kf.update(z)
                    self.kf.H = original_H
                    
                    # Update velocity and angular velocity directly
                    self.kf.x[6:9] = np.array(imu_data['angular_velocity']).reshape(-1, 1)
        
        # Extract state variables
        position = (float(self.kf.x[0]), float(self.kf.x[1]), float(self.kf.x[2]))
        
        # Convert Euler angles to quaternion
        roll, pitch, yaw = float(self.kf.x[3]), float(self.kf.x[4]), float(self.kf.x[5])
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        orientation = (float(qw), float(qx), float(qy), float(qz))
        
        velocity = (float(self.kf.x[6]), float(self.kf.x[7]), float(self.kf.x[8]))
        if len(self.kf.x) > 9:
            angular_velocity = (float(self.kf.x[9]), float(self.kf.x[10]), float(self.kf.x[11]))
        else:
            angular_velocity = (0.0, 0.0, 0.0)
        
        # Get acceleration from IMU directly if available
        acceleration = (0.0, 0.0, 0.0)
        for sensor_id, data in sensor_data.items():
            if 'config' in data and data['config'].sensor_type == SensorType.IMU:
                acceleration = tuple(data['data']['acceleration'])
                break
        
        # Return drone state
        return DroneState(
            position=position,
            orientation=orientation,
            velocity=velocity,
            angular_velocity=angular_velocity,
            acceleration=acceleration,
            timestamp=current_time
        )


# ===== SITUATION AWARENESS =====

class DangerLevel(Enum):
    """Represents the level of danger in the environment."""
    SAFE = 0
    CAUTION = 1
    WARNING = 2
    DANGER = 3
    CRITICAL = 4


class SituationAwareness:
    """Analyzes perception data to understand the environment and threats."""
    
    def __init__(self, safety_distance: float = 2.0):
        """Initialize the situation awareness module.
        
        Args:
            safety_distance: Minimum safe distance to obstacles in meters
        """
        self.safety_distance = safety_distance
        self.detected_objects = []
        self.obstacles = []
        self.drone_state = None
    
    def update(self, drone_state: DroneState, detected_objects: List[DetectedObject], obstacles: List[Obstacle]):
        """Update situation awareness with new perception data."""
        self.drone_state = drone_state
        self.detected_objects = detected_objects
        self.obstacles = obstacles
    
    def assess_danger_level(self):
        """Assess the current danger level based on obstacles and objects."""
        if not self.drone_state:
            return DangerLevel.CAUTION  # No state information
        
        # Check for collision risks with obstacles
        min_distance = float('inf')
        for obstacle in self.obstacles:
            distance = self._calculate_distance(self.drone_state.position, obstacle.position)
            
            # Adjust for obstacle dimensions
            effective_distance = distance - np.linalg.norm(obstacle.dimensions) / 2
            min_distance = min(min_distance, effective_distance)
        
        # Check for collision risks with tracked objects
        for obj in self.detected_objects:
            distance = self._calculate_distance(self.drone_state.position, obj.bbox.center)
            
            # Adjust for object dimensions
            obj_size = np.linalg.norm(obj.bbox.dimensions) / 2
            effective_distance = distance - obj_size
            min_distance = min(min_distance, effective_distance)
        
        # Determine danger level based on distance
        if min_distance <= 0:
            return DangerLevel.CRITICAL  # Collision occurring
        elif min_distance < self.safety_distance * 0.5:
            return DangerLevel.DANGER  # Imminent collision risk
        elif min_distance < self.safety_distance:
            return DangerLevel.WARNING  # Approaching safety threshold
        elif min_distance < self.safety_distance * 2:
            return DangerLevel.CAUTION  # Within monitoring distance
        else:
            return DangerLevel.SAFE  # No immediate threats
    
    def get_nearest_obstacle(self):
        """Get the nearest obstacle and its distance."""
        if not self.drone_state or not self.obstacles:
            return None, float('inf')
        
        nearest_obstacle = None
        min_distance = float('inf')
        
        for obstacle in self.obstacles:
            distance = self._calculate_distance(self.drone_state.position, obstacle.position)
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = obstacle
        
        return nearest_obstacle, min_distance
    
    def get_nearest_object(self, filter_class=None):
        """Get the nearest detected object and its distance, optionally filtered by class."""
        if not self.drone_state or not self.detected_objects:
            return None, float('inf')
        
        nearest_object = None
        min_distance = float('inf')
        
        for obj in self.detected_objects:
            if filter_class is not None and obj.object_class != filter_class:
                continue
                
            distance = self._calculate_distance(self.drone_state.position, obj.bbox.center)
            if distance < min_distance:
                min_distance = distance
                nearest_object = obj
        
        return nearest_object, min_distance
    
    def _calculate_distance(self, point1, point2):
        """Calculate Euclidean distance between two 3D points."""
        return np.sqrt(
            (point1[0] - point2[0])**2 + 
            (point1[1] - point2[1])**2 + 
            (point1[2] - point2[2])**2
        )
    
    def analyze_trajectory(self, planned_trajectory=None):
        """Analyze if the planned trajectory is safe.
        
        Args:
            planned_trajectory: List of waypoints (x, y, z) representing the planned path
            
        Returns:
            is_safe: Boolean indicating if the trajectory is safe
            danger_points: List of (point, obstacle) tuples for unsafe points
        """
        if planned_trajectory is None or not self.drone_state:
            return False, []
        
        is_safe = True
        danger_points = []
        
        # Check each waypoint for potential collisions
        for waypoint in planned_trajectory:
            for obstacle in self.obstacles:
                distance = self._calculate_distance(waypoint, obstacle.position)
                
                # Adjust for obstacle dimensions
                effective_distance = distance - np.linalg.norm(obstacle.dimensions) / 2
                
                if effective_distance < self.safety_distance:
                    is_safe = False
                    danger_points.append((waypoint, obstacle))
        
        return is_safe, danger_points


# ===== MAIN PERCEPTION SYSTEM =====

class DronePerceptionSystem:
    """Main perception system integrating all perception components."""
    
    def __init__(self, sensor_configs: List[SensorConfig]):
        """Initialize the perception system.
        
        Args:
            sensor_configs: List of sensor configurations
        """
        # Initialize components
        self.sensor_acquisition = SensorDataAcquisition(sensor_configs)
        self.object_detector = ObjectDetector()
        self.object_tracker = ObjectTracker()
        self.occupancy_grid = OccupancyGrid()
        self.sensor_fusion = SensorFusion()
        self.situation_awareness = SituationAwareness()
        
        # State variables
        self.drone_state = None
        self.tracked_objects = []
        self.obstacles = []
        self.danger_level = DangerLevel.SAFE
    
    def update(self):
        """Update the perception system with the latest sensor data."""
        # Get sensor data
        sensor_data = self.sensor_acquisition.get_sensor_data()
        
        # Update state estimation
        self.drone_state = self.sensor_fusion.update(sensor_data)
        
        # Process camera data for object detection
        detections = []
        for sensor_id, data in sensor_data.items():
            if 'config' in data and data['config'].sensor_type == SensorType.CAMERA:
                image = data['data']
                if image is not None:
                    new_detections = self.object_detector.detect(image)
                    detections.extend(new_detections)
        
        # Update object tracker
        self.tracked_objects = self.object_tracker.update(detections)
        
        # Update environmental mapping with LiDAR data
        for sensor_id, data in sensor_data.items():
            if 'config' in data and data['config'].sensor_type == SensorType.LIDAR:
                point_cloud = data['data']
                self.occupancy_grid.update_from_point_cloud(
                    point_cloud, 
                    self.drone_state.position
                )
        
        # Extract obstacles from occupancy grid
        self.obstacles = self.occupancy_grid.get_obstacles()
        
        # Update situation awareness
        self.situation_awareness.update(
            self.drone_state,
            self.tracked_objects,
            self.obstacles
        )
        
        # Assess danger level
        self.danger_level = self.situation_awareness.assess_danger_level()
        
        # Return the perception state
        return self.get_perception_state()
    
    def get_perception_state(self):
        """Get the current perception state."""
        return {
            'drone_state': self.drone_state,
            'tracked_objects': self.tracked_objects,
            'obstacles': self.obstacles,
            'danger_level': self.danger_level
        }


# ===== USAGE EXAMPLE =====

def create_example_configuration():
    """Create an example sensor configuration for a drone."""
    sensor_configs = [
        # Front-facing camera
        SensorConfig(
            sensor_id=1,
            sensor_type=SensorType.CAMERA,
            position=(0.1, 0, 0),  # slightly forward from center
            orientation=(0, 0, 0),  # facing forward
            field_of_view=np.radians(90),
            resolution=(640, 480)
        ),
        
        # LiDAR
        SensorConfig(
            sensor_id=2,
            sensor_type=SensorType.LIDAR,
            position=(0, 0, -0.05),  # slightly below center
            orientation=(0, 0, 0),
            field_of_view=np.radians(360),
            range=30.0  # 30 meter range
        ),
        
        # IMU
        SensorConfig(
            sensor_id=3,
            sensor_type=SensorType.IMU,
            position=(0, 0, 0),  # at center
            orientation=(0, 0, 0)
        ),
        
        # GPS
        SensorConfig(
            sensor_id=4,
            sensor_type=SensorType.GPS,
            position=(0, 0, 0),  # at center
            orientation=(0, 0, 0)
        ),
        
        # Downward-facing camera
        SensorConfig(
            sensor_id=5,
            sensor_type=SensorType.CAMERA,
            position=(0, 0, -0.1),  # below center
            orientation=(np.radians(90), 0, 0),  # facing down
            field_of_view=np.radians(90),
            resolution=(640, 480)
        ),
        
        # Ultrasonic sensors for close-range obstacle detection
        SensorConfig(
            sensor_id=6,
            sensor_type=SensorType.ULTRASONIC,
            position=(0.1, 0, 0),  # front
            orientation=(0, 0, 0),
            range=5.0
        ),
        SensorConfig(
            sensor_id=7,
            sensor_type=SensorType.ULTRASONIC,
            position=(-0.1, 0, 0),  # back
            orientation=(np.radians(180), 0, 0),
            range=5.0
        ),
        SensorConfig(
            sensor_id=8,
            sensor_type=SensorType.ULTRASONIC,
            position=(0, 0.1, 0),  # right
            orientation=(0, 0, np.radians(90)),
            range=5.0
        ),
        SensorConfig(
            sensor_id=9,
            sensor_type=SensorType.ULTRASONIC,
            position=(0, -0.1, 0),  # left
            orientation=(0, 0, np.radians(-90)),
            range=5.0
        ),
        SensorConfig(
            sensor_id=10,
            sensor_type=SensorType.ULTRASONIC,
            position=(0, 0, -0.1),  # down
            orientation=(np.radians(90), 0, 0),
            range=5.0
        )
    ]
    
    return sensor_configs


def main():
    """Example usage of the drone perception system."""
    # Create sensor configuration
    sensor_configs = create_example_configuration()
    
    # Initialize perception system
    perception_system = DronePerceptionSystem(sensor_configs)
    
    # Simulation loop
    for _ in range(10):
        # Update perception
        perception_state = perception_system.update()
        
        # Print some state information
        drone_state = perception_state['drone_state']
        danger_level = perception_state['danger_level']
        
        print(f"Drone position: {drone_state.position}")
        print(f"Drone velocity: {drone_state.velocity}")
        print(f"Danger level: {danger_level}")
        print(f"Tracked objects: {len(perception_state['tracked_objects'])}")
        print(f"Obstacles: {len(perception_state['obstacles'])}")
        print("---")
        
        # Simulate time passing
        time.sleep(0.1)


if __name__ == "__main__":
    main()
