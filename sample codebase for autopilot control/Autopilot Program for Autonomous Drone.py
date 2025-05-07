# Autonomous Drone Autopilot Program
# Description: This Python script uses ROS and ArduPilot's MAVROS to control a drone autonomously.
# It includes waypoint navigation, obstacle avoidance using a LIDAR sensor, and state monitoring.
# Prerequisites:
# - ArduPilot firmware on flight controller (e.g., Pixhawk)
# - ROS (Robot Operating System) installed on a companion computer (e.g., Raspberry Pi)
# - MAVROS package for communication with ArduPilot
# - LIDAR sensor (e.g., RPLIDAR) for obstacle detection
# - Python 3.x, rospy, mavros_msgs, sensor_msgs, geometry_msgs

import rospy
from mavros_msgs.msg import State, Waypoint, WaypointList
from mavros_msgs.srv import SetMode, CommandBool, WaypointPush
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import math
import time

class DroneAutopilot:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('drone_autopilot', anonymous=True)
        
        # MAVROS subscriptions and publishers
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_cb)
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # MAVROS services
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.waypoint_push_srv = rospy.ServiceProxy('/mavros/mission/push', WaypointPush)
        
        # Drone state variables
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.lidar_ranges = []
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Autopilot parameters
        self.target_altitude = 5.0  # meters
        self.waypoint_tolerance = 0.5  # meters
        self.obstacle_threshold = 2.0  # meters
        
        # Initialize pose message
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        
    def state_cb(self, state):
        """Callback for drone state updates"""
        self.current_state = state
        
    def pose_cb(self, pose):
        """Callback for drone position updates"""
        self.current_pose = pose
        
    def lidar_cb(self, scan):
        """Callback for LIDAR data"""
        self.lidar_ranges = scan.ranges
        
    def set_waypoints(self):
        """Define a list of waypoints (latitude, longitude, altitude)"""
        # Example waypoints (replace with actual coordinates)
        waypoint1 = Waypoint()
        waypoint1.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint1.command = 16  # NAV_WAYPOINT
        waypoint1.is_current = True
        waypoint1.autocontinue = True
        waypoint1.x_lat = 47.397742
        waypoint1.y_long = 8.545594
        waypoint1.z_alt = self.target_altitude
        
        waypoint2 = Waypoint()
        waypoint2.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint2.command = 16
        waypoint2.is_current = False
        waypoint2.autocontinue = True
        waypoint2.x_lat = 47.397842
        waypoint2.y_long = 8.546594
        waypoint2.z_alt = self.target_altitude
        
        self.waypoints = [waypoint1, waypoint2]
        
        # Push waypoints to flight controller
        try:
            self.waypoint_push_srv(self.waypoints)
            rospy.loginfo("Waypoints pushed successfully")
        except rospy.ServiceException as e:
            rospy.logerr("Waypoint push failed: %s" % e)
            
    def check_obstacle(self):
        """Check for obstacles using LIDAR data"""
        if not self.lidar_ranges:
            return False
        
        # Check front-facing LIDAR ranges (e.g., -30 to 30 degrees)
        front_ranges = self.lidar_ranges[0:30] + self.lidar_ranges[-30:]
        min_distance = min([r for r in front_ranges if r > 0], default=float('inf'))
        
        if min_distance < self.obstacle_threshold:
            rospy.logwarn("Obstacle detected at %.2f meters", min_distance)
            return True
        return False
    
    def navigate_to_waypoint(self):
        """Navigate to the current waypoint"""
        if self.current_waypoint_index >= len(self.waypoints):
            rospy.loginfo("All waypoints reached")
            return False
            
        target = self.waypoints[self.current_waypoint_index]
        self.target_pose.pose.position.x = target.x_lat  # Simplified for local frame
        self.target_pose.pose.position.y = target.y_long
        self.target_pose.pose.position.z = target.z_alt
        
        # Publish target pose
        self.target_pose.header.stamp = rospy.get_rostime()
        self.local_pos_pub.publish(self.target_pose)
        
        # Check if waypoint is reached
        dx = self.current_pose.pose.position.x - target.x_lat
        dy = self.current_pose.pose.position.y - target.y_long
        dz = self.current_pose.pose.position.z - target.z_alt
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance < self.waypoint_tolerance:
            rospy.loginfo("Waypoint %d reached", self.current_waypoint_index)
            self.current_waypoint_index += 1
            return False
        return True
    
    def run(self):
        """Main autopilot loop"""
        rate = rospy.Rate(20)  # 20 Hz
        
        # Wait for FCU connection
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.loginfo("Waiting for FCU connection...")
            rate.sleep()
            
        # Set mode to GUIDED and arm the drone
        try:
            self.set_mode_srv(custom_mode="GUIDED")
            self.arming_srv(True)
            rospy.loginfo("Drone armed and in GUIDED mode")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return
            
        # Set waypoints
        self.set_waypoints()
        
        # Main loop
        while not rospy.is_shutdown():
            if self.check_obstacle():
                # Stop and hover if obstacle detected
                self.target_pose.pose.position = self.current_pose.pose.position
                self.local_pos_pub.publish(self.target_pose)
                rospy.loginfo("Hovering due to obstacle")
            else:
                # Navigate to waypoint
                if not self.navigate_to_waypoint():
                    # Land if all waypoints are reached
                    try:
                        self.set_mode_srv(custom_mode="LAND")
                        rospy.loginfo("Initiating landing")
                        break
                    except rospy.ServiceException as e:
                        rospy.logerr("Landing failed: %s" % e)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        autopilot = DroneAutopilot()
        autopilot.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autopilot node terminated")