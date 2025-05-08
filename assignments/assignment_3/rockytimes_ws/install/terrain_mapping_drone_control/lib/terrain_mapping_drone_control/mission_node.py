#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import math
import time

class AutonomousCylinderLanding(Node):
    def __init__(self):
        super().__init__('autonomous_cylinder_landing')
        self.bridge = CvBridge()

        # FSM states: SEARCH -> APPROACH -> LAND -> COMPLETE
        self.state = "SEARCH"
        self.start_time = time.time()
        self.energy_used = 0.0  # Dummy energy counter

        # ArUco marker parameters
        self.marker_size = 0.15  # Known physical marker size (meters)
        # Camera intrinsics (update with your calibrated values if available)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0,   0,   1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1))  # Assuming minimal distortion
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        # Subscribers: front RGB, front Depth, and odometry (world pose)
        self.rgb_sub = self.create_subscription(Image, '/drone/front_rgb', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/drone/front_depth', self.depth_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/fmu/out/vehicle_odometry', self.odom_callback, 10)

        # Publisher for setpoints for offboard control
        self.sp_pub = self.create_publisher(PoseStamped, '/fmu/in/vehicle_local_position_setpoint', 10)
        # Publisher for logging the landing target (optional)
        self.target_pub = self.create_publisher(PoseStamped, '/drone/landing_target', 10)

        # Timer for state machine updates (10 Hz)
        self.timer = self.create_timer(0.1, self.update_state)

        # Sensor data storage
        self.current_rgb = None
        self.current_depth = None
        self.current_pose = None  # from odometry
        self.detected_markers = {}  # marker_id -> (rvec, tvec)

        # Define a search pattern: here a circular (lawnmower‐like) pattern around the origin
        self.search_pattern = self.generate_search_pattern(center=(0, 0), radius=5.0, num_points=12, altitude=2.0)
        self.search_index = 0

        # Information about the target cylinder
        # Format: {"id": marker_id, "height": estimated_height, "diameter": estimated_diameter, "world_position": np.array([x,y,z])}
        self.target_cylinder = None

    def generate_search_pattern(self, center, radius, num_points, altitude):
        waypoints = []
        cx, cy = center
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            z = altitude
            waypoints.append((x, y, z))
        return waypoints

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def rgb_callback(self, msg: Image):
        try:
            self.current_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def depth_callback(self, msg: Image):
        try:
            # Assume depth is encoded as 32FC1
            self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def update_state(self):
        # Update dummy energy usage (increment every cycle)
        dt = 0.1  # timer period (seconds)
        self.energy_used += 0.05  # dummy value

        # Process image data for marker detection if available
        if self.current_rgb is not None and self.current_depth is not None:
            self.process_marker_detection(self.current_rgb, self.current_depth)

        # State machine logic
        if self.state == "SEARCH":
            self.search_state()
        elif self.state == "APPROACH":
            self.approach_state()
        elif self.state == "LAND":
            self.land_state()
        elif self.state == "COMPLETE":
            self.complete_state()
        else:
            self.get_logger().warn(f"Unknown state: {self.state}")

    def process_marker_detection(self, rgb_image, depth_image):
        # Detect ArUco markers in the RGB image
        gray = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        self.detected_markers.clear()
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size,
                                                               self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                self.detected_markers[int(marker_id)] = (rvec, tvec)
                # Draw markers (for visualization)
                aruco.drawDetectedMarkers(rgb_image, corners)
                aruco.drawAxis(rgb_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        # Optionally, display the detection window (if a display is available)
        cv2.imshow("Autonomous Search", rgb_image)
        cv2.waitKey(1)

        # If markers are detected and odometry is available, estimate dimensions
        if self.detected_markers and self.current_pose is not None:
            for marker_id, (rvec, tvec) in self.detected_markers.items():
                # Transform marker position (tvec) from the camera frame to world coordinates.
                marker_world = self.transform_to_world(tvec)
                # For this example, assume the marker’s world z coordinate represents the cylinder’s top.
                estimated_height = marker_world[2]
                # For diameter, we use a dummy estimation based on marker size
                estimated_diameter = self.marker_size * 1.5  # factor as placeholder

                self.get_logger().info(f"Marker {marker_id}: Height {estimated_height:.2f} m, Diameter {estimated_diameter:.2f} m")

                # Update target cylinder if this one is taller than previously seen
                if self.target_cylinder is None or estimated_height > self.target_cylinder["height"]:
                    self.target_cylinder = {
                        "id": marker_id,
                        "height": estimated_height,
                        "diameter": estimated_diameter,
                        "world_position": marker_world
                    }
                    self.get_logger().info(f"Target cylinder updated to marker {marker_id} with height {estimated_height:.2f} m")
                    # Publish the target landing pose for logging/visualization
                    target_pose = PoseStamped()
                    target_pose.header.stamp = self.get_clock().now().to_msg()
                    target_pose.header.frame_id = "world"
                    target_pose.pose.position.x = marker_world[0]
                    target_pose.pose.position.y = marker_world[1]
                    target_pose.pose.position.z = marker_world[2] + 0.2  # hover slightly above
                    target_pose.pose.orientation.w = 1.0
                    self.target_pub.publish(target_pose)

    def transform_to_world(self, tvec):
        # Transform a marker's translation vector from the camera (assumed aligned with the drone)
        # to the world frame using the drone's current pose.
        if self.current_pose is None:
            return np.array([0, 0, 0])
        drone_pos = np.array([self.current_pose.position.x,
                              self.current_pose.position.y,
                              self.current_pose.position.z])
        # Convert drone's quaternion to rotation matrix.
        q = self.current_pose.orientation
        quat = np.array([q.w, q.x, q.y, q.z])
        R = self.quaternion_to_rotation_matrix(quat)
        tvec_np = np.array(tvec)
        marker_world = drone_pos + R.dot(tvec_np)
        return marker_world

    def quaternion_to_rotation_matrix(self, q):
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
            [2*(x*y + z*w),   1 - 2*(x*x + z*z),   2*(y*z - x*w)],
            [2*(x*z - y*w),   2*(y*z + x*w),   1 - 2*(x*x + y*y)]
        ])
        return R

    def publish_setpoint(self, x, y, z):
        sp = PoseStamped()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.header.frame_id = "world"
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.w = 1.0  # Neutral orientation
        self.sp_pub.publish(sp)

    def search_state(self):
        # In SEARCH state, follow the predefined pattern.
        if self.current_pose is None:
            return
        target_wp = self.search_pattern[self.search_index]
        self.publish_setpoint(*target_wp)
        curr_pos = np.array([self.current_pose.position.x,
                             self.current_pose.position.y,
                             self.current_pose.position.z])
        wp_pos = np.array(target_wp)
        if np.linalg.norm(curr_pos - wp_pos) < 0.5:
            self.search_index = (self.search_index + 1) % len(self.search_pattern)
            self.get_logger().info(f"Reached search waypoint {self.search_index}, moving to next point.")

        # If a target cylinder has been detected, switch to APPROACH state.
        if self.target_cylinder is not None:
            self.get_logger().info("Target cylinder detected. Switching to APPROACH state.")
            self.state = "APPROACH"

    def approach_state(self):
        # In APPROACH state, navigate to a point above the target cylinder.
        if self.target_cylinder is None:
            self.state = "SEARCH"
            return
        target_pos = self.target_cylinder["world_position"]
        approach_altitude = target_pos[2] + 1.0  # hover 1 m above the cylinder top
        self.publish_setpoint(target_pos[0], target_pos[1], approach_altitude)
        curr_pos = np.array([self.current_pose.position.x,
                             self.current_pose.position.y,
                             self.current_pose.position.z])
        approach_point = np.array([target_pos[0], target_pos[1], approach_altitude])
        if np.linalg.norm(curr_pos - approach_point) < 0.5:
            self.get_logger().info("Approached target cylinder. Switching to LAND state.")
            self.state = "LAND"

    def land_state(self):
        # In LAND state, command the drone to descend onto the cylinder.
        if self.target_cylinder is None:
            self.state = "SEARCH"
            return
        target_pos = self.target_cylinder["world_position"]
        landing_altitude = target_pos[2]  # Assume landing at the top of the cylinder
        self.publish_setpoint(target_pos[0], target_pos[1], landing_altitude)
        curr_pos = np.array([self.current_pose.position.x,
                             self.current_pose.position.y,
                             self.current_pose.position.z])
        landing_point = np.array([target_pos[0], target_pos[1], landing_altitude])
        if np.linalg.norm(curr_pos - landing_point) < 0.2:
            self.get_logger().info("Landed on target cylinder. Mission complete.")
            self.state = "COMPLETE"

    def complete_state(self):
        # Log mission performance data.
        total_time = time.time() - self.start_time
        self.get_logger().info(f"Mission complete. Total time: {total_time:.2f} s, Energy used: {self.energy_used:.2f} units")
        # Optionally, stop sending setpoints or shutdown the node.

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousCylinderLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down autonomous cylinder landing node.")
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
