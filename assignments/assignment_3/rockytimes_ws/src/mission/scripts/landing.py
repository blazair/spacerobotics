#!/usr/bin/env python3
"""
Drone ArUco Visualizer Node with Landing

Flight sequence:
  1. TAKEOFF: The node uses offboard control to command the drone to reach 20 m altitude.
  2. SEARCH: Once at ~20 m, the node holds altitude and runs ArUco marker detection.
  3. LANDING: When a marker is detected, the node locks onto it and commands the drone
     to move horizontally to the marker’s (x,y) position (based on the marker’s tvec)
     and descend until near ground (e.g. 0.5 m altitude), then disarms.
     
Detection:
  - Uses OpenCV’s ArUco API with the 4×4 dictionary to detect markers from the downward camera.
  - If camera calibration is available (from /drone/down_mono/camera_info), it computes the marker’s pose via solvePnP,
    draws the 3D axes, and overlays text showing marker ID and its (x, y, z) pose.
    
Visualization:
  - The annotated image is displayed in an OpenCV window ("Drone ArUco Visualizer")
    and published on /aruco/debug_image.
  - Marker pose info is also published as text on /aruco/marker_pose.
  
QoS:
  - The camera info subscription uses BEST_EFFORT reliability with VOLATILE durability.
  
Ensure that your markers are generated with the DICT_4X4_50 dictionary.
"""

import rclpy
from rclpy.node import Node
import math, time
import numpy as np
import cv2
from cv_bridge import CvBridge

# ROS2 messages for camera and offboard control
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import mat2euler

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint

# QoS classes
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, DurabilityPolicy

class DroneArucoVisualizer(Node):
    def __init__(self):
        super().__init__('drone_aruco_visualizer')
        
        # --- Offboard Control Setup ---
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Subscribe to odometry to know altitude
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        self.vehicle_odometry = VehicleOdometry()
        
        # --- ArUco Detection & Visualization Setup ---
        self.cv_bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/drone/down_mono', self.image_callback, 10)
        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(CameraInfo, '/drone/down_mono/camera_info', self.camera_info_callback, camera_info_qos)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        
        # Default camera calibration (if not received yet)
        self.camera_matrix = np.array([[554.254691191187, 0.0, 320.5],
                                       [0.0, 554.254691191187, 240.5],
                                       [0.0, 0.0, 1.0]])
        self.dist_coeffs = np.zeros(5)
        self.calibration_received = False
        
        # ArUco setup (using DICT_4X4_50)
        try:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info("Using OpenCV 4.7+ ArUco API with 4x4 dictionary")
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.get_logger().info("Using older OpenCV ArUco API with 4x4 dictionary")
        self.marker_size = 0.8  # meters
        
        # Create local OpenCV window
        cv2.namedWindow("Drone ArUco Visualizer", cv2.WINDOW_NORMAL)
        
        # --- State Machine ---
        # States: "TAKEOFF" -> "SEARCH" -> "LANDING" -> "LANDED"
        self.state = "TAKEOFF"
        self.target_altitude = 20.0  # target altitude in meters
        self.offboard_setpoint_counter = 0
        self.locked_marker = None  # will store a dict with keys 'id' and 'tvec' once a marker is locked
        
        # Timer callback at 20 Hz
        self.create_timer(0.05, self.timer_callback)
    
    # --- Odometry & Camera Info Callbacks ---
    def odometry_callback(self, msg):
        self.vehicle_odometry = msg
    
    def camera_info_callback(self, msg):
        try:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info("Camera calibration received.")
        except Exception as e:
            self.get_logger().error(f"Error processing camera info: {e}")
    
    # --- Image Callback (ArUco Detection) ---
    def image_callback(self, msg):
        try:
            # Convert ROS image (mono8) to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            display_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            
            # Only run detection if we are in SEARCH state (i.e. not already locked onto a marker)
            if self.state == "SEARCH" and self.locked_marker is None:
                if self.detector is not None:
                    corners, ids, _ = self.detector.detectMarkers(cv_image)
                else:
                    corners, ids, _ = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
                
                if ids is not None and len(ids) > 0:
                    # Draw detected markers
                    cv2.aruco.drawDetectedMarkers(display_image, corners, ids)
                    # For simplicity, lock on the first marker detected
                    # (Alternatively, you could choose the one with the smallest tvec[2])
                    marker_points = np.array([
                        [-self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2,  self.marker_size/2, 0],
                        [ self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    objPoints = marker_points.reshape((4,3))
                    imgPoints = corners[0].reshape((4,2))
                    if self.calibration_received:
                        success, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                        if success:
                            self.locked_marker = {'id': ids[0][0], 'tvec': tvec.flatten()}
                            self.get_logger().info(f"Locked on marker {self.locked_marker['id']}: tvec = {self.locked_marker['tvec']}")
                    # Draw marker info on the image
                    for i in range(len(ids)):
                        if self.calibration_received:
                            success, rvec, tvec = cv2.solvePnP(objPoints, corners[i].reshape((4,2)), self.camera_matrix, self.dist_coeffs)
                            if success:
                                cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_size/2)
                                tvec = tvec.flatten()
                                marker_center = corners[i][0].mean(axis=0).astype(int)
                                cv2.putText(display_image,
                                            f"ID:{ids[i][0]} x:{tvec[0]:.2f} y:{tvec[1]:.2f} z:{tvec[2]:.2f}",
                                            (marker_center[0], marker_center[1]-10),
                                            cv2.FONT_HERSHEY_SIMPLEX,
                                            0.5, (0,255,0), 2)
                        else:
                            marker_center = corners[i][0].mean(axis=0).astype(int)
                            cv2.putText(display_image,
                                        f"ID:{ids[i][0]} (no calib)",
                                        (marker_center[0], marker_center[1]),
                                        cv2.FONT_HERSHEY_SIMPLEX,
                                        0.5, (0,0,255), 2)
            
            # Publish debug image and display locally
            debug_msg = self.cv_bridge.cv2_to_imgmsg(display_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            cv2.imshow("Drone ArUco Visualizer", display_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [x, y, z, w]
    
    # --- Offboard Control Methods ---
    def publish_offboard(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_setpoint(self, x, y, z, yaw):
        sp = TrajectorySetpoint()
        sp.position = [x, y, z]
        sp.yaw = yaw
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(sp)
    
    def arm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("Arm command sent")
    
    def engage_offboard(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("Offboard mode command sent")
    
    # --- Timer Callback for Offboard Control & Landing ---
    def timer_callback(self):
        # Send initial offboard setpoints before engaging offboard and arming
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard()
            self.arm()
        self.publish_offboard()
        
        if self.state == "TAKEOFF":
            # Command setpoint to fly to 20 m altitude (z is negative)
            self.publish_setpoint(0.0, 0.0, -20.0, 0.0)
            try:
                alt = -self.vehicle_odometry.position[2]
            except Exception:
                alt = 0.0
            self.get_logger().info(f"Taking off... Altitude: {alt:.2f} m (target: 20.00 m)")
            if alt >= 19.5:
                self.get_logger().info("Reached 20 m altitude. Transitioning to SEARCH state.")
                self.state = "SEARCH"
        elif self.state == "SEARCH":
            # Hold altitude at 20 m and run marker detection (handled in image callback)
            self.publish_setpoint(0.0, 0.0, -20.0, 0.0)
            self.get_logger().info("At 20 m altitude. Running ArUco detection...")
            # If a marker has been locked (from image callback), switch to LANDING state
            if self.locked_marker is not None:
                self.get_logger().info(f"Locked marker {self.locked_marker['id']} detected, transitioning to LANDING state.")
                self.state = "LANDING"
        elif self.state == "LANDING":
            # In LANDING state, use the locked marker's tvec to compute a landing target
            # Assume camera is downward facing; tvec x and y are horizontal offsets
            try:
                current_x = self.vehicle_odometry.position[0]
                current_y = self.vehicle_odometry.position[1]
                current_alt = -self.vehicle_odometry.position[2]
            except Exception:
                current_x, current_y, current_alt = 0.0, 0.0, 20.0
            if self.locked_marker is not None and self.locked_marker.get('tvec') is not None:
                offset = self.locked_marker['tvec']
                # Compute target global position as current position plus the offset (assuming direct addition works)
                target_x = current_x + offset[0]
                target_y = current_y + offset[1]
            else:
                target_x, target_y = current_x, current_y
            # Command landing setpoint: horizontal target and descend to near ground (e.g. 0.5 m altitude)
            target_z = -0.5
            self.publish_setpoint(target_x, target_y, target_z, 0.0)
            self.get_logger().info(f"Landing setpoint: x={target_x:.2f}, y={target_y:.2f}, z={target_z:.2f}")
            # If altitude is low enough, assume landing complete and disarm
            if current_alt < 1.0:
                self.get_logger().info("Landing complete. Disarming.")
                cmd = VehicleCommand()
                cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
                cmd.param1 = 0.0
                cmd.target_system = 1
                cmd.target_component = 1
                cmd.source_system = 1
                cmd.source_component = 1
                cmd.from_external = True
                cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                self.vehicle_command_pub.publish(cmd)
                self.state = "LANDED"
        elif self.state == "LANDED":
            self.get_logger().info("Drone has landed and disarmed.")
        
        self.offboard_setpoint_counter += 1
    
    # Additional subscription for odometry (if needed)
    def odometry_callback(self, msg):
        self.vehicle_odometry = msg

def main():
    rclpy.init()
    node = DroneArucoVisualizer()
    # Subscribe to odometry with same QoS as offboard topics
    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )
    node.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', node.odometry_callback, qos_profile)
    node.create_timer(0.05, node.timer_callback)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Drone ArUco Visualizer node.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
