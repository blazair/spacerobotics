#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math, time
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import cv2
import numpy as np
from sklearn.cluster import KMeans

import matplotlib.pyplot as plt
import statistics

class CircleFlightCylinderEstimator(Node):
    def __init__(self):
        super().__init__('circle_flight_cylinder_estimator')
        self.get_logger().info("CircleFlightCylinderEstimator node starting.")

        # --- Offboard Flight Setup ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.gimbal_yaw_pub = self.create_publisher(Float64, '/model/gimbal_yaw', 10)

        # Flight parameters
        self.takeoff_alt = 6.0         # meters
        self.hover_radius = 15.0       # drone will hover at a fixed position 15m from origin
        # We'll set the hover position to [15, 0, -6] in a NED system.
        self.hover_position = [self.hover_radius, 0.0, -self.takeoff_alt]
        # Yaw is set so that the drone faces the origin.
        self.hover_yaw = math.atan2(-0.0, -(self.hover_radius))  # should be pi

        self.state = "TAKEOFF"
        self.flight_start_time = time.time()

        # --- Image Processing Setup ---
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        # Correct subscription usage: (message type, topic name, callback, QoS)
        self.rgb_sub = self.create_subscription(
            Image, '/drone/front_rgb', self.rgb_callback, QoSProfile(depth=1))
        self.depth_sub = self.create_subscription(
            Image, '/drone/front_depth', self.depth_callback, QoSProfile(depth=1))

        # Camera intrinsics (example values; adjust as needed)
        self.fx, self.fy = 525.0, 525.0
        self.cx, self.cy = 319.5, 239.5

        # For feature detection: ORB detector.
        self.orb = cv2.ORB_create(nfeatures=500)
        
        # Measurements storage for live plotting.
        self.frame_count = 0
        self.times = []
        self.cyl1_heights = []
        self.cyl1_widths = []
        self.cyl2_heights = []
        self.cyl2_widths = []

        # Set up live Matplotlib plots.
        plt.ion()
        self.fig, (self.ax_height, self.ax_width) = plt.subplots(2, 1, figsize=(8,6))
        self.ax_height.set_title("Cylinder Height (m)")
        self.ax_height.set_xlabel("Frame")
        self.ax_height.set_ylabel("Height (m)")
        self.ax_width.set_title("Cylinder Width (m)")
        self.ax_width.set_xlabel("Frame")
        self.ax_width.set_ylabel("Width (m)")

        # Create timers: one for flight control and one for image processing.
        self.flight_timer = self.create_timer(0.05, self.flight_timer_callback)
        self.image_timer = self.create_timer(0.5, self.image_timer_callback)

    # ------------------ Flight Control (Hover) ------------------
    def flight_timer_callback(self):
        self.publish_offboard_mode()
        if self.state == "TAKEOFF":
            # Command takeoff to our desired hover position.
            sp = TrajectorySetpoint()
            sp.position = self.hover_position  # e.g., [15, 0, -6]
            sp.yaw = self.hover_yaw           # should face the origin.
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_pub.publish(sp)
            if time.time() - self.flight_start_time > 5.0:
                self.get_logger().info("Takeoff complete. Switching to HOVER mode.")
                self.state = "HOVER"
                self.flight_start_time = time.time()
                self.arm_drone()
                self.engage_offboard_mode()
        elif self.state == "HOVER":
            # Maintain hover at fixed position.
            sp = TrajectorySetpoint()
            sp.position = self.hover_position
            sp.yaw = self.hover_yaw
            sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.trajectory_setpoint_pub.publish(sp)
            # Ensure camera (gimbal) faces the origin.
            gimbal_msg = Float64()
            gimbal_msg.data = self.hover_yaw
            self.gimbal_yaw_pub.publish(gimbal_msg)
    
    def publish_offboard_mode(self):
        mode_msg = OffboardControlMode()
        mode_msg.position = True
        mode_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(mode_msg)

    def arm_drone(self):
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
        self.get_logger().info("Arm command sent.")

    def engage_offboard_mode(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0  # Offboard mode number.
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info("Offboard mode engaged.")

    # ------------------ Image Processing for Cylinder Detection ------------------
    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error("RGB conversion error: " + str(e))

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error("Depth conversion error: " + str(e))

    def image_timer_callback(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        # Use ORB to detect features in the RGB image.
        gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        keypoints = self.orb.detect(gray, None)
        if len(keypoints) < 2:
            self.get_logger().info("Not enough keypoints detected.")
            cv2.imshow("Cylinder Estimation Overlay", self.rgb_image)
            cv2.waitKey(1)
            return

        # Cluster keypoints into 2 groups using KMeans.
        pts = np.array([kp.pt for kp in keypoints])
        try:
            kmeans = KMeans(n_clusters=2, random_state=0).fit(pts)
        except Exception as e:
            self.get_logger().error("KMeans error: " + str(e))
            return
        labels = kmeans.labels_

        boxes = []
        for i in range(2):
            cluster_pts = pts[labels == i]
            if cluster_pts.size == 0:
                continue
            # Compute bounding box around the cluster.
            x_min = int(np.min(cluster_pts[:, 0]))
            y_min = int(np.min(cluster_pts[:, 1]))
            x_max = int(np.max(cluster_pts[:, 0]))
            y_max = int(np.max(cluster_pts[:, 1]))
            # Optionally, expand the box by a fixed margin (e.g. 10 pixels) to better cover the cylinder.
            margin = 10
            x_min = max(x_min - margin, 0)
            y_min = max(y_min - margin, 0)
            x_max = x_max + margin
            y_max = y_max + margin
            boxes.append((x_min, y_min, x_max - x_min, y_max - y_min))

        # Sort boxes left-to-right by x-coordinate.
        boxes.sort(key=lambda b: b[0])
        annotated = self.rgb_image.copy()
        measurements = []
        for idx, box in enumerate(boxes):
            x, y, w, h = box
            color = (0, 255, 0) if idx == 0 else (0, 0, 255)
            cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)
            # Get ROI from the depth image.
            roi = self.depth_image[y:y + h, x:x + w]
            valid = roi[(roi > 0) & (~np.isnan(roi))]
            if valid.size == 0:
                continue
            # Remove the multiplication by 0.001 since the depth values are already in meters.
            median_depth = float(np.median(valid))
            # Convert bounding box dimensions (in pixels) to real-world dimensions.
            phys_width = (w * median_depth) / self.fx
            phys_height = (h * median_depth) / self.fy
            measurements.append({
                "box": box,
                "depth": median_depth,
                "width": phys_width,
                "height": phys_height
            })
            label_text = f"{'Cylinder' if len(boxes)==1 else ('Left' if idx==0 else 'Right')}: {phys_height:.2f}m x {phys_width:.2f}m"
            cv2.putText(annotated, label_text, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            self.get_logger().info(label_text)

        if len(measurements) >= 1:  # now we assume one cylinder if only one is detected.
            self.frame_count += 1
            self.times.append(self.frame_count)
            # If two boxes detected, assign first to cylinder1 and second to cylinder2.
            if len(measurements) >= 2:
                self.cyl1_heights.append(measurements[0]["height"])
                self.cyl1_widths.append(measurements[0]["width"])
                self.cyl2_heights.append(measurements[1]["height"])
                self.cyl2_widths.append(measurements[1]["width"])
            else:
                # If only one is detected, store it for cylinder1.
                self.cyl1_heights.append(measurements[0]["height"])
                self.cyl1_widths.append(measurements[0]["width"])
            self.update_plots()

        cv2.imshow("Cylinder Estimation Overlay", annotated)
        cv2.waitKey(1)

    def update_plots(self):
        self.ax_height.cla()
        self.ax_width.cla()
        if self.cyl1_heights:
            self.ax_height.plot(self.times[:len(self.cyl1_heights)], self.cyl1_heights, 'g.-', label="Cylinder 1 Height")
        if self.cyl2_heights:
            self.ax_height.plot(self.times[:len(self.cyl2_heights)], self.cyl2_heights, 'r.-', label="Cylinder 2 Height")
        self.ax_height.set_title("Cylinder Height (m)")
        self.ax_height.set_xlabel("Frame")
        self.ax_height.set_ylabel("Height (m)")
        self.ax_height.legend()

        if self.cyl1_widths:
            self.ax_width.plot(self.times[:len(self.cyl1_widths)], self.cyl1_widths, 'g.-', label="Cylinder 1 Width")
        if self.cyl2_widths:
            self.ax_width.plot(self.times[:len(self.cyl2_widths)], self.cyl2_widths, 'r.-', label="Cylinder 2 Width")
        self.ax_width.set_title("Cylinder Width (m)")
        self.ax_width.set_xlabel("Frame")
        self.ax_width.set_ylabel("Width (m)")
        self.ax_width.legend()

        self.fig.tight_layout()
        plt.pause(0.001)

    def shutdown_hook(self):
        if self.cyl1_heights:
            median_h1 = statistics.median(self.cyl1_heights)
            median_w1 = statistics.median(self.cyl1_widths)
            self.get_logger().info(f"Final Cylinder 1: Height = {median_h1:.2f} m, Width = {median_w1:.2f} m")
        else:
            self.get_logger().info("No measurements for Cylinder 1.")
        if self.cyl2_heights:
            median_h2 = statistics.median(self.cyl2_heights)
            median_w2 = statistics.median(self.cyl2_widths)
            self.get_logger().info(f"Final Cylinder 2: Height = {median_h2:.2f} m, Width = {median_w2:.2f} m")
        else:
            self.get_logger().info("No measurements for Cylinder 2.")
        rclpy.shutdown()
        cv2.destroyAllWindows()
        plt.close('all')

def main(args=None):
    rclpy.init(args=args)
    node = CircleFlightCylinderEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
