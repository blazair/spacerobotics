#!/usr/bin/env python3
"""
Marker Landing Mission with IBVS Landing:

Sequence:
  1. ARM_TAKEOFF:
       Command the drone to (0, 0, -13)  --> Drone flies at ~13 m AGL.
  2. GOTO_FIRST:
       Fly slowly to (0, 5, -13) for marker sample detection.
  3. DETECT_FIRST:
       Hover for 5 seconds and record one marker sample (around 4 sec).
  4. RETURN_CENTER_FIRST:
       Return to (0, 0, -13) and wait 2 sec.
  5. GOTO_SECOND:
       Fly slowly to (0, -5, -13) for another marker sample.
  6. DETECT_SECOND:
       Hover for 5 sec and record one marker sample.
  7. RETURN_CENTER_SECOND:
       Return to (0, 0, -13) and wait 2 sec.
  8. COMPARE:
       Compare the two samples by z value (choose the marker with lower z, i.e. closer).
  9. APPROACH:
       Move laterally to either (0, 5, -13) or (0, -5, -13) based on the chosen marker.
 10. LAND_IBVS:
       Engage IBVS landing:
         - Lateral error: marker (x, y) error (aim for 0,0 in image space).
         - Altitude error: computed as current_z - DESIRED_ALT (with DESIRED_ALT = 0).
         With current_z negative, a positive vz will be commanded so that the new setpoint is less negative (descent).
         The control law is:
           vx = -K_LAT * marker_x,  vy = -K_LAT * marker_y,
           vz = -K_ALT * (current_z - DESIRED_ALT)
         The velocities are integrated (dt = 0.1 s) to update position setpoints.
 11. DONE:
       Mission complete.
       
Assumptions:
  • An external node (the ArUco tracker) publishes marker poses as text.
  • The downward-facing camera feed is available on /drone/down_mono.
  • The published debug image (/aruco/debug_image) will have the drone position overlaid.
"""

import math
import time
import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.logging import LoggingSeverity

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Controller gains (tune these for smooth behavior)
K_LAT = 0.1    # Gain for lateral corrections (x,y)
K_ALT = 0.05   # Gain for altitude correction

# Error threshold to switch to landing command
ERROR_THRESHOLD = 0.1

# Desired landing altitude (ground level)
DESIRED_ALT = 0.0

class MarkerLanding(Node):
    def __init__(self):
        super().__init__('marker_landing_node')

        # --- Setup Publishers and Subscribers ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.marker_pose_cb, 10)
        self.down_mono_sub = self.create_subscription(
            Image, '/drone/down_mono', self.down_mono_callback, 10)
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.cv_bridge = CvBridge()
        
        # --- State Machine Setup ---
        # Mission states:
        # WAIT -> ARM_TAKEOFF -> GOTO_FIRST -> DETECT_FIRST -> RETURN_CENTER_FIRST ->
        # GOTO_SECOND -> DETECT_SECOND -> RETURN_CENTER_SECOND -> COMPARE -> APPROACH -> LAND_IBVS -> DONE
        self.state = "WAIT"
        self.offboard_counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- Drone's current position [x, y, z] (z is negative when flying) ---
        # We start with a nominal altitude of -13 (i.e., 13 m above ground in NED)
        self.position = [0.0, 0.0, -13.0]

        # --- Marker Detection Samples (a single sample, no averaging) ---
        self.marker1_sample = None  # from DETECT_FIRST
        self.marker1_sample_time = None
        self.marker2_sample = None  # from DETECT_SECOND
        self.marker2_sample_time = None

        # These are used in the COMPARE phase.
        self.marker1_pos = None  
        self.marker2_pos = None  

        self.detect_start_time = None  # For detection phases
        self.return_start_time = None  # For return-to-center phases

        # Chosen marker (after COMPARE) and its lateral target.
        self.chosen_marker_pos = None
        self.chosen_direction_y = None

        # Latest marker pose (from tracker, continuously updated)
        self.latest_marker_pose = None

    # --- Odometry Callback ---
    def odom_cb(self, msg: VehicleOdometry):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    # --- Marker Pose Callback ---
    def marker_pose_cb(self, msg: String):
        # Expected string: "Marker 0 detected at x:1.23m, y:4.56m, z:7.89m"
        line = msg.data.strip()
        match = re.search(r"x:\s*([-\d.]+).*y:\s*([-\d.]+).*z:\s*([-\d.]+)", line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            self.latest_marker_pose = (x, y, z)
            # Record a single sample for the current detection state.
            if self.state == "DETECT_FIRST":
                self.marker1_sample = (x, y, z)
                self.marker1_sample_time = time.time()
                self.get_logger().info(f"DETECT_FIRST sample: {(x, y, z)}")
            elif self.state == "DETECT_SECOND":
                self.marker2_sample = (x, y, z)
                self.marker2_sample_time = time.time()
                self.get_logger().info(f"DETECT_SECOND sample: {(x, y, z)}")

    # --- Down Mono Callback for Debug Overlay ---
    def down_mono_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error("Failed to convert down mono image")
            return

        text = f"Drone Pos: x:{self.position[0]:.2f}, y:{self.position[1]:.2f}, z:{self.position[2]:.2f}"
        cv2.putText(cv_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255,255,255), 2)
        debug_msg = self.cv_bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header.stamp = msg.header.stamp
        self.debug_image_pub.publish(debug_msg)

    # --- Main Timer Callback (State Machine) ---
    def timer_callback(self):
        self.publish_offboard_control_mode()
        if self.offboard_counter == 5 and self.state not in ("WAIT", "DONE"):
            self.engage_offboard_mode()
            self.arm()

        # WAIT → ARM_TAKEOFF
        if self.state == "WAIT":
            self.get_logger().info("Starting mission => ARM_TAKEOFF")
            self.state = "ARM_TAKEOFF"

        elif self.state == "ARM_TAKEOFF":
            target = [0.0, 0.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            if self._dist_to(target) < 0.5:
                self.get_logger().info("Arrived at (0,0,-13). Next => GOTO_FIRST")
                self.state = "GOTO_FIRST"

        # GOTO_FIRST → fly to (0, 5, -13) for first marker sample detection.
        elif self.state == "GOTO_FIRST":
            target = [0.0, 5.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            if self._dist_to(target) < 0.5:
                self.get_logger().info("Arrived at (0,5,-13). Starting DETECT_FIRST (5s detection).")
                self.marker1_sample = None
                self.marker1_sample_time = None
                self.detect_start_time = time.time()
                self.state = "DETECT_FIRST"

        elif self.state == "DETECT_FIRST":
            target = [0.0, 5.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            elapsed = time.time() - self.detect_start_time
            if elapsed >= 4.0 and self.marker1_sample is not None and self.marker1_pos is None:
                self.marker1_pos = self.marker1_sample
                self.get_logger().info(f"DETECT_FIRST marker chosen: {self.marker1_pos}")
            if elapsed >= 5.0:
                if self.marker1_pos is None:
                    self.marker1_pos = (999.0, 999.0, 999.0)
                    self.get_logger().warn("DETECT_FIRST: No sample received; using default")
                self.return_start_time = time.time()
                self.state = "RETURN_CENTER_FIRST"
                self.get_logger().info("DETECT_FIRST complete; returning to center.")

        elif self.state == "RETURN_CENTER_FIRST":
            target = [0.0, 0.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            if time.time() - self.return_start_time >= 2.0:
                self.get_logger().info("Returned to center after first detection. Now GOTO_SECOND.")
                self.state = "GOTO_SECOND"

        # GOTO_SECOND → fly to (0, -5, -13) for second marker sample.
        elif self.state == "GOTO_SECOND":
            target = [0.0, -5.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            if self._dist_to(target) < 0.5:
                self.get_logger().info("Arrived at (0,-5,-13). Starting DETECT_SECOND (5s detection).")
                self.marker2_sample = None
                self.marker2_sample_time = None
                self.detect_start_time = time.time()
                self.state = "DETECT_SECOND"

        elif self.state == "DETECT_SECOND":
            target = [0.0, -5.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            elapsed = time.time() - self.detect_start_time
            if elapsed >= 4.0 and self.marker2_sample is not None and self.marker2_pos is None:
                self.marker2_pos = self.marker2_sample
                self.get_logger().info(f"DETECT_SECOND marker chosen: {self.marker2_pos}")
            if elapsed >= 5.0:
                if self.marker2_pos is None:
                    self.marker2_pos = (999.0, 999.0, 999.0)
                    self.get_logger().warn("DETECT_SECOND: No sample received; using default")
                self.return_start_time = time.time()
                self.state = "RETURN_CENTER_SECOND"
                self.get_logger().info("DETECT_SECOND complete; returning to center.")

        elif self.state == "RETURN_CENTER_SECOND":
            target = [0.0, 0.0, -13.0]
            self.publish_trajectory_setpoint(*target)
            if time.time() - self.return_start_time >= 2.0:
                self.get_logger().info("Returned to center after second detection. Now comparing markers.")
                self.state = "COMPARE"

        # COMPARE: choose the marker with lower z (closer in marker's frame)
        elif self.state == "COMPARE":
            if self.marker1_pos is None or self.marker2_pos is None:
                self.get_logger().error("Missing marker positions! Aborting mission.")
                self.state = "DONE"
                return
            if self.marker1_pos[2] < self.marker2_pos[2]:
                chosen = self.marker1_pos
                chosen_direction = +5.0  # Approach from y = +5
                self.get_logger().info(f"Chosen marker from DETECT_FIRST: {chosen}")
            else:
                chosen = self.marker2_pos
                chosen_direction = -5.0  # Approach from y = -5
                self.get_logger().info(f"Chosen marker from DETECT_SECOND: {chosen}")
            self.chosen_marker_pos = chosen
            self.chosen_direction_y = chosen_direction
            self.state = "APPROACH"

        elif self.state == "APPROACH":
            # Approach target becomes (0, chosen_direction, -13)
            target = [0.0, self.chosen_direction_y, -13.0]
            self.publish_trajectory_setpoint(*target)
            if self._dist_to(target) < 0.5:
                self.get_logger().info(f"Approached chosen cylinder at (0,{self.chosen_direction_y},-13).")
                self.state = "LAND_IBVS"

        elif self.state == "LAND_IBVS":
            # LAND_IBVS: Use lateral marker error (assume desired marker image position is (0,0))
            # For altitude, compute error using the drone's current altitude vs. the desired landing altitude (0 m).
            if self.latest_marker_pose is None:
                self.get_logger().warn("No marker feedback in LAND_IBVS; holding position.")
                self.publish_trajectory_setpoint(*self.position)
                return
            marker_x, marker_y, _ = self.latest_marker_pose
            error_x = marker_x
            error_y = marker_y
            # Altitude error: current_z (e.g. -13) minus DESIRED_ALT (0) => e_z = -13; then controller produces
            # a command that makes the new setpoint less negative (descent).
            error_z = self.position[2] - DESIRED_ALT
            error = np.array([error_x, error_y, error_z])
            error_norm = np.linalg.norm(error)
            self.get_logger().info(f"LAND_IBVS error norm: {error_norm:.3f}")

            if error_norm < ERROR_THRESHOLD:
                self.get_logger().info("Error below threshold. Issuing LAND command.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = "DONE"
            else:
                vx = -K_LAT * error_x
                vy = -K_LAT * error_y
                vz = -K_ALT * error_z
                self.get_logger().info(f"IBVS cmd: vx:{vx:.3f}, vy:{vy:.3f}, vz:{vz:.3f}")
                dt = 0.1
                new_x = self.position[0] + vx * dt
                new_y = self.position[1] + vy * dt
                new_z = self.position[2] + vz * dt
                self.publish_trajectory_setpoint(new_x, new_y, new_z)

        elif self.state == "DONE":
            self.get_logger().info("Mission complete. Holding position.")

        self.offboard_counter += 1

    # --- Helper: Euclidean distance ---
    def _dist_to(self, target):
        return math.sqrt(sum((p - t) ** 2 for p, t in zip(self.position, target)))

    # --- Publisher Helpers ---
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        if self.state == "LAND_IBVS":
            msg.position = False
            msg.velocity = True
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
        else:
            msg.position = True
            msg.velocity = False
            msg.acceleration = False
            msg.attitude = False
            msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent.")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent.")

def main(args=None):
    rclpy.logging.get_logger("rmw_cyclonedds_cpp").set_level(LoggingSeverity.ERROR)
    rclpy.init(args=args)
    node = MarkerLanding()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
