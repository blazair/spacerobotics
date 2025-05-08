#!/usr/bin/env python3

"""
Auto‐Takeoff Cylinder Estimation Mission

This node will:
  1. ARM & switch to OFFBOARD automatically on startup.
  2. TAKEOFF to z = –5 m (vertical), then fly to (15,0,–5).
  3. CIRCLE at radius = 15 m, altitude = –5 m.
Meanwhile:
  • Continuously subscribe to /drone/front_rgb and /drone/front_depth.
  • Segment, find the largest contour, compute real‑world width/height.
  • Draw bounding box + size text and log each detection.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint
)
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer


class AutoCylinderEstimate(Node):
    def __init__(self):
        super().__init__('auto_cylinder_estimate')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offb_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint,   '/fmu/in/trajectory_setpoint',   qos)
        self.cmd_pub  = self.create_publisher(VehicleCommand,       '/fmu/in/vehicle_command',       qos)

        # Subscribers
        self.odom_sub    = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry',    self.odom_cb,    qos)
        self.caminfo_sub = self.create_subscription(CameraInfo,     '/drone/front_depth/camera_info', self.caminfo_cb, 10)

        # RGB+Depth sync
        rgb_sub   = Subscriber(self, Image, '/drone/front_rgb')
        depth_sub = Subscriber(self, Image, '/drone/front_depth')
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.image_cb)

        # State
        self.state = "ARM_OFFBOARD"   # first thing: arm & offboard
        self.offb_counter = 0

        # Takeoff stages: 0 = vertical, 1 = move out
        self.takeoff_stage = 0

        # Drone pose
        self.position = [0.0, 0.0, 0.0]

        # Camera intrinsics
        self.fx = self.fy = None
        self.bridge = CvBridge()

        # Circle params
        self.circle_radius = 15.0
        self.altitude      = -5.0
        self.circle_speed  = -0.02
        self.theta         = 0.0

        # Estimation params
        self.lower_hsv = np.array([0, 0, 110])
        self.upper_hsv = np.array([180, 40, 180])
        self.min_area  = 5000

        # OpenCV windows
        cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Mask',      cv2.WINDOW_NORMAL)

        # Run at 10 Hz
        self.create_timer(0.1, self.timer_cb)

    def odom_cb(self, msg: VehicleOdometry):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def caminfo_cb(self, msg: CameraInfo):
        # grab intrinsics once
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.get_logger().info("Camera intrinsics received")
        self.destroy_subscription(self.caminfo_sub)

    def image_cb(self, rgb_msg: Image, depth_msg: Image):
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        # need intrinsics
        if self.fx is None:
            cv2.imshow('Detection', rgb)
            cv2.waitKey(1)
            return

        # mask by color + depth
        hsv        = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        cm         = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv) > 0
        dm         = (depth > 1.0) & (depth < 30.0)
        mask       = (cm & dm).astype(np.uint8)*255
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))

        cnts, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        large   = [c for c in cnts if cv2.contourArea(c) > self.min_area]

        overlay = rgb.copy()
        if large:
            c = max(large, key=cv2.contourArea)
            x,y,w,h = cv2.boundingRect(c)
            roi = depth[y:y+h, x:x+w]
            vals = roi[np.isfinite(roi)]
            if vals.size>0:
                Z   = float(np.median(vals))
                w_m = (w * Z)/self.fx
                h_m = (h * Z)/self.fy

                cv2.rectangle(overlay, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(
                    overlay,
                    f"{w_m:.2f}m x {h_m:.2f}m",
                    (x, y-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2
                )
                self.get_logger().info(f"Detected cylinder: W={w_m:.2f} H={h_m:.2f} D={Z:.2f}")

        cv2.imshow('Detection', overlay)
        cv2.imshow('Mask', mask_clean)
        cv2.waitKey(1)

    def timer_cb(self):
        # publish offboard control mode
        self.publish_offboard()

        # auto arm & offboard
        if self.offb_counter == 10:
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,        1.0, 6.0)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)

        # state machine
        if self.state == "ARM_OFFBOARD":
            # immediately go to takeoff stage
            self.state = "ARM_TAKEOFF"

        elif self.state == "ARM_TAKEOFF":
            if self.takeoff_stage == 0:
                # vertical takeoff
                target_z = self.altitude
                self.publish_setpoint(0, 0, target_z, 0)
                if abs(self.position[2] - target_z) < 0.5:
                    self.takeoff_stage = 1
            else:
                # move to circle start
                self.publish_setpoint(self.circle_radius, 0, self.altitude, 0)
                if math.hypot(self.position[0]-self.circle_radius,
                              self.position[1]) < 0.5:
                    self.get_logger().info("Reached circle start → CIRCLE")
                    self.state = "CIRCLE"

        elif self.state == "CIRCLE":
            # perform circle
            x = self.circle_radius * math.cos(self.theta)
            y = self.circle_radius * math.sin(self.theta)
            z = self.altitude
            yaw = math.atan2(-y, -x)  # face center
            self.publish_setpoint(x, y, z, yaw)
            self.theta += self.circle_speed

        self.offb_counter += 1

    def publish_offboard(self):
        m = OffboardControlMode()
        m.position  = True
        m.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offb_pub.publish(m)

    def publish_setpoint(self, x, y, z, yaw):
        sp = TrajectorySetpoint()
        sp.position  = [float(x), float(y), float(z)]
        sp.yaw       = float(yaw)
        sp.timestamp = self.get_clock().now().nanoseconds // 1000
        self.traj_pub.publish(sp)

    def send_cmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command         = cmd
        m.param1          = float(p1)
        m.param2          = float(p2)
        m.target_system   = 1
        m.target_component= 1
        m.source_system   = 1
        m.source_component= 1
        m.from_external   = True
        m.timestamp       = self.get_clock().now().nanoseconds // 1000
        self.cmd_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = AutoCylinderEstimate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
