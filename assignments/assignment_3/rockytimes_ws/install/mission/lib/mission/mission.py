#!/usr/bin/env python3

"""
Combined Cylinder‐Estimation & Marker‐Landing Mission

Stage 1:  
  • ARM & Offboard automatically on startup  
  • TAKEOFF to z = –5 m, move to (15,0,–5)  
  • CIRCLE once (radius = 15 m, alt = –5 m), doing cylinder estimation each frame  
  • After one full revolution, stop and WAIT 5 s  

Stage 2:  
  • Start Marker Landing mission with IBVS landing logic (exactly as in the second node)  
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

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import (
    VehicleOdometry,
    OffboardControlMode,
    VehicleCommand,
    TrajectorySetpoint
)
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer

# --- Cylinder‐Estimation Node ---
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
        rgb_sub   = Subscriber(self, Image, '/drone/front_rgb')
        depth_sub = Subscriber(self, Image, '/drone/front_depth')
        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.image_cb)

        # state & counters
        self.state            = "ARM_OFFBOARD"
        self.offb_counter     = 0
        self.takeoff_stage    = 0
        self.one_rev_complete = False
        self.start_theta      = None

        # pose & intrinsics
        self.position = [0.0, 0.0, 0.0]
        self.fx = self.fy = None
        self.bridge = CvBridge()

        # circle params
        self.circle_radius = 15.0
        self.altitude      = -5.0
        self.circle_speed  = -0.02
        self.theta         = 0.0

        # estimation params
        self.lower_hsv = np.array([0, 0, 110])
        self.upper_hsv = np.array([180, 40, 180])
        self.min_area  = 5000

        cv2.namedWindow('Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Mask',      cv2.WINDOW_NORMAL)

        self.create_timer(0.1, self.timer_cb)

    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def caminfo_cb(self, msg):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.get_logger().info("Camera intrinsics received")
        self.destroy_subscription(self.caminfo_sub)

    def image_cb(self, rgb_msg, depth_msg):
        rgb   = self.bridge.imgmsg_to_cv2(rgb_msg,   'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        if self.fx is None:
            cv2.imshow('Detection', rgb)
            cv2.waitKey(1)
            return

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
        # offboard mode
        self.publish_offboard()

        # auto arm/offboard
        if self.offb_counter == 10:
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,        1.0, 6.0)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)

        # state machine
        if self.state == "ARM_OFFBOARD":
            self.state = "ARM_TAKEOFF"

        elif self.state == "ARM_TAKEOFF":
            if self.takeoff_stage == 0:
                # vertical
                tz = self.altitude
                self.publish_setpoint(0,0,tz,0)
                if abs(self.position[2] - tz) < 0.5:
                    self.takeoff_stage = 1
            else:
                # move out
                cx = self.circle_radius
                self.publish_setpoint(cx,0,self.altitude,0)
                if math.hypot(self.position[0]-cx, self.position[1]) < 0.5:
                    self.get_logger().info("Reached circle start → CIRCLE")
                    self.state = "CIRCLE"
                    self.start_theta = self.theta

        elif self.state == "CIRCLE":
            # one revolution?
            delta = abs(self.theta - self.start_theta)
            if delta >= 2*math.pi:
                self.get_logger().info("Completed one revolution")
                self.one_rev_complete = True
                return

            x = self.circle_radius * math.cos(self.theta)
            y = self.circle_radius * math.sin(self.theta)
            z = self.altitude
            yaw = math.atan2(-y, -x)
            self.publish_setpoint(x,y,z,yaw)
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


# --- Marker Landing Node (unchanged) ---
class MarkerLanding(Node):
    def __init__(self):
        super().__init__('marker_landing_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.offb_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.traj_pub= self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        self.odom_sub        = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)
        self.marker_pose_sub= self.create_subscription(String, '/aruco/marker_pose', self.marker_pose_cb, 10)
        self.down_mono_sub  = self.create_subscription(Image, '/drone/down_mono', self.down_mono_callback, 10)
        self.debug_pub      = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.cv_bridge      = CvBridge()

        self.state   = "WAIT"
        self.offb_counter = 0
        self.timer   = self.create_timer(0.1, self.timer_cb)
        self.position       = [0.0,0.0,-13.0]
        self.marker1_sample = None
        self.marker2_sample = None
        self.marker1_pos    = None
        self.marker2_pos    = None
        self.detect_start   = None
        self.return_start   = None
        self.latest_marker_pose = None

    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def marker_pose_cb(self, msg):
        line = msg.data.strip()
        m = re.search(r"x:\s*([-\d.]+).*y:\s*([-\d.]+).*z:\s*([-\d.]+)", line)
        if m:
            x,y,z = float(m.group(1)), float(m.group(2)), float(m.group(3))
            self.latest_marker_pose = (x,y,z)
            if self.state=="DETECT_FIRST" and self.marker1_sample is None:
                self.marker1_sample, self.detect_start = (x,y,z), time.time()
                self.get_logger().info(f"DETECT_FIRST sample: {self.marker1_sample}")
            elif self.state=="DETECT_SECOND" and self.marker2_sample is None:
                self.marker2_sample, self.detect_start = (x,y,z), time.time()
                self.get_logger().info(f"DETECT_SECOND sample: {self.marker2_sample}")

    def down_mono_callback(self, msg):
        cv_img = self.cv_bridge.imgmsg_to_cv2(msg,'bgr8')
        txt = f"Pos: x={self.position[0]:.2f}, y={self.position[1]:.2f}, z={self.position[2]:.2f}"
        cv2.putText(cv_img, txt, (10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        out = self.cv_bridge.cv2_to_imgmsg(cv_img,'bgr8')
        out.header.stamp = msg.header.stamp
        self.debug_pub.publish(out)

    def timer_cb(self):
        self.publish_offboard_control_mode()
        if self.offb_counter==5 and self.state not in ("WAIT","DONE"):
            self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1.0,6.0)
            self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1.0)

        if self.state=="WAIT":
            self.get_logger().info("Starting mission => ARM_TAKEOFF")
            self.state="ARM_TAKEOFF"

        elif self.state=="ARM_TAKEOFF":
            tgt=[0,0,-13]
            self.publish_trajectory_setpoint(*tgt)
            if self._dist_to(tgt)<0.5:
                self.get_logger().info("Arrived → GOTO_FIRST")
                self.state="GOTO_FIRST"

        elif self.state=="GOTO_FIRST":
            tgt=[0,5,-13]
            self.publish_trajectory_setpoint(*tgt)
            if self._dist_to(tgt)<0.5:
                self.marker1_sample=None
                self.detect_start=time.time()
                self.get_logger().info("Arrived → DETECT_FIRST")
                self.state="DETECT_FIRST"

        elif self.state=="DETECT_FIRST":
            tgt=[0,5,-13]; self.publish_trajectory_setpoint(*tgt)
            if time.time()-self.detect_start>=4.0 and self.marker1_sample:
                self.marker1_pos=self.marker1_sample
                self.get_logger().info(f"Chosen1: {self.marker1_pos}")
            if time.time()-self.detect_start>=5.0:
                self.return_start=time.time()
                self.state="RETURN_CENTER_FIRST"
                self.get_logger().info("DETECT_FIRST done")

        elif self.state=="RETURN_CENTER_FIRST":
            tgt=[0,0,-13]; self.publish_trajectory_setpoint(*tgt)
            if time.time()-self.return_start>=2.0:
                self.get_logger().info("→ GOTO_SECOND")
                self.state="GOTO_SECOND"

        elif self.state=="GOTO_SECOND":
            tgt=[0,-5,-13]; self.publish_trajectory_setpoint(*tgt)
            if self._dist_to(tgt)<0.5:
                self.detect_start=time.time()
                self.get_logger().info("→ DETECT_SECOND")
                self.state="DETECT_SECOND"

        elif self.state=="DETECT_SECOND":
            tgt=[0,-5,-13]; self.publish_trajectory_setpoint(*tgt)
            if time.time()-self.detect_start>=4.0 and self.marker2_sample:
                self.marker2_pos=self.marker2_sample
                self.get_logger().info(f"Chosen2: {self.marker2_pos}")
            if time.time()-self.detect_start>=5.0:
                self.return_start=time.time()
                self.state="RETURN_CENTER_SECOND"
                self.get_logger().info("DETECT_SECOND done")

        elif self.state=="RETURN_CENTER_SECOND":
            tgt=[0,0,-13]; self.publish_trajectory_setpoint(*tgt)
            if time.time()-self.return_start>=2.0:
                self.get_logger().info("→ COMPARE")
                self.state="COMPARE"

        elif self.state=="COMPARE":
            if self.marker1_pos[2] < self.marker2_pos[2]:
                self.chosen_direction_y=5.0
                self.get_logger().info(f"Choose1")
            else:
                self.chosen_direction_y=-5.0
                self.get_logger().info(f"Choose2")
            self.state="APPROACH"

        elif self.state=="APPROACH":
            tgt=[0,self.chosen_direction_y,-13]; self.publish_trajectory_setpoint(*tgt)
            if self._dist_to(tgt)<0.5:
                self.get_logger().info("→ LAND_IBVS")
                self.state="LAND_IBVS"

        elif self.state=="LAND_IBVS":
            if not self.latest_marker_pose:
                self.publish_trajectory_setpoint(*self.position); return
            mx,my,_=self.latest_marker_pose
            ez = self.position[2] - 0.0
            ex,ey = mx,my
            norm = math.sqrt(ex**2+ey**2+ez**2)
            if norm<0.1:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state="DONE"
            else:
                vx = -0.1*ex; vy = -0.1*ey; vz = -0.05*ez
                nt = 0.1
                self.publish_trajectory_setpoint(
                    self.position[0]+vx*nt,
                    self.position[1]+vy*nt,
                    self.position[2]+vz*nt,
                    0.0
                )

        elif self.state=="DONE":
            self.get_logger().info("Mission complete")

        self.offb_counter+=1

    def publish_offboard_control_mode(self):
        m=OffboardControlMode()
        m.position = (self.state!="LAND_IBVS")
        m.velocity = (self.state=="LAND_IBVS")
        m.timestamp = self.get_clock().now().nanoseconds//1000
        self.offb_pub.publish(m)

    def publish_trajectory_setpoint(self,x,y,z,yaw=0.0):
        sp=TrajectorySetpoint()
        sp.position=[x,y,z]; sp.yaw=yaw
        sp.timestamp=self.get_clock().now().nanoseconds//1000
        self.traj_pub.publish(sp)

    def send_cmd(self, cmd,p1=0.0,p2=0.0):
        m=VehicleCommand()
        m.command=cmd; m.param1=p1; m.param2=p2
        m.target_system=1; m.target_component=1
        m.source_system=1; m.source_component=1
        m.from_external=True
        m.timestamp=self.get_clock().now().nanoseconds//1000
        self.cmd_pub.publish(m)

    def _dist_to(self, tgt):
        return math.sqrt(sum((p - t)**2 for p, t in zip(self.position, tgt)))


def main(args=None):
    rclpy.logging.get_logger("rmw_cyclonedds_cpp").set_level(LoggingSeverity.ERROR)
    rclpy.init(args=args)

    # Stage 1: run cylinder estimation until one revolution completes
    est = AutoCylinderEstimate()
    while rclpy.ok() and not est.one_rev_complete:
        rclpy.spin_once(est, timeout_sec=0.1)
    est.destroy_node()

    # 5 s transition
    time.sleep(5)

    # Stage 2: marker landing
    land = MarkerLanding()
    try:
        rclpy.spin(land)
    except KeyboardInterrupt:
        pass
    finally:
        land.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
