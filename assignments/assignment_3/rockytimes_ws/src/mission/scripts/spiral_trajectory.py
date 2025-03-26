#!/usr/bin/env python3
"""
Spiral to Hover Node

Flight sequence:
  1. TAKEOFF: The drone takes off to the first altitude (2 m).
  2. LOOP: It performs circular loops at altitudes: 2, 4, 6, 8, 10, and 12 m.
  3. RETURN: The drone returns to the origin (x=0, y=0) at 12 m.
  4. CLIMB: It then climbs from 12 m to 20 m while staying at the origin.
  5. HOVER: Finally, the drone hovers at 20 m altitude.
  
Note: This node only handles flight until a stable hover at 20 m.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from std_msgs.msg import Float64

class SpiralToHover(Node):
    def __init__(self):
        super().__init__('spiral_to_hover')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Optional gimbal control publishers
        self.gimbal_pitch_pub = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_pitch', 10)
        self.gimbal_roll_pub = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_roll', 10)
        self.gimbal_yaw_pub = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_yaw', 10)
        
        # Subscribers
        self.vehicle_odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        
        self.start_time = time.time()
        self.loop_start_time = None

        # Flight parameters
        self.altitude_steps = [2.0, 4.0, 6.0, 8.0, 10.0, 12.0]  # altitudes for loops (meters)
        self.current_loop = 0
        self.loop_duration = 20.0  # seconds per loop
        self.RADIUS = 15.0         # circle radius (meters)
        self.climb_altitude = 20.0  # target altitude after climb (meters)
        
        # State machine: "TAKEOFF" → "LOOP" → "RETURN" → "CLIMB" → "HOVER"
        self.state = "TAKEOFF"
        
        self.create_timer(0.05, self.timer_callback)

    def vehicle_odometry_callback(self, msg):
        self.vehicle_odometry = msg
        self.update_gimbal()

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def update_gimbal(self):
        try:
            x = self.vehicle_odometry.position[0]
            y = self.vehicle_odometry.position[1]
            yaw = math.atan2(-y, -x)
        except Exception:
            yaw = 0.0
        msg = Float64()
        msg.data = yaw
        self.gimbal_yaw_pub.publish(msg)
        msg.data = 0.0
        self.gimbal_pitch_pub.publish(msg)
        self.gimbal_roll_pub.publish(msg)

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

    def is_at_altitude(self, target):
        try:
            current = -self.vehicle_odometry.position[2]
            return abs(current - target) < 0.5
        except Exception:
            return False

    def calculate_circle(self, t, target_alt):
        omega = 2 * math.pi / self.loop_duration
        x = self.RADIUS * math.cos(omega * t)
        y = self.RADIUS * math.sin(omega * t)
        z = -target_alt
        yaw = math.atan2(-y, -x)
        return x, y, z, yaw

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard()
            self.arm()
            self.start_time = time.time()
            self.loop_start_time = self.start_time

        self.publish_offboard()

        if self.state == "TAKEOFF":
            target = self.altitude_steps[0]
            if not self.is_at_altitude(target):
                self.publish_setpoint(0.0, 0.0, -target, 0.0)
                try:
                    alt = -self.vehicle_odometry.position[2]
                except Exception:
                    alt = 0.0
                self.get_logger().info(f"Taking off... Altitude: {alt:.2f} m (target: {target} m)")
            else:
                self.state = "LOOP"
                self.loop_start_time = time.time()
                self.current_loop = 0
                self.get_logger().info("Takeoff complete. Starting loops.")
        elif self.state == "LOOP":
            t = time.time() - self.loop_start_time
            target_alt = self.altitude_steps[self.current_loop]
            x, y, z, yaw = self.calculate_circle(t, target_alt)
            self.publish_setpoint(x, y, z, yaw)
            self.get_logger().info(f"Loop {self.current_loop+1}: pos=({x:.2f}, {y:.2f}), alt={target_alt:.2f} m, yaw={yaw:.2f}")
            if t > self.loop_duration:
                self.current_loop += 1
                self.loop_start_time = time.time()
                if self.current_loop < len(self.altitude_steps):
                    self.get_logger().info(f"Starting loop {self.current_loop+1} at altitude {self.altitude_steps[self.current_loop]:.2f} m.")
                else:
                    self.get_logger().info("Loops complete. Transitioning to RETURN state.")
                    self.state = "RETURN"
        elif self.state == "RETURN":
            target_alt = self.altitude_steps[-1]
            self.publish_setpoint(0.0, 0.0, -target_alt, 0.0)
            try:
                cur_x = self.vehicle_odometry.position[0]
                cur_y = self.vehicle_odometry.position[1]
            except Exception:
                cur_x, cur_y = 0.0, 0.0
            dist = math.sqrt(cur_x**2 + cur_y**2)
            self.get_logger().info(f"Returning to origin: distance={dist:.2f} m")
            if dist < 1.0:
                self.get_logger().info("Origin reached. Transitioning to CLIMB state.")
                self.state = "CLIMB"
        elif self.state == "CLIMB":
            self.publish_setpoint(0.0, 0.0, -self.climb_altitude, 0.0)
            if self.is_at_altitude(self.climb_altitude):
                self.get_logger().info(f"Reached climb altitude of {self.climb_altitude} m. Transitioning to HOVER state.")
                self.state = "HOVER"
            else:
                self.get_logger().info("Climbing to 20 m...")
        elif self.state == "HOVER":
            # Maintain hover at the origin at 20 m altitude.
            self.publish_setpoint(0.0, 0.0, -self.climb_altitude, 0.0)
            self.get_logger().info("Hovering at 20 m altitude.")
        
        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = SpiralToHover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Flight interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
