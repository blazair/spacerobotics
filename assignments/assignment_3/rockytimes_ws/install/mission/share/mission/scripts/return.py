#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint
import math

class HoverAndMove(Node):
    def __init__(self):
        super().__init__('hover_and_move')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos_profile)

        # Internal state
        self.offboard_setpoint_counter = 0
        self.state = "HOVER"  # States: HOVER, MOVE, LAND, COMPLETED
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.position = [0.0, 0.0, 0.0]

        # Target hover and land positions
        self.hover_z = -20.0
        self.land_target = [0.0, -5.0, -13.0]

    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    def timer_callback(self):
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        self.publish_offboard_control_mode()

        if self.state == "HOVER":
            self.publish_trajectory_setpoint(x=0.0, y=0.0, z=self.hover_z)
            if abs(self.position[2] - self.hover_z) < 0.5:
                self.get_logger().info("Reached hover height. Proceeding to MOVE.")
                self.state = "MOVE"

        elif self.state == "MOVE":
            self.publish_trajectory_setpoint(
                x=self.land_target[0], y=self.land_target[1], z=self.land_target[2])
            distance = math.sqrt(
                (self.position[0] - self.land_target[0])**2 +
                (self.position[1] - self.land_target[1])**2 +
                (self.position[2] - self.land_target[2])**2)
            if distance < 0.5:
                self.get_logger().info("Reached landing point. Initiating LAND.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = "LAND"

        elif self.state == "LAND":
            # Wait until vehicle touches down (altitude close to 0)
            if abs(self.position[2]) < 0.3:
                self.get_logger().info("Landing complete. Disarming.")
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
                self.state = "COMPLETED"

        elif self.state == "COMPLETED":
            self.get_logger().info("Mission completed. Standing by.")

        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
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
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

def main():
    rclpy.init()
    node = HoverAndMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()