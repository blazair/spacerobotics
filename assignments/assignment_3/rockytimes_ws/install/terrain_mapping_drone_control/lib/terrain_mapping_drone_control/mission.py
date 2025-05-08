#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class SimpleTakeoffNode(Node):
    def __init__(self):
        super().__init__('simple_takeoff_node')

        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)

        # Subscriber (optional)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos
        )

        self.offboard_counter = 0
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        current_height = -msg.position[2]
        self.get_logger().info(f"Current height: {current_height:.2f}m")

    def control_loop(self):
        if self.offboard_counter == 10:
            self.get_logger().info("Sending Offboard Mode and Arm Commands")
            self.set_mode()
            self.arm()

        self.publish_offboard_control_mode()
        self.publish_takeoff_setpoint()

        self.offboard_counter += 1

    def set_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0  # PX4 Offboard mode
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(msg)

    def publish_takeoff_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -10.0]  # NED: -10 meters
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(msg)

def main():
    rclpy.init()
    node = SimpleTakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
