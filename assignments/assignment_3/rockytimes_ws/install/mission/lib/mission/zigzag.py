#!/usr/bin/env python3
"""
FlightControlNode (Zigzag Flight Pattern)

Flight sequence:
  1. TAKEOFF: Drone takes off from origin and climbs to 5 m altitude.
  2. MOVE_5_1: Fly to (15, 0, -5) (i.e. 15 m ahead at 5 m).
  3. HOLD_5_1: Hold for 5 s.
  4. MOVE_5_2: Fly to (-15, 0, -5) (i.e. opposite side at 5 m).
  5. HOLD_5_2: Hold for 5 s.
  6. (Repeat at next altitudes, e.g., 7 m, 9 m, etc.)
  7. RETURN: Fly back to origin at the last altitude.
  8. CLIMB_20: Climb from last altitude to 20 m.
  9. HOVER: Hover at 20 m.
  
Note: This node sends offboard flight setpoints only. Adjust waypoints and timing as needed.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint
from std_msgs.msg import Float64
import math
import time

# Flight parameters (adjust as needed)
ALTITUDES = [5.0, 7.0, 9.0, 11.0]  # Altitudes at which to perform the measurement pattern
HOLD_TIME = 5.0                   # Seconds to hold at each measurement point
# Waypoints for each altitude: (x, y) positions (z is altitude, negative)
WAYPOINTS = {
    "MOVE_POS": (15.0, 0.0),
    "MOVE_NEG": (-15.0, 0.0)
}

class FlightControlNode(Node):
    def __init__(self):
        super().__init__('flight_control_node')
        
        qos_profile = rclpy.qos.QoSProfile(depth=1)
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.gimbal_yaw_pub = self.create_publisher(Float64, '/model/x500_gimbal_0/command/gimbal_yaw', 10)
        
        self.odometry_sub = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        self.vehicle_odometry = None
        
        # Flight state management:
        self.alt_index = 0  # index into ALTITUDES
        self.wp_index = 0   # index into sub-waypoints for current altitude
        self.state = "TAKEOFF"  # initial state
        self.state_start_time = time.time()
        self.finished = False
        
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz update
        self.offboard_counter = 0
        
        self.get_logger().info("FlightControlNode initialized.")

    def odometry_callback(self, msg):
        self.vehicle_odometry = msg

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
        self.get_logger().info("Arm command sent.")

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
        self.get_logger().info("Offboard mode command sent.")

    def in_position(self, target_pos, target_alt):
        if self.vehicle_odometry is None:
            return False
        pos = self.vehicle_odometry.position
        dx = pos[0] - target_pos[0]
        dy = pos[1] - target_pos[1]
        # In PX4, z is negative.
        dalt = -pos[2] - target_alt
        dist = math.sqrt(dx*dx + dy*dy + dalt*dalt)
        return dist < 1.0

    def timer_callback(self):
        if self.finished:
            return
        
        # First few iterations: send offboard and arm.
        if self.offboard_counter < 50:
            self.publish_offboard()
            self.offboard_counter += 1
            if self.offboard_counter == 10:
                self.engage_offboard()
                self.arm()
            return

        # Flight state machine:
        if self.state == "TAKEOFF":
            target_alt = ALTITUDES[0]
            self.publish_setpoint(0.0, 0.0, -target_alt, 0.0)
            if self.vehicle_odometry and self.in_position((0.0, 0.0), target_alt):
                self.get_logger().info(f"Takeoff complete to {target_alt} m.")
                self.state = "MOVE_POS"
                self.state_start_time = time.time()
        elif self.state in ["MOVE_POS", "MOVE_NEG"]:
            target_alt = ALTITUDES[self.alt_index]
            # Determine target x,y from waypoint
            wp = WAYPOINTS[self.state]
            self.publish_setpoint(wp[0], wp[1], -target_alt, math.atan2(-wp[1], -wp[0]))
            if self.vehicle_odometry and self.in_position(wp, target_alt):
                self.get_logger().info(f"Reached {self.state} at altitude {target_alt} m.")
                # After MOVE, transition to HOLD.
                self.state = "HOLD_" + self.state.split("_")[1]
                self.state_start_time = time.time()
        elif self.state.startswith("HOLD"):
            target_alt = ALTITUDES[self.alt_index]
            wp = WAYPOINTS["MOVE_" + self.state.split("_")[1]]
            self.publish_setpoint(wp[0], wp[1], -target_alt, math.atan2(-wp[1], -wp[0]))
            if time.time() - self.state_start_time >= HOLD_TIME:
                self.get_logger().info(f"Holding at {self.state} complete.")
                # Transition to next waypoint
                if self.state == "HOLD_POS":
                    self.state = "MOVE_NEG"
                elif self.state == "HOLD_NEG":
                    # Completed one altitude measurement pattern.
                    self.alt_index += 1
                    if self.alt_index < len(ALTITUDES):
                        self.state = "CLIMB"
                    else:
                        self.state = "RETURN"
                self.state_start_time = time.time()
        elif self.state == "CLIMB":
            # Climb from previous altitude to next altitude.
            target_alt = ALTITUDES[self.alt_index]
            # Use current horizontal position.
            if self.vehicle_odometry:
                cur_x = self.vehicle_odometry.position[0]
                cur_y = self.vehicle_odometry.position[1]
            else:
                cur_x, cur_y = 0.0, 0.0
            self.publish_setpoint(cur_x, cur_y, -target_alt, math.atan2(-cur_y, -cur_x))
            if self.vehicle_odometry and self.in_position((cur_x, cur_y), target_alt):
                self.get_logger().info(f"Climb to {target_alt} m complete.")
                self.state = "MOVE_POS"
                self.state_start_time = time.time()
        elif self.state == "RETURN":
            # Return to origin at the last altitude.
            target_alt = ALTITUDES[-1]
            self.publish_setpoint(0.0, 0.0, -target_alt, 0.0)
            if self.vehicle_odometry and self.in_position((0.0, 0.0), target_alt):
                self.get_logger().info("Return reached. Transitioning to CLIMB_20.")
                self.state = "CLIMB_20"
                self.state_start_time = time.time()
        elif self.state == "CLIMB_20":
            self.publish_setpoint(0.0, 0.0, -20.0, 0.0)
            if self.vehicle_odometry and self.in_position((0.0, 0.0), 20.0):
                self.get_logger().info("Reached 20 m altitude. Transitioning to HOVER.")
                self.state = "HOVER"
                self.state_start_time = time.time()
        elif self.state == "HOVER":
            self.publish_setpoint(0.0, 0.0, -20.0, 0.0)
            self.get_logger().info("Hovering at 20 m. Flight complete.")
            self.finished = True
        # Publish offboard regularly.
        self.publish_offboard()

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

def main():
    rclpy.init()
    node = FlightControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Flight interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
