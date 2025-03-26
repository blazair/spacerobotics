#!/usr/bin/env python3
import sys, select, tty, termios
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

msg_bindings = {
    'w': {'x':  0.5, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
    's': {'x': -0.5, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
    'a': {'x':  0.0, 'y':  0.5, 'z': 0.0, 'yaw': 0.0},
    'd': {'x':  0.0, 'y': -0.5, 'z': 0.0, 'yaw': 0.0},
    'r': {'x':  0.0, 'y': 0.0, 'z': -0.5, 'yaw': 0.0}, 
    'f': {'x':  0.0, 'y': 0.0, 'z':  0.5, 'yaw': 0.0},
    'q': {'x':  0.0, 'y': 0.0, 'z': 0.0, 'yaw':  0.1},
    'e': {'x':  0.0, 'y': 0.0, 'z': 0.0, 'yaw': -0.1},
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=1)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)

        self.timer = self.create_timer(0.05, self.publish_setpoint)

        self.setpoint = TrajectorySetpoint()
        self.setpoint.position = [0.0, 0.0, -10.0]  # Initial altitude at 10m high
        self.setpoint.yaw = 0.0

        self.start_time = self.get_clock().now()
        self.offboard_set = False

    def arm_and_offboard(self):
        # Arm vehicle
        arm_cmd = VehicleCommand()
        arm_cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        arm_cmd.param1 = 1.0
        arm_cmd.target_system = 1
        arm_cmd.target_component = 1
        arm_cmd.source_system = 1
        arm_cmd.source_component = 1
        arm_cmd.from_external = True
        arm_cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(arm_cmd)

        # Set to offboard mode
        offboard_cmd = VehicleCommand()
        offboard_cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        offboard_cmd.param1 = 1.0
        offboard_cmd.param2 = 6.0
        offboard_cmd.target_system = 1
        offboard_cmd.target_component = 1
        offboard_cmd.source_system = 1
        offboard_cmd.source_component = 1
        offboard_cmd.from_external = True
        offboard_cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_cmd.publish(offboard_cmd)

    def publish_setpoint(self):
        now = self.get_clock().now()
        if not self.offboard_set and (now - self.start_time).nanoseconds > 2e9:
            self.get_logger().info("Sending arm and offboard commands")
            self.arm_and_offboard()
            self.offboard_set = True

        offboard_mode_msg = OffboardControlMode()
        offboard_mode_msg.position = True
        offboard_mode_msg.timestamp = int(now.nanoseconds / 1000)
        self.pub_offboard_mode.publish(offboard_mode_msg)

        self.setpoint.timestamp = int(now.nanoseconds / 1000)
        self.pub_sp.publish(self.setpoint)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    print("Teleop started: WASD/RF/QE keys to move. Ctrl-C to quit.")
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':
                break
            if key in msg_bindings:
                delta = msg_bindings[key]
                node.setpoint.position[0] += delta['x']
                node.setpoint.position[1] += delta['y']
                node.setpoint.position[2] += delta['z']
                node.setpoint.yaw += delta['yaw']
                print("Setpoint:", node.setpoint.position, "Yaw:", node.setpoint.yaw)
            elif key == 'x':
                node.setpoint.position = [0.0, 0.0, -10.0]
                node.setpoint.yaw = 0.0
                print("Hover reset to 10m altitude.")
            rclpy.spin_once(node, timeout_sec=0.01)
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
