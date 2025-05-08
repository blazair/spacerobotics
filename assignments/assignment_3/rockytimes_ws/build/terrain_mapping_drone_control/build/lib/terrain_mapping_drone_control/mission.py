#!/usr/bin/env python3
import sys, select, tty, termios
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Instructions for keyboard:
#   WASD: Move Forward/Backward and Left/Right (adjust as needed)
#   R: Increase altitude (go up)  --> In NED, increasing altitude means more negative Z
#   F: Decrease altitude (go down)
#   Q/E: Rotate (yaw left/right)
#   X: Reset to hover (stop movement)

msg_bindings = {
    'w': {'x':  0.5, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
    's': {'x': -0.5, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},
    'a': {'x':  0.0, 'y':  0.5, 'z': 0.0, 'yaw': 0.0},
    'd': {'x':  0.0, 'y': -0.5, 'z': 0.0, 'yaw': 0.0},
    'r': {'x':  0.0, 'y': 0.0, 'z': -0.5, 'yaw': 0.0},  # in NED, going up => more negative z
    'f': {'x':  0.0, 'y': 0.0, 'z':  0.5, 'yaw': 0.0},  # going down => z becomes less negative
    'q': {'x':  0.0, 'y': 0.0, 'z': 0.0, 'yaw':  0.1},
    'e': {'x':  0.0, 'y': 0.0, 'z': 0.0, 'yaw': -0.1},
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
    return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.timer = self.create_timer(0.05, self.publish_setpoint)  # 20Hz publishing rate
        # Start with a neutral setpoint
        self.setpoint = TrajectorySetpoint()
        self.setpoint.position = [0.0, 0.0, 0.0]
        self.setpoint.yaw = 0.0

    def publish_setpoint(self):
        self.setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub.publish(self.setpoint)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = TeleopNode()
    print("Teleop Node Started")
    print("Use WASD to move horizontally, R/F to change altitude, Q/E to rotate, X to reset, Ctrl-C to quit.")
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':  # Ctrl-C
                break
            elif key in msg_bindings:
                # Update setpoint relative to current value
                delta = msg_bindings[key]
                node.setpoint.position[0] += delta['x']
                node.setpoint.position[1] += delta['y']
                node.setpoint.position[2] += delta['z']
                node.setpoint.yaw += delta['yaw']
                print("Setpoint:", node.setpoint.position, "Yaw:", node.setpoint.yaw)
            elif key == 'x':
                # Reset to hover (set position to current or zero as needed)
                node.setpoint.position = [0.0, 0.0, 0.0]
                node.setpoint.yaw = 0.0
                print("Reset to hover.")
    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
