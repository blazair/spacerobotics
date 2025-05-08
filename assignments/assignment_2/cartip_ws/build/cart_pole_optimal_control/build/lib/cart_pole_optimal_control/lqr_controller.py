#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg

class CartPoleLQRController(Node):
    def __init__(self):
        super().__init__('lqr_controller')

        # System parameters
        self.M = 1.0
        self.m = 1.0
        self.L = 1.0
        self.g = 9.81

        # Declare separate float parameters for Q and R
        self.declare_parameter('Q0', 1.0)
        self.declare_parameter('Q1', 1.0)
        self.declare_parameter('Q2', 10.0)
        self.declare_parameter('Q3', 10.0)
        self.declare_parameter('R0', 0.2)  # Single float for the R penalty

        # Initialize state-space matrices
        self.initialize_state_space_matrices()

        # Default zero gain
        self.K = np.zeros((1, 4), dtype=np.float64)

        # Update LQR parameters once at startup
        self.update_lqr_parameters()

        # State estimate
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.state_initialized = False

        # Publishers / Subscribers
        self.cart_cmd_pub = self.create_publisher(
            Float64, 
            '/model/cart_pole/joint/cart_to_base/cmd_force',
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            10
        )

        # Control loop timer
        self.timer = self.create_timer(0.01, self.control_loop)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info('Cart-Pole LQR Controller initialized (no startup delay).')

    def initialize_state_space_matrices(self):
        """Initialize the state-space representation of the cart-pole system."""
        self.A = np.array([
            [0, 1, 0, 0],
            [0, 0, (self.m * self.g) / self.M, 0],
            [0, 0, 0, 1],
            [0, 0, ((self.M + self.m) * self.g) / (self.M * self.L), 0]
        ], dtype=np.float64)

        self.B = np.array([
            [0],
            [1 / self.M],
            [0],
            [-1 / (self.M * self.L)]
        ], dtype=np.float64)

    def update_lqr_parameters(self):
        """Construct Q, R from Q0..Q3, R0, and compute the new LQR gain."""
        try:
            q0 = self.get_parameter('Q0').value
            q1 = self.get_parameter('Q1').value
            q2 = self.get_parameter('Q2').value
            q3 = self.get_parameter('Q3').value
            r0 = self.get_parameter('R0').value

            # Build the diagonal Q matrix
            q_values = np.array([q0, q1, q2, q3], dtype=np.float64)
            self.Q = np.diag(q_values)

            # R is a single float
            self.R = np.array([[r0]], dtype=np.float64)

            # Log them with full precision
            self.get_logger().info(
                "Updating LQR parameters with Q: "
                f"[{q0:.4f}, {q1:.4f}, {q2:.4f}, {q3:.4f}] "
                f"and R: [{r0:.4f}]"
            )

            self.K = self.compute_lqr_gain()
            self.get_logger().info(
                "Updated LQR Gain Matrix: "
                f"{np.array2string(self.K, formatter={'float_kind': lambda x: f'{x:.4f}'})}"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to update LQR parameters: {e}")

    def compute_lqr_gain(self):
        """Compute the LQR gain matrix K = R^-1 * B^T * P, with P solving the CARE."""
        try:
            A_c = np.ascontiguousarray(self.A)
            B_c = np.ascontiguousarray(self.B)
            Q_c = np.ascontiguousarray(self.Q)
            R_c = np.ascontiguousarray(self.R)

            P = linalg.solve_continuous_are(A_c, B_c, Q_c, R_c)
            K = np.linalg.inv(R_c) @ B_c.T @ P
            return K
        except Exception as e:
            self.get_logger().error(f"LQR computation failed: {e}")
            return np.zeros((1, 4), dtype=np.float64)

    def parameter_callback(self, params):
        """Accumulate changes; update LQR once at the end."""
        changed = False
        for param in params:
            if param.name in ["Q0", "Q1", "Q2", "Q3", "R0"]:
                changed = True
        if changed:
            self.update_lqr_parameters()

        return SetParametersResult(successful=True)

    def joint_state_callback(self, msg):
        """Update cart-pole state from the JointState message."""
        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            self.x = np.array([
                [msg.position[cart_idx]],
                [msg.velocity[cart_idx]],
                [msg.position[pole_idx]],
                [msg.velocity[pole_idx]]
            ], dtype=np.float64)

            if not self.state_initialized:
                self.state_initialized = True
                self.get_logger().info("Received initial joint states. Controller active.")

        except (ValueError, IndexError):
            self.get_logger().warn('Failed to process joint states')

    def control_loop(self):
        """Periodic control force computation: u = -K x."""
        if not self.state_initialized:
            return

        u = -self.K @ self.x
        force_cmd = float(u[0])
        msg = Float64()
        msg.data = force_cmd
        self.cart_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = CartPoleLQRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
