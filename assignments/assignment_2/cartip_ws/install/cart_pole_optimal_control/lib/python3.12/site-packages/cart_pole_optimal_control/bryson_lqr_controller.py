#!/home/blazar/envs/ros2_rl/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import csv

# Set the output directory (change as needed)
OUTPUT_DIR = "/home/blazar/workspaces/cartip_ws/src/cart_pole_optimal_control/graphs/proper"

class BrysonLQRController(Node):
    def __init__(self):
        super().__init__('bryson_lqr_controller')

        # System parameters
        self.M = 1.0   # Cart mass (kg)
        self.m = 1.0   # Pole mass (kg)
        self.L = 1.0   # Pole length (m)
        self.g = 9.81  # Gravity (m/s^2)

        # Maximum allowed values (for Bryson's rule)
        self.x_max = 0.3         # Max cart displacement (m)
        self.xdot_max = 2.0      # Max cart velocity (m/s)
        self.theta_max = 0.2     # Max pole angle (rad)
        self.thetadot_max = 2.0  # Max pole angular velocity (rad/s)
        self.max_force = 15.0    # Max control force (N)

        self.initialize_state_space_matrices()
        self.compute_bryson_weights()
        self.Q_orig = self.Q.copy()
        self.R_orig = self.R.copy()
        self.K = self.compute_lqr_gain()

        # Initialize state and log arrays
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.state_initialized = False
        self.t_log = []   # time stamps
        self.x_log = []   # state vectors
        self.u_log = []   # control inputs
        self.q_log = []   # Q diagonal values at each step
        self.r_log = []   # R values at each step
        self.q_r_update_times = []  # times when update_q_r is called
        self.q_r_values = []        # (q_multiplier, r_multiplier) values

        self.cart_cmd_pub = self.create_publisher(
            Float64, '/model/cart_pole/joint/cart_to_base/cmd_force', 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback, 10)

        # Control loop timer (100 Hz) and update timer (5 seconds)
        self.timer = self.create_timer(0.01, self.control_loop)
        self.update_timer = self.create_timer(5.0, self.update_q_r)

        self.get_logger().info('Bryson LQR Controller initialized.')

    def initialize_state_space_matrices(self):
        # Define the state-space A and B matrices for the cart-pole system
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

    def compute_bryson_weights(self):
        # Compute Q and R using Bryson's rule
        q0 = 1.0 / (self.x_max ** 2)
        q1 = 1.0 / (self.xdot_max ** 2)
        q2 = 1.0 / (self.theta_max ** 2)
        q3 = 1.0 / (self.thetadot_max ** 2)
        self.Q = np.diag([q0, q1, q2, q3])
        r_val = 1.0 / (self.max_force ** 2)
        self.R = np.array([[r_val]], dtype=np.float64)
        self.get_logger().info(
            f"Bryson Weights: Q = diag([{q0:.4f}, {q1:.4f}, {q2:.4f}, {q3:.4f}]), R = {r_val:.6f}"
        )

    def compute_lqr_gain(self):
        # Solve the Riccati equation and compute gain K = R⁻¹ B^T P
        try:
            P = linalg.solve_continuous_are(self.A, self.B, self.Q, self.R)
            K = np.linalg.inv(self.R) @ self.B.T @ P
            self.get_logger().info(f"Computed LQR Gain: {K}")
            return K
        except Exception as e:
            self.get_logger().error(f"LQR computation failed: {e}")
            return np.zeros((1, 4), dtype=np.float64)

    def update_q_r(self):
        # Variant A: update multipliers each call (dynamic update)
        self.q_multiplier = getattr(self, 'q_multiplier', 1.0) * 1.5
        self.r_multiplier = getattr(self, 'r_multiplier', 1.0) * 0.8
        self.Q = self.q_multiplier * self.Q_orig
        self.R = self.r_multiplier * self.R_orig
        self.K = self.compute_lqr_gain()
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.q_r_update_times.append(current_time)
        self.q_r_values.append((self.q_multiplier, self.r_multiplier))
        self.get_logger().info(
            f"Variant A update at t={current_time:.2f}s: q_multiplier={self.q_multiplier:.2f}, r_multiplier={self.r_multiplier:.2f}"
        )

    def joint_state_callback(self, msg):
        # Update state x from joint state message
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
                self.get_logger().info("Initial joint states received. Controller active.")
        except Exception as e:
            self.get_logger().warn(f"Joint state callback error: {e}")

    def control_loop(self):
        # Compute control input u = -Kx and log data
        if not self.state_initialized:
            return
        u = -self.K @ self.x
        force_cmd = float(u[0])
        msg = Float64()
        msg.data = force_cmd
        self.cart_cmd_pub.publish(msg)
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.t_log.append(current_time)
        self.x_log.append(self.x.flatten())
        self.u_log.append(force_cmd)
        self.q_log.append(np.diag(self.Q))
        self.r_log.append(self.R[0, 0])

    def save_plots_and_csv(self):
        # Save plots and CSV data to OUTPUT_DIR
        if not self.t_log:
            self.get_logger().warn("No data logged.")
            return
        if not os.path.exists(OUTPUT_DIR):
            os.makedirs(OUTPUT_DIR)

        t = np.array(self.t_log)
        x = np.array(self.x_log)
        u = np.array(self.u_log)
        q_arr = np.array(self.q_log)
        r_arr = np.array(self.r_log)

        # Plot 1: State Evolution (4 subplots)
        fig1, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
        labels = ["Cart Position (m)", "Cart Velocity (m/s)", "Pole Angle (rad)", "Pole Angular Velocity (rad/s)"]
        for i in range(4):
            axs[i].plot(t, x[:, i], label=labels[i])
            axs[i].set_ylabel(labels[i])
            axs[i].legend()
        axs[-1].set_xlabel("Time (s)")
        fig1.suptitle("State Evolution Over Time")
        fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig1.savefig(os.path.join(OUTPUT_DIR, "state_evolution.png"))
        plt.close(fig1)

        # Plot 2: Control Input vs Time
        fig2, ax2 = plt.subplots(figsize=(10, 4))
        ax2.plot(t, u, label="Control Input (N)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Force (N)")
        ax2.set_title("Control Input vs Time")
        ax2.legend()
        fig2.tight_layout()
        fig2.savefig(os.path.join(OUTPUT_DIR, "control_input.png"))
        plt.close(fig2)

        # Plot 3: Pole Angle vs Cart Position
        fig3, ax3 = plt.subplots(figsize=(6, 6))
        ax3.plot(x[:, 0], x[:, 2], '.-', label="Trajectory")
        ax3.set_xlabel("Cart Position (m)")
        ax3.set_ylabel("Pole Angle (rad)")
        ax3.set_title("Pole Angle vs Cart Position")
        ax3.legend()
        fig3.tight_layout()
        fig3.savefig(os.path.join(OUTPUT_DIR, "pole_angle_vs_cart_position.png"))
        plt.close(fig3)

        # Plot 4: Q Diagonal Values vs Time
        fig4, ax4 = plt.subplots(figsize=(10, 4))
        ax4.plot(t, q_arr[:, 0], label="Q0", color='green')
        ax4.plot(t, q_arr[:, 1], label="Q1", color='blue')
        ax4.plot(t, q_arr[:, 2], label="Q2", color='red')
        ax4.plot(t, q_arr[:, 3], label="Q3", color='cyan')
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("Q Value")
        ax4.set_title("Q Values vs Time")
        ax4.legend()
        fig4.tight_layout()
        fig4.savefig(os.path.join(OUTPUT_DIR, "q_values_vs_time.png"))
        plt.close(fig4)

        # Plot 5: R Value vs Time
        fig5, ax5 = plt.subplots(figsize=(10, 4))
        ax5.plot(t, r_arr, label="R", color='magenta')
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("R Value")
        ax5.set_title("R Value vs Time")
        ax5.legend()
        fig5.tight_layout()
        fig5.savefig(os.path.join(OUTPUT_DIR, "r_value_vs_time.png"))
        plt.close(fig5)

        self.get_logger().info(f"Plots saved in {OUTPUT_DIR}")

        # Save CSV data
        csv_file = os.path.join(OUTPUT_DIR, "run_data.csv")
        with open(csv_file, mode='w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            header = ['Time (s)', 'Cart Position (m)', 'Cart Velocity (m/s)', 
                      'Pole Angle (rad)', 'Pole Angular Velocity (rad/s)', 
                      'Control Input (N)', 'Q0', 'Q1', 'Q2', 'Q3', 'R']
            csv_writer.writerow(header)
            for i in range(len(t)):
                row = [f"{t[i]:.4f}"]
                row.extend([f"{val:.4f}" for val in x[i]])
                row.append(f"{u[i]:.4f}")
                row.extend([f"{q:.4f}" for q in q_arr[i]])
                row.append(f"{r_arr[i]:.4f}")
                csv_writer.writerow(row)
        self.get_logger().info(f"CSV data saved in {csv_file}")

    def save_and_exit(self):
        self.save_plots_and_csv()

def main(args=None):
    rclpy.init(args=args)
    controller = BrysonLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt received. Saving data and shutting down...")
    finally:
        controller.save_and_exit()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
