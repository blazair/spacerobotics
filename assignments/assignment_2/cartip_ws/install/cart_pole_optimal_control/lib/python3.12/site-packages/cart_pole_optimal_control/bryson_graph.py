#!/usr/bin/env python3
"""
Bryson's Rule LQR Controller with Graphing and CSV Logging

This script implements an LQR controller for a cart-pole system using Bryson's Rule
to set the weighting matrices Q and R. In optimal control design, Bryson's Rule is used to 
choose these matrices based on the maximum allowable values for each state and for the 
control input. Specifically, it sets:

    Q(i,i) = 1 / (max_allowed_state_i^2)
    R = 1 / (max_allowed_control^2)

This approach penalizes deviations from desired limits in a systematic manner. In this 
implementation, we start with the original Bryson weights and then periodically adjust 
the weighting by increasing Q (multiplied by 1.5) and decreasing R (multiplied by 0.8)
every 10 seconds. This changes the relative importance of state error versus control 
effort and leads to a new LQR gain at each update.

To change the multipliers in the future, simply modify the factors in the 
update_q_r() method (currently 1.5 for Q and 0.8 for R).

Bryson’s method is used here to compute the LQR parameters in a systematic way. In essence,
it is the same LQR design—the controller computes the gain K by solving the continuous-time
algebraic Riccati equation—but Bryson’s rule gives you a principled way to set the Q and R 
matrices based on known maximum deviations and forces.

The script logs the system state and control input over time. When you terminate the 
node (e.g., with Ctrl-C), it automatically generates and saves three plots and a CSV file:
  1. State Evolution Over Time (all state variables)
  2. Control Input vs. Time
  3. Pole Angle vs. Cart Position
  4. A CSV file (run_data.csv) with time, state, and control input data

The plots and CSV file are saved to:
    /home/blazar/workspaces/cartip_ws/src/cart_pole_optimal_control/graphs/run1

They can be used for documentation and further analysis.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
from scipy import linalg
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend to allow saving plots
import matplotlib.pyplot as plt
import os
import csv

class BrysonLQRController(Node):
    def __init__(self):
        super().__init__('bryson_lqr_controller')

        ###################################
        # 1) System Physical Parameters
        ###################################
        self.M = 1.0   # Mass of cart (kg)
        self.m = 1.0   # Mass of pole (kg)
        self.L = 1.0   # Pole length (m)
        self.g = 9.81  # Gravity (m/s^2)

        ###################################
        # 2) Bryson’s Rule Setup
        ###################################
        # Maximum allowed values for state variables and control force
        self.x_max = 0.3          # Max cart displacement (m)
        self.xdot_max = 2.0       # Max cart velocity (m/s)
        self.theta_max = 0.2      # Max pole angle from vertical (rad)
        self.thetadot_max = 2.0   # Max pole angular velocity (rad/s)
        self.max_force = 15.0     # Max control force (N)

        ###################################
        # 3) Construct the State-Space Matrices
        ###################################
        self.initialize_state_space_matrices()

        ###################################
        # 4) Compute the Bryson’s Rule Q, R
        ###################################
        self.compute_bryson_weights()

        # Save original Q and R for later adjustments
        self.Q_orig = self.Q.copy()
        self.R_orig = self.R.copy()
        # Initialize multipliers. To change future behavior, modify these factors in update_q_r().
        self.q_multiplier = 1.0
        self.r_multiplier = 1.0

        ###################################
        # 5) Compute the LQR Gain
        ###################################
        self.K = self.compute_lqr_gain()

        ###################################
        # 6) State Estimate & Data Logging Setup
        ###################################
        self.x = np.zeros((4, 1), dtype=np.float64)
        self.state_initialized = False

        # Data logs for plotting and CSV
        self.t_log = []  # Time stamps
        self.x_log = []  # List of state vectors [x, xdot, theta, thetadot]
        self.u_log = []  # Control input values
        self.q_r_update_times = []  # Times when Q/R were updated
        self.q_r_values = []  # List of tuples: (q_multiplier, r_multiplier)

        # Create ROS publishers and subscribers
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

        # Control loop timer (100 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)
        # Timer to update Q and R multipliers every 10 seconds
        self.update_timer = self.create_timer(10.0, self.update_q_r)

        self.get_logger().info('Bryson LQR Controller initialized.')

    def initialize_state_space_matrices(self):
        """Initialize the A and B matrices for the cart-pole system."""
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
        """
        Use Bryson’s rule to set Q and R.
        Q diagonal entries = 1 / (state_max^2)
        R = 1 / (max_force^2)
        """
        q0 = 1.0 / (self.x_max**2)
        q1 = 1.0 / (self.xdot_max**2)
        q2 = 1.0 / (self.theta_max**2)
        q3 = 1.0 / (self.thetadot_max**2)
        self.Q = np.diag([q0, q1, q2, q3])

        r_val = 1.0 / (self.max_force**2)
        self.R = np.array([[r_val]], dtype=np.float64)

        self.get_logger().info(
            f"Bryson Weights => Q = diag([{q0:.4f}, {q1:.4f}, {q2:.4f}, {q3:.4f}]), R = {r_val:.6f}"
        )

    def compute_lqr_gain(self):
        """Compute LQR gain matrix K using continuous-time algebraic Riccati equation."""
        try:
            A_c = np.ascontiguousarray(self.A)
            B_c = np.ascontiguousarray(self.B)
            Q_c = np.ascontiguousarray(self.Q)
            R_c = np.ascontiguousarray(self.R)

            P = linalg.solve_continuous_are(A_c, B_c, Q_c, R_c)
            K = np.linalg.inv(R_c) @ B_c.T @ P

            self.get_logger().info(
                "Computed LQR gain matrix (Bryson’s rule): " +
                f"{np.array2string(K, formatter={'float_kind': lambda x: f'{x:.4f}'})}"
            )
            return K
        except Exception as e:
            self.get_logger().error(f"LQR computation failed: {e}")
            return np.zeros((1, 4), dtype=np.float64)

    def update_q_r(self):
        """
        Every 10 seconds, update Q and R by adjusting their multipliers.
        In this implementation, we multiply Q by 1.5 (increasing the penalty on state deviations)
        and multiply R by 0.8 (reducing the penalty on control effort). This changes the ratio of Q to R,
        resulting in a new LQR gain.
        
        To change these factors, simply modify the values 1.5 and 0.8 below.
        """
        self.q_multiplier *= 1.5  # Increase Q multiplier (more penalization on state error)
        self.r_multiplier *= 0.8  # Decrease R multiplier (less penalization on control effort)
        self.Q = self.q_multiplier * self.Q_orig
        self.R = self.r_multiplier * self.R_orig
        self.K = self.compute_lqr_gain()

        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.q_r_update_times.append(current_time)
        self.q_r_values.append((self.q_multiplier, self.r_multiplier))

        self.get_logger().info(
            f"Updated Q/R at t={current_time:.2f}s: q_multiplier={self.q_multiplier:.2f}, "
            f"r_multiplier={self.r_multiplier:.2f}"
        )

    def joint_state_callback(self, msg):
        """Process joint states from Gazebo to update the system state."""
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
            self.get_logger().warn("Failed to process joint states from Gazebo.")

    def control_loop(self):
        """Compute and apply control force u = -Kx, and log the data."""
        if not self.state_initialized:
            return

        u = -self.K @ self.x
        force_cmd = float(u[0])

        # Publish the control force command
        msg = Float64()
        msg.data = force_cmd
        self.cart_cmd_pub.publish(msg)

        # Log current time, state, and control input for plotting/CSV
        current_time = self.get_clock().now().nanoseconds * 1e-9
        self.t_log.append(current_time)
        self.x_log.append(self.x.flatten())
        self.u_log.append(force_cmd)

    def save_plots_and_csv(self):
        """Generate and save plots and a CSV file from the logged data."""
        if not self.t_log:
            self.get_logger().warn("No data logged for plotting/CSV.")
            return

        # Define the directory where plots and CSV will be saved.
        graphs_dir = "/home/blazar/workspaces/cartip_ws/src/cart_pole_optimal_control/graphs"
        run_dir = os.path.join(graphs_dir, "run2")
        if not os.path.exists(run_dir):
            os.makedirs(run_dir)

        t = np.array(self.t_log)
        x = np.array(self.x_log)  # shape (n, 4)
        u = np.array(self.u_log)

        # --------------------------
        # Plot 1: State Evolution Over Time
        # --------------------------
        fig1, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
        axs[0].plot(t, x[:, 0], label='Cart Position')
        axs[0].set_ylabel("Position (m)")
        axs[1].plot(t, x[:, 1], label='Cart Velocity')
        axs[1].set_ylabel("Velocity (m/s)")
        axs[2].plot(t, x[:, 2], label='Pole Angle')
        axs[2].set_ylabel("Angle (rad)")
        axs[3].plot(t, x[:, 3], label='Pole Angular Velocity')
        axs[3].set_ylabel("Angular Velocity (rad/s)")
        axs[3].set_xlabel("Time (s)")

        # Mark Q/R update times with vertical dashed lines
        for update_time in self.q_r_update_times:
            for ax in axs:
                ax.axvline(x=update_time, color='red', linestyle='--', label='Q/R Update')
        # Remove duplicate legend entries
        handles, labels = axs[0].get_legend_handles_labels()
        unique = dict(zip(labels, handles))
        axs[0].legend(unique.values(), unique.keys())

        fig1.suptitle("State Evolution Over Time")
        fig1.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig1.savefig(os.path.join(run_dir, "state_evolution.png"))
        plt.close(fig1)

        # --------------------------
        # Plot 2: Control Input vs Time
        # --------------------------
        fig2, ax2 = plt.subplots(figsize=(10, 4))
        ax2.plot(t, u, label='Control Input')
        ax2.set_ylabel("Force (N)")
        ax2.set_xlabel("Time (s)")
        ax2.set_title("Control Input vs Time")
        ax2.legend()
        fig2.tight_layout()
        fig2.savefig(os.path.join(run_dir, "control_input.png"))
        plt.close(fig2)

        # --------------------------
        # Plot 3: Pole Angle vs Cart Position
        # --------------------------
        fig3, ax3 = plt.subplots(figsize=(6, 6))
        ax3.plot(x[:, 0], x[:, 2], '.-', label='Trajectory')
        ax3.set_xlabel("Cart Position (m)")
        ax3.set_ylabel("Pole Angle (rad)")
        ax3.set_title("Pole Angle vs Cart Position")
        ax3.legend()
        fig3.tight_layout()
        fig3.savefig(os.path.join(run_dir, "pole_angle_vs_cart_position.png"))
        plt.close(fig3)

        self.get_logger().info(
            f"Plots saved in {run_dir}: state_evolution.png, control_input.png, pole_angle_vs_cart_position.png"
        )

        # --------------------------
        # Save CSV Data
        # --------------------------
        csv_file = os.path.join(run_dir, "run_data.csv")
        with open(csv_file, mode='w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            # Write header
            header = ['Time (s)', 'Cart Position (m)', 'Cart Velocity (m/s)', 'Pole Angle (rad)', 'Pole Angular Velocity (rad/s)', 'Control Input (N)']
            csv_writer.writerow(header)
            # Write data rows
            for i in range(len(t)):
                row = [f"{t[i]:.4f}"]
                row.extend([f"{val:.4f}" for val in x[i]])
                row.append(f"{u[i]:.4f}")
                csv_writer.writerow(row)
        self.get_logger().info(f"CSV data saved in {csv_file}")

    # Save plots and CSV on shutdown
    def save_and_exit(self):
        self.save_plots_and_csv()

def main(args=None):
    rclpy.init(args=args)
    controller = BrysonLQRController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt received. Saving plots/CSV and shutting down...")
    finally:
        controller.save_and_exit()
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
