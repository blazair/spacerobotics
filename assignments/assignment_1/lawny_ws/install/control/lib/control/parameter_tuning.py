#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from std_srvs.srv import Empty

import time

class ParameterTuner(Node):
    def __init__(self):
        super().__init__('parameter_tuner')

        # We'll subscribe to the /final_avg_error topic
        self.subscription = self.create_subscription(
            Float64,
            '/final_avg_error',
            self.final_error_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Create a client to call /reset (turtlesim reset service)
        self.reset_client = self.create_client(Empty, '/reset')

        # We will store the final average error once we receive it
        self.current_run_error = None

        # For storing results
        self.results = []  # list of tuples: (Kp_lin, Kd_lin, Kp_ang, Kd_ang, avg_error)

        # Parameter ranges
        self.kp_lin_values = [5.0, 7.5, 10.0, 12.5, 15.0]
        self.kd_lin_values = [0.0, 0.1, 0.2, 0.5, 0.8, 1.0]
        self.kp_ang_values = [2.0, 4.0, 6.0, 8.0, 10.0]
        self.kd_ang_values = [0.0, 0.1, 0.2, 0.5, 0.8, 1.0]

        # We'll run after we are sure we can call /reset
        self.timer = self.create_timer(3.0, self.run_grid_search)
        self.run_started = False
        self.combinations = []
        self.current_index = 0

    def final_error_callback(self, msg):
        """
        This is called when the /final_avg_error message is published
        by the lawnmower_controller node. We store the error and proceed.
        """
        self.current_run_error = msg.data
        self.get_logger().info(f"Received final average error: {self.current_run_error:.3f}")

    def run_grid_search(self):
        """
        Called once, or repeatedly if you prefer, but we'll do a single pass of the entire grid.
        """
        if self.run_started:
            # We are in the middle of the grid search
            self.monitor_run()
        else:
            # Initialize the grid of parameter combinations
            self.initialize_combinations()
            self.run_started = True
            self.current_index = 0
            self.start_next_run()

    def initialize_combinations(self):
        """
        Build the list of all parameter combos we want to try.
        """
        self.combinations = []
        for kp_lin in self.kp_lin_values:
            for kd_lin in self.kd_lin_values:
                for kp_ang in self.kp_ang_values:
                    for kd_ang in self.kd_ang_values:
                        self.combinations.append((kp_lin, kd_lin, kp_ang, kd_ang))
        self.get_logger().info(f"Total combinations to try: {len(self.combinations)}")

    def start_next_run(self):
        """
        Start the run for the next set of parameters in the combinations list.
        """
        if self.current_index >= len(self.combinations):
            self.get_logger().info("All combinations tested. Printing results.")
            self.print_results()
            self.timer.cancel()
            return

        # Clear current error
        self.current_run_error = None

        # Reset Turtlesim
        if self.reset_client.service_is_ready():
            request = Empty.Request()
            self.reset_client.call_async(request)
        else:
            self.get_logger().warn("/reset service not ready, waiting...")
            time.sleep(2.0)
            return self.start_next_run()

        # Now set parameters
        (kp_lin, kd_lin, kp_ang, kd_ang) = self.combinations[self.current_index]

        self.set_parameter_in_lawnmower('Kp_linear', kp_lin)
        self.set_parameter_in_lawnmower('Kd_linear', kd_lin)
        self.set_parameter_in_lawnmower('Kp_angular', kp_ang)
        self.set_parameter_in_lawnmower('Kd_angular', kd_ang)

        self.get_logger().info(
            f"Starting run {self.current_index+1}/{len(self.combinations)} with "
            f"Kp_lin={kp_lin}, Kd_lin={kd_lin}, Kp_ang={kp_ang}, Kd_ang={kd_ang}"
        )

        # Let it run. The lawnmower controller will do its path,
        # eventually publish /final_avg_error. We'll wait in monitor_run().

    def monitor_run(self):
        """
        Check if we've received the final average error from the current run.
        If yes, record it and move on to the next.
        """
        if self.current_run_error is not None:
            (kp_lin, kd_lin, kp_ang, kd_ang) = self.combinations[self.current_index]
            self.results.append((kp_lin, kd_lin, kp_ang, kd_ang, self.current_run_error))

            # Move to the next combination
            self.current_index += 1
            self.start_next_run()

    def set_parameter_in_lawnmower(self, param_name, value):
        import subprocess
        cmd = ["ros2", "param", "set", "/lawnmower_controller", param_name, str(value)]
        self.get_logger().info(f"Executing: {' '.join(cmd)}")
        subprocess.run(cmd)

    def print_results(self):
        """
        Print the results sorted by final error.
        """
        self.results.sort(key=lambda x: x[4])
        self.get_logger().info("===== Final Tuning Results =====")
        for (kp_lin, kd_lin, kp_ang, kd_ang, avg_err) in self.results[:10]:
            self.get_logger().info(
                f"Kp_lin={kp_lin}, Kd_lin={kd_lin}, "
                f"Kp_ang={kp_ang}, Kd_ang={kd_ang} => "
                f"Avg Error={avg_err:.4f}"
            )
        best = self.results[0]
        self.get_logger().info(
            f"BEST RESULT => Kp_lin={best[0]}, Kd_lin={best[1]}, "
            f"Kp_ang={best[2]}, Kd_ang={best[3]} with Avg Error={best[4]:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    tuner = ParameterTuner()
    rclpy.spin(tuner)
    tuner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
