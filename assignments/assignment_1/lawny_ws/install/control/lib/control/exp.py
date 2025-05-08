#!/usr/bin/env python3

# Import required modules
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
import math
from collections import deque  # To store a fixed-size history of cross-track errors
from std_msgs.msg import Float64  # For publishing scalar messages
from control.msg import PerformanceMetrics  # Custom message type for performance metrics

class BoustrophedonController(Node):
    def __init__(self):
        # Initialize the node with the name 'lawnmower_controller'
        super().__init__('knode')

        # Declare ROS2 parameters for PD gains (proportional-derivative control)
        self.declare_parameter('Kp_linear', 9.4)  # Proportional gain for linear velocity
        self.declare_parameter('Kd_linear', 0.2)   # Derivative gain for linear velocity
        self.declare_parameter('Kp_angular', 7.7)  # Proportional gain for angular velocity
        self.declare_parameter('Kd_angular', 0.05)  # Derivative gain for angular velocity

        # Get parameters from the YAML file or use the default values
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value

        # Add a callback to dynamically update the parameters when changed via RQT
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Define publishers for velocity commands and performance metrics
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.metrics_pub = self.create_publisher(PerformanceMetrics, 'performance_metrics', 10)

        # Define a subscriber for the turtle's pose (position and orientation)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Define a publisher for the cross-track error as a standalone topic
        self.error_pub = self.create_publisher(Float64, 'cross_track_error', 10)

        # Define the lawnmower pattern spacing and initialize waypoints
        self.spacing = 1.0  # Vertical distance between successive rows
        self.waypoints = self.generate_waypoints()  # Generate the list of waypoints to follow
        self.current_waypoint = 0  # Index of the current waypoint being targeted

        # Initialize the pose of the turtle
        self.pose = Pose()

        # Initialize a deque to store a fixed-size history of cross-track errors
        self.cross_track_errors = deque(maxlen=1000)

        # Variables for storing previous errors and time for derivative calculations
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()

        # Initialize filtered angular velocity for low-pass filter
        self.filtered_angular_velocity = 0.0

        # ---- ADDITION #1: Start time and 5-second delay variable ----
        self.start_time = self.get_clock().now()
        self.start_delay_seconds = 5.0

        # Define a timer to call the control loop at regular intervals (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Log initialization messages
        self.get_logger().info('Lawnmower controller started')
        self.get_logger().info(f'Following waypoints: {self.waypoints}')

    def parameter_callback(self, params):
        """
        Callback to update parameters dynamically when they are changed.
        """
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'Kp_linear':
                self.Kp_linear = float(param.value)
                self.get_logger().info(f'Kp_linear updated to {self.Kp_linear}')
            elif param.name == 'Kd_linear':
                self.Kd_linear = float(param.value)
                self.get_logger().info(f'Kd_linear updated to {self.Kd_linear}')
            elif param.name == 'Kp_angular':
                self.Kp_angular = float(param.value)
                self.get_logger().info(f'Kp_angular updated to {self.Kp_angular}')
            elif param.name == 'Kd_angular':
                self.Kd_angular = float(param.value)
                self.get_logger().info(f'Kd_angular updated to {self.Kd_angular}')
        return SetParametersResult(successful=True)

    def generate_waypoints(self):
        """
        Generate a list of waypoints for the lawnmower pattern.
        """
        waypoints = []
        y = 8.0  # Starting y-coordinate
        while y >= 3.0:  # Generate waypoints until reaching the lower y-bound
            if len(waypoints) % 2 == 0:
                # Move from left to right
                waypoints.append((2.0, y))
                waypoints.append((9.0, y))
            else:
                # Move from right to left
                waypoints.append((9.0, y))
                waypoints.append((2.0, y))
            y -= self.spacing  # Move down to the next row
        return waypoints

    def calculate_cross_track_error(self):
        """
        Calculate the cross-track error (distance to the path) and publish it.
        """
        if self.current_waypoint < 1:
            return 0.0

        # Define the start and end points of the current line segment
        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])

        # Calculate the unit vector along the path
        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:  # Handle edge case where start == end
            return np.linalg.norm(pos - start)
        path_unit = path_vector / path_length

        # Project the current position onto the path
        pos_vector = pos - start
        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit

        # Calculate the error vector and signed cross-track error
        error_vector = pos - projected_point
        error_sign = np.sign(np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign

        # Store the error and calculate statistics
        self.cross_track_errors.append(abs(error))
        current_avg = sum(self.cross_track_errors) / len(self.cross_track_errors)
        current_max = max(self.cross_track_errors)
        current_min = min(self.cross_track_errors)

        # Publish the error as a Float64 message
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        # Log statistics
        self.get_logger().info(
            f'Cross-track error - Current: {error:.3f}, '
            f'Avg: {current_avg:.3f}, '
            f'Min: {current_min:.3f}, '
            f'Max: {current_max:.3f}, '
            f'Segment: ({start[0]:.1f},{start[1]:.1f})->({end[0]:.1f},{end[1]:.1f})'
        )

        return error

    def pose_callback(self, msg):
        """
        Callback to update the turtle's current pose.
        """
        self.pose = msg

    def get_distance(self, x1, y1, x2, y2):
        """
        Calculate the Euclidean distance between two points.
        """
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_angle(self, x1, y1, x2, y2):
        """
        Calculate the angle between two points (atan2).
        """
        return math.atan2(y2 - y1, x2 - x1)

    def low_pass_filter(self, current_value, previous_value, alpha=0.8):
        """
        Applies a low-pass filter to smooth out abrupt changes in values.

        Parameters:
            current_value (float): The latest value to be filtered.
            previous_value (float): The previously filtered value.
            alpha (float): Smoothing factor (0 < alpha <= 1).

        Returns:
            float: Smoothed value after applying the low-pass filter.
        """
        return alpha * current_value + (1 - alpha) * previous_value

    def control_loop(self):
        """
        Main control loop for PD-based navigation.
        """
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        if elapsed < self.start_delay_seconds:
            vel_msg = Twist()
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info(
                f"Waiting {self.start_delay_seconds} seconds so parameters can be set. (elapsed: {elapsed:.1f}s)"
            )
            return

        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('Lawnmower pattern complete')
            self.timer.cancel()
            return

        target_x, target_y = self.waypoints[self.current_waypoint]
        dt = max((current_time - self.prev_time).nanoseconds / 1e9, 1e-5)

        cross_track_error = self.calculate_cross_track_error()
        distance = self.get_distance(self.pose.x, self.pose.y, target_x, target_y)
        target_angle = self.get_angle(self.pose.x, self.pose.y, target_x, target_y)
        angular_error = target_angle - self.pose.theta

        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        linear_velocity = (self.Kp_linear * distance +
                           self.Kd_linear * ((distance - self.prev_linear_error) / dt))
        angular_velocity = (self.Kp_angular * angular_error +
                            self.Kd_angular * ((angular_error - self.prev_angular_error) / dt))

        # Apply low-pass filter
        angular_velocity = self.low_pass_filter(angular_velocity, self.filtered_angular_velocity)
        self.filtered_angular_velocity = angular_velocity

        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)

        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time

        if distance < 0.1:
            self.current_waypoint += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    controller = BoustrophedonController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
