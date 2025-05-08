#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64
from control.msg import PerformanceMetrics  # Custom message for performance metrics
import numpy as np
import math
from collections import deque


class BoustrophedonController(Node):
    def __init__(self):
        super().__init__('lawnmower_controller')
        
        # Declare parameters for dynamic tuning
        self.declare_parameter('Kp_linear', 10.0)
        self.declare_parameter('Kd_linear', 0.1)
        self.declare_parameter('Kp_angular', 5.0)
        self.declare_parameter('Kd_angular', 0.2)

        # Read initial parameter values
        self.Kp_linear = self.get_parameter('Kp_linear').value
        self.Kd_linear = self.get_parameter('Kd_linear').value
        self.Kp_angular = self.get_parameter('Kp_angular').value
        self.Kd_angular = self.get_parameter('Kd_angular').value

        # Parameter callback for live updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create publishers and subscribers
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Publishers for cross-track error and performance metrics
        self.error_pub = self.create_publisher(Float64, '/cross_track_error', 10)
        self.metrics_pub = self.create_publisher(PerformanceMetrics, '/performance_metrics', 10)

        # Internal states
        self.pose = Pose()
        self.waypoints = self.generate_waypoints()
        self.current_waypoint = 0
        self.cross_track_errors = deque(maxlen=1000)
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.prev_time = self.get_clock().now()
        self.min_error = float('inf')
        self.max_error = 0.0

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Lawnmower controller started')

    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            if param.name == 'Kp_linear':
                self.Kp_linear = float(param.value)
            elif param.name == 'Kd_linear':
                self.Kd_linear = float(param.value)
            elif param.name == 'Kp_angular':
                self.Kp_angular = float(param.value)
            elif param.name == 'Kd_angular':
                self.Kd_angular = float(param.value)
        return SetParametersResult(successful=True)

    def generate_waypoints(self):
        waypoints = []
        y = 8.0
        while y >= 3.0:
            if len(waypoints) % 2 == 0:
                waypoints.append((2.0, y))
                waypoints.append((9.0, y))
            else:
                waypoints.append((9.0, y))
                waypoints.append((2.0, y))
            y -= 1.0
        return waypoints

    def pose_callback(self, msg):
        self.pose = msg

    def wrap_position(self):
        if self.pose.x < 0.0:
            self.pose.x = 11.0
        elif self.pose.x > 11.0:
            self.pose.x = 0.0
        if self.pose.y < 0.0:
            self.pose.y = 11.0
        elif self.pose.y > 11.0:
            self.pose.y = 0.0

    def calculate_cross_track_error(self):
        if self.current_waypoint < 1:
            return 0.0
        start = np.array(self.waypoints[self.current_waypoint - 1])
        end = np.array(self.waypoints[self.current_waypoint])
        pos = np.array([self.pose.x, self.pose.y])
        path_vector = end - start
        path_length = np.linalg.norm(path_vector)
        if path_length < 1e-6:
            return np.linalg.norm(pos - start)
        path_unit = path_vector / path_length
        pos_vector = pos - start
        projection_length = np.dot(pos_vector, path_unit)
        projection_length = max(0, min(path_length, projection_length))
        projected_point = start + projection_length * path_unit
        error_vector = pos - projected_point
        error_sign = np.sign(np.cross(path_unit, error_vector / np.linalg.norm(error_vector)))
        error = np.linalg.norm(error_vector) * error_sign
        self.cross_track_errors.append(abs(error))
        self.min_error = min(self.min_error, abs(error))
        self.max_error = max(self.max_error, abs(error))
        error_msg = Float64()
        error_msg.data = error
        self.error_pub.publish(error_msg)
        return error

    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.current_waypoint = 0
        self.wrap_position()
        cross_track_error = self.calculate_cross_track_error()
        target_x, target_y = self.waypoints[self.current_waypoint]
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 1e-5
        distance = math.sqrt((target_x - self.pose.x) ** 2 + (target_y - self.pose.y) ** 2)
        target_angle = math.atan2(target_y - self.pose.y, target_x - self.pose.x)
        angular_error = target_angle - self.pose.theta
        angular_error = (angular_error + math.pi) % (2 * math.pi) - math.pi
        linear_error_derivative = (distance - self.prev_linear_error) / dt
        angular_error_derivative = (angular_error - self.prev_angular_error) / dt
        linear_velocity = self.Kp_linear * distance + self.Kd_linear * linear_error_derivative
        angular_velocity = self.Kp_angular * angular_error + self.Kd_angular * angular_error_derivative
        vel_msg = Twist()
        vel_msg.linear.x = min(linear_velocity, 2.0)
        vel_msg.angular.z = angular_velocity
        self.velocity_publisher.publish(vel_msg)
        metrics_msg = PerformanceMetrics()
        metrics_msg.cross_track_error = float(cross_track_error)
        metrics_msg.current_velocity = float(vel_msg.linear.x)
        metrics_msg.distance_to_next_waypoint = float(distance)
        metrics_msg.completion_percentage = (self.current_waypoint / len(self.waypoints)) * 100.0
        metrics_msg.max_cross_track_error = float(self.max_error)
        metrics_msg.min_cross_track_error = float(self.min_error)
        metrics_msg.current_waypoint_index = self.current_waypoint
        metrics_msg.total_waypoints = len(self.waypoints)
        self.metrics_pub.publish(metrics_msg)
        self.prev_linear_error = distance
        self.prev_angular_error = angular_error
        self.prev_time = current_time
        if distance < 0.1:
            self.current_waypoint += 1


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
