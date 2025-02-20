#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class ResetTimePublisher(Node):
    def __init__(self):
        super().__init__('reset_time_publisher')
        self.publisher_ = self.create_publisher(Empty, '/reset_time', 10)
        # Publish one reset message after a short delay (or you can publish repeatedly)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        msg = Empty()
        self.get_logger().info('Publishing empty message on /reset_time to reset TF buffer')
        self.publisher_.publish(msg)
        # If you only need to send one message, you could cancel the timer after the first publish:
        # self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = ResetTimePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
