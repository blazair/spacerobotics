import rclpy
from rclpy.node import Node
import gymnasium as gym

class GymNode(Node):
    def __init__(self):
        super().__init__('gym_node')
        self.get_logger().info("Gymnasium ROS 2 Node Started")
        
        # Create Gymnasium environment
        self.env = gym.make("CartPole-v1")
        self.state, self.info = self.env.reset()
        
        self.get_logger().info(f"Initial state: {self.state}")
        self.get_logger().info(f"Action space: {self.env.action_space}")
        self.get_logger().info(f"Observation space: {self.env.observation_space}")
        
        # Simulate one action
        action = self.env.action_space.sample()
        next_state, reward, done, truncated, _ = self.env.step(action)
        
        self.get_logger().info(f"Action taken: {action}")
        self.get_logger().info(f"Next state: {next_state}")
        self.get_logger().info(f"Reward received: {reward}")
        self.get_logger().info(f"Done flag: {done}")
        self.get_logger().info(f"Truncated flag: {truncated}")
        
        self.env.close()

def main(args=None):
    rclpy.init(args=args)
    node = GymNode()
    rclpy.spin(node)  # Keep node alive
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
