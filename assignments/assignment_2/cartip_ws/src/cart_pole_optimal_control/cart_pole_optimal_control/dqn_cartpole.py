#!/home/blazar/envs/ros2_rl/bin/python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import gymnasium as gym
import numpy as np
from collections import deque
import random
import torch
import torch.nn as nn
import torch.optim as optim

###############################################################################
# 1) Q-Network: Simple MLP for the DQN
###############################################################################
class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, 64)
        self.fc2 = nn.Linear(64, 64)
        self.fc3 = nn.Linear(64, action_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)

###############################################################################
# 2) DQNAgent: Stores memory, handles epsilon-greedy, training, etc.
###############################################################################
class DQNAgent:
    def __init__(self, state_dim, action_dim,
                 gamma=0.99, lr=1e-3,
                 epsilon=0.2, min_epsilon=0.01, decay=0.995):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.epsilon = epsilon
        self.min_epsilon = min_epsilon
        self.decay = decay

        self.batch_size = 128
        self.memory = deque(maxlen=100_000)

        self.q_network = QNetwork(state_dim, action_dim)
        self.target_network = QNetwork(state_dim, action_dim)
        self.target_network.load_state_dict(self.q_network.state_dict())

        self.optimizer = optim.Adam(self.q_network.parameters(), lr=lr)

    def select_action(self, state, evaluate=False):
        """Epsilon-greedy selection."""
        if (not evaluate) and (random.random() < self.epsilon):
            return random.randint(0, self.action_dim - 1)
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
            return torch.argmax(q_values).item()

    def store_transition(self, state, action, reward, next_state, done):
        """Save transition with a naive 'priority' approach."""
        priority = abs(reward) + 1e-5
        self.memory.append((priority, state, action, reward, next_state, done))

    def train(self):
        if len(self.memory) < self.batch_size:
            return

        # Weighted sampling using reward as priority (simplified)
        priorities = np.array([exp[0] for exp in self.memory], dtype=np.float32)
        priorities /= (priorities.sum() + 1e-8)
        idxs = np.random.choice(len(self.memory), self.batch_size, p=0.7 * priorities + 0.3 * (1/len(self.memory)))
        batch = [self.memory[i] for i in idxs]

        # Unpack
        _, states, actions, rewards, next_states, dones = zip(*[(b[0], b[1], b[2], b[3], b[4], b[5]) for b in [(batch[i][0],) + batch[i][1:] for i in range(len(batch))]])
        # Correction: it's simpler to do:
        # batch_elt = [p, s, a, r, ns, d]
        # but let's keep it consistent.

        states = torch.FloatTensor(states)
        actions = torch.LongTensor(actions).unsqueeze(1)
        rewards = torch.FloatTensor(rewards).unsqueeze(1)
        next_states = torch.FloatTensor(next_states)
        dones = torch.FloatTensor(dones).unsqueeze(1)

        q_values = self.q_network(states).gather(1, actions)

        with torch.no_grad():
            next_q_values = self.target_network(next_states).max(1, keepdim=True)[0]
            target_q_values = rewards + (1 - dones) * self.gamma * next_q_values

        loss_fn = nn.MSELoss()
        loss = loss_fn(q_values, target_q_values)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Epsilon decay
        self.epsilon = max(self.min_epsilon, self.epsilon * self.decay)

    def update_target_model(self):
        self.target_network.load_state_dict(self.q_network.state_dict())

    def save_model(self, filename="dqn_model.pth"):
        torch.save(self.q_network.state_dict(), filename)
        print(f"Model saved as {filename}")

    def load_model(self, filename="dqn_model.pth"):
        self.q_network.load_state_dict(torch.load(filename))
        self.q_network.eval()
        print(f"Loaded model from {filename}")

###############################################################################
# 3) CartPoleEnvWithEarthquake: gym Wrapper
###############################################################################
from gymnasium.wrappers import TimeLimit

class CartPoleEnvWithEarthquake(TimeLimit):
    """
    A wrapper that adds an external force (from /earthquake_force)
    and modifies the reward for the CartPole environment.
    """
    def __init__(self, env, initial_force=0.0, max_steps=1000):
        super().__init__(env, max_episode_steps=max_steps)
        self.external_force = initial_force
        self.last_action = None

    def step(self, action):
        # 1) artificially apply external force to the cart's velocity
        cart_velocity = self.env.unwrapped.state[1]
        cart_accel = self.external_force / 1.0  # F=ma (cart mass=1)
        self.env.unwrapped.state[1] += cart_accel * 0.1

        obs, reward, done, truncated, info = super().step(action)

        pole_angle = abs(obs[2])
        cart_pos = abs(obs[0])

        # Additional shaping
        reward += (1.0 - pole_angle) * 1.5
        reward += max(0, 1 - cart_pos*2) * 1.5

        # Bonus for stable upright
        if pole_angle < 0.2 and cart_pos < 0.2:
            reward += 2.0

        # Encourage changing action occasionally
        if self.last_action is not None and action != self.last_action:
            reward += 0.1

        self.last_action = action
        done = done or truncated  # unify flags
        return obs, reward, done, info

    def set_external_force(self, force):
        self.external_force = force

###############################################################################
# 4) DQNROSController Node
###############################################################################
class DQNROSController(Node):
    def __init__(self):
        super().__init__('dqn_cartpole')  # single-word name
        # 4.1) Create environment
        base_env = gym.make('CartPole-v1')
        self.env = CartPoleEnvWithEarthquake(base_env, initial_force=0.0, max_steps=1000)

        self.state_dim = self.env.observation_space.shape[0]
        self.action_dim = self.env.action_space.n
        self.agent = DQNAgent(self.state_dim, self.action_dim)

        self.external_force = 0.0
        self.current_episode = 1
        self.num_episodes = 1000
        self.max_steps = 2500

        # 4.2) Subscribe to the earthquake force topic
        self.force_sub = self.create_subscription(
            Float64,
            '/earthquake_force',
            self.earthquake_force_callback,
            10
        )

        # 4.3) Timer: train 1 episode every second
        self.train_timer = self.create_timer(1.0, self.train_dqn_episode)

        self.get_logger().info("DQNROSController node started. Ready to train.")

    def earthquake_force_callback(self, msg):
        self.external_force = msg.data
        self.env.set_external_force(self.external_force)

    def train_dqn_episode(self):
        # Stop after we exceed num_episodes
        if self.current_episode > self.num_episodes:
            self.get_logger().info("Training complete. Shutting down.")
            self.destroy_node()
            return

        state, _ = self.env.reset()
        total_reward = 0.0
        done = False
        steps = 0

        while not done and steps < self.max_steps:
            action = self.agent.select_action(state)
            next_state, reward, done, info = self.env.step(action)

            # if done, next_state is meaningless
            if done:
                next_state = np.zeros_like(state)

            self.agent.store_transition(state, action, reward, next_state, done)
            self.agent.train()

            state = next_state
            total_reward += reward
            steps += 1

            # Early stop if the reward is extremely large
            if total_reward > 15_000:
                self.get_logger().info(f"Episode {self.current_episode}: reward > 15k => stop early.")
                break

        self.get_logger().info(f"Episode {self.current_episode}: Reward={total_reward:.1f} Steps={steps}")

        # Update target network every 5 episodes
        if self.current_episode % 5 == 0:
            self.agent.update_target_model()
            self.get_logger().info("Target network updated.")

        # Save model every 50 episodes
        if self.current_episode % 50 == 0:
            fname = f'dqn_model_{self.current_episode}.pth'
            self.agent.save_model(fname)
            self.get_logger().info(f"Model saved: {fname}")

        self.current_episode += 1

def main(args=None):
    rclpy.init(args=args)
    node = DQNROSController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
