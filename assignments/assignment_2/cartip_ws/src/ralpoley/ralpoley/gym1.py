import gymnasium as gym

# Create a simple environment
env = gym.make("CartPole-v1")

# Reset the environment and get the initial state
state, info = env.reset()

# Print confirmation that Gymnasium is working
print("Gymnasium imported successfully!")
print(f"Initial state: {state}")
print(f"Environment action space: {env.action_space}")
print(f"Environment observation space: {env.observation_space}")

# Take one random action in the environment
action = env.action_space.sample()  # Pick a random action
next_state, reward, done, truncated, _ = env.step(action)

print(f"Action taken: {action}")
print(f"Next state: {next_state}")
print(f"Reward received: {reward}")
print(f"Done flag: {done}")
print(f"Truncated flag: {truncated}")

# Close the environment
env.close()