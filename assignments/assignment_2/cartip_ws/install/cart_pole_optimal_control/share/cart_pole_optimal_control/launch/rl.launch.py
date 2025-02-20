from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  # ✅ Correct import
import os

def generate_launch_description():
    pkg_share = FindPackageShare('cart_pole_optimal_control').find('cart_pole_optimal_control')
    hyper_yaml = os.path.join(pkg_share, 'config', 'hyper.yaml')

    return LaunchDescription([
        # Launch DQN RL Node
        Node(
            package='cart_pole_optimal_control',
            executable='dqn',
            name='dqn',
            output='screen',
            parameters=[{'config_file': hyper_yaml}]
        ),

        # Launch PPO RL Node
        Node(
            package='cart_pole_optimal_control',
            executable='ppo',
            name='ppo',
            output='screen',
            parameters=[{'config_file': hyper_yaml}]
        ),

        # Launch TensorBoard Automatically
        ExecuteProcess(
            cmd=['tensorboard', '--logdir', 'runs', '--port', '6006'],
            output='screen'
        )
    ])
