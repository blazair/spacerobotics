from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('cart_pole_optimal_control').find('cart_pole_optimal_control')
    urdf_model_path = os.path.join(pkg_share, 'models', 'cart_pole', 'model.urdf')

    return LaunchDescription([
        # Gazebo (headless mode)
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', 'empty.sdf'],
            output='screen'
        ),

        # Spawn the Cart-Pole in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'cart_pole',
                '-allow_renaming', 'true'
            ],
            output='screen'
        ),

        # Direct topic bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                '/model/cart_pole/joint/cart_to_base/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
                '/world/empty/model/cart_pole/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['cat ', urdf_model_path]),
                'publish_frequency': 50.0,
                'use_tf_static': True,
                'ignore_timestamp': True,
                'use_sim_time': True
            }]
        ),

        # State Republisher
        Node(
            package='cart_pole_optimal_control',
            executable='state_republisher',
            name='state_republisher',
            output='screen'
        ),

        # Force Visualizer
        Node(
            package='cart_pole_optimal_control',
            executable='force_visualizer',
            name='force_visualizer',
            output='screen'
        ),

        # LQR Controller (now with separate Q parameters)
        Node(
            package='cart_pole_optimal_control',
            executable='lqr_controller',
            name='lqr_controller',
            output='screen',
            parameters=[{
                'Q0': 1.0,
                'Q1': 1.0,
                'Q2': 10.0,
                'Q3': 10.0,
                'R': [0.1]
            }]
        ),

        # Earthquake Force Generator
        Node(
            package='cart_pole_optimal_control',
            executable='earthquake_force_generator',
            name='earthquake_force_generator',
            output='screen',
            parameters=[{
                'base_amplitude': 15.0,
                'frequency_range': [0.5, 4.0],
                'update_rate': 50.0
            }]
        ),

        # RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'config', 'cart_pole.rviz')],
            parameters=[{'update_rate': 50.0}]
        ),

        Node(
            package='cart_pole_optimal_control',
            executable='reset_time_publisher',
            name='reset_time_publisher',
            output='screen'
        ),

        # Automatically Open rqt_reconfigure
        ExecuteProcess(
            cmd=["ros2", "run", "rqt_reconfigure", "rqt_reconfigure"],
            output="screen"
        ),
    ])
