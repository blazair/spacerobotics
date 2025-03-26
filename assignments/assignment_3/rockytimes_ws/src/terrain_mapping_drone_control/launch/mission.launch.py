#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package share and model path
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    gz_model_path = os.path.join(pkg_share, 'models')
    
    # Set initial drone pose for Gazebo
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,0.1,0,0,0"
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    
    # PX4 SITL process
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_mono'],
        cwd=px4_autopilot_path,
        output='screen'
    )
    
    # Spawn front cylinder
    spawn_cylinder_front = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder', 'model.sdf'),
            '-name', 'cylinder_front',
            '-x', '5',
            '-y', '0',
            '-z', '0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
            '-scale', '1 1 1',
            '-static'
        ],
        output='screen'
    )
    
    # Spawn back cylinder
    spawn_cylinder_back = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_back',
            '-x', '-5',
            '-y', '0',
            '-z', '0',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
            '-static'
        ],
        output='screen'
    )
    
    # Bridge node for sensor topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/mono_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        remappings=[
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            ('/depth_camera', '/drone/front_depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info')
        ],
        output='screen'
    )
    
    # Custom ROS 2 nodes (ensure these executables exist and are installed)
    custom_nodes = [
        Node(
            package='terrain_mapping_drone_control',
            executable='aruco_tracker',
            name='aruco_tracker',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='terrain_mapping_drone_control',
            executable='feature_tracker',
            name='feature_tracker',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='terrain_mapping_drone_control',
            executable='geometry_tracker',
            name='geometry_tracker',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='terrain_mapping_drone_control',
            executable='pose_visualizer',
            name='pose_visualizer',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='terrain_mapping_drone_control',
            executable='spiral_trajectory',
            name='spiral_trajectory',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='terrain_mapping_drone_control',
            executable='cylinder_landing_node',
            name='cylinder_landing_node',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ]
    
    # Define the launch sequence with delays
    ld = LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.path.join(os.environ.get('HOME', '/home/user'), 'PX4-Autopilot'),
            description='Path to the PX4-Autopilot directory'
        ),
        
        # Start PX4 SITL
        px4_sitl,
        
        # Spawn Gazebo models
        TimerAction(period=2.0, actions=[spawn_cylinder_front]),
        TimerAction(period=2.5, actions=[spawn_cylinder_back]),
        
        # Launch the bridge
        TimerAction(period=3.0, actions=[bridge]),
        
        # Launch custom nodes (after ensuring simulation and bridge are running)
        TimerAction(period=5.0, actions=custom_nodes)
    ])
    
    return ld

