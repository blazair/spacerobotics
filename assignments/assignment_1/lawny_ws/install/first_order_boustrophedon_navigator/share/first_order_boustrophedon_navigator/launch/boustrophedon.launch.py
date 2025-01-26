from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        
        # Start our boustrophedon controller
        Node(
            package='first_order_boustrophedon_navigator',
            executable='boustrophedon_controller',
            name='boustrophedon_controller'
        )
    ]) 