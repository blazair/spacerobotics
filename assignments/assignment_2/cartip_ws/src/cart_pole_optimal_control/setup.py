#!/home/blazar/envs/rl/bin/python


from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cart_pole_optimal_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),  # Automatically detects all Python modules
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz') + glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models/cart_pole'), glob('models/cart_pole/*.urdf')),
        (os.path.join('share', package_name, 'models/cart_pole/meshes'), glob('models/cart_pole/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jnaneshwar Das',
    maintainer_email='jdas5@asu.edu',
    description='ROS2 package for cart-pole control using LQR and RL',
    license='Creative Commons Attribution 4.0',
    entry_points={
        'console_scripts': [
            'lqr_controller = cart_pole_optimal_control.lqr_controller:main',
            'earthquake_force_generator = cart_pole_optimal_control.earthquake_force_generator:main',
            'force_visualizer = cart_pole_optimal_control.force_visualizer:main',
            'state_republisher = cart_pole_optimal_control.state_republisher:main',
            'performance_monitor = cart_pole_optimal_control.performance_monitor:main',
            'bryson_lqr_controller = cart_pole_optimal_control.bryson_lqr_controller:main',
            'dqn_cartpole = cart_pole_optimal_control.dqn_cartpole:main',
            'ros_control_env = cart_pole_optimal_control.ros_control_env:main',
            'bryson_graph = cart_pole_optimal_control.bryson_graph:main',
        ],
    },
)
