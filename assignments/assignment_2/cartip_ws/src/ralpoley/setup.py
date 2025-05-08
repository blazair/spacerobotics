from setuptools import setup
import os
os.environ["PYTHON_EXECUTABLE"] = "/home/blazar/envs/ros2_rl/bin/python3"

package_name = 'ralpoley'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blazar',
    maintainer_email='your_email@example.com',  # Change this
    description='Reinforcement Learning package using Gymnasium inside ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gymmie = ralpoley.gymmie:main',
        ],
    },
)
