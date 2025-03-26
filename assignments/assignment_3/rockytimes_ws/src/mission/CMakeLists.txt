cmake_minimum_required(VERSION 3.8)
project(mission)

# Enable extra warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
# Add other dependencies as needed
# find_package(std_msgs REQUIRED)

# Install Python scripts from scripts/ as executables
install(PROGRAMS
  scripts/mission.py
  scripts/aruco_tracker.py
  scripts/geometry_tracker.py
  scripts/feature_tracker.py
  scripts/pose_visualizer.py
  scripts/spiral_trajectory.py
  scripts/cylinder_landing_node.py
  scripts/estimate.py
  scripts/landing.py
  scripts/zigzag.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configuration files, scripts, models, etc.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
