cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 1. Find dependencies ----
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ---- 2. Add message files ----
# Specify the message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerformanceMetrics.msg"
)

# ---- 3. Python installation ----
install(
  PROGRAMS
    scripts/controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---- 4. Install launch files ----
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---- 5. Install parameters ----
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}/params
)

# ---- 6. Export dependencies ----
ament_export_dependencies(
  rosidl_default_runtime
  rclpy
  geometry_msgs
  std_msgs
)

ament_package()
