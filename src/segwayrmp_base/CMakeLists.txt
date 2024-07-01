cmake_minimum_required(VERSION 3.8)
project(segwayrmp_base)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ros2
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# interfaces
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
)

add_executable(segwayrmp_base_node
  src/segwayrmp_base_node.cpp
  src/segwayrmp_base_ros.cpp
  src/odometry.cpp
)

ament_target_dependencies(segwayrmp_base_node
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "action_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "builtin_interfaces"
)

# Link libraries
target_link_libraries(segwayrmp_base_node

  # Use 'libctrl_x86_64.so' or 'libctrl_arm64-v8a.so'
  # depending on your system architecture
  ${PROJECT_SOURCE_DIR}/lib/libctrl_x86_64.so
)

# Install the executable
install(TARGETS segwayrmp_base_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

