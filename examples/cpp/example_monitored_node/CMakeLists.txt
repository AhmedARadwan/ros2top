cmake_minimum_required(VERSION 3.8)
project(example_monitored_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

# ros2top is a pip package, so its CMake config is not on CMake's default
# search path unless it was installed system-wide. If this fails, run:
#   colcon build --cmake-args -Dros2top_DIR=$(ros2top --cmake-dir)
find_package(ros2top REQUIRED)

# Create executable
add_executable(example_monitored_node src/example_monitored_node.cpp)

# Link dependencies.
#
# Imported targets rather than ament_target_dependencies(): that command is no
# longer provided by ament_cmake on Rolling, so this package did not configure
# there at all. rclcpp::rclcpp and ${std_msgs_TARGETS} exist on Humble, Jazzy,
# Kilted and Rolling alike.
#
# ros2top::ros2top carries the include path, the C++17 requirement of
# ros2top/ros2top.hpp, and nlohmann_json.
target_link_libraries(example_monitored_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  nlohmann_json::nlohmann_json
  ros2top::ros2top
)

# Install executable
install(TARGETS
  example_monitored_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
