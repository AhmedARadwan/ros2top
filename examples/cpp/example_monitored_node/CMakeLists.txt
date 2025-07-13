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
find_package(ros2top REQUIRED)

# Include ros2top headers
include_directories(${ros2top_INCLUDE_DIRS})

# Create executable
add_executable(example_monitored_node src/example_monitored_node.cpp)

# Link dependencies
ament_target_dependencies(example_monitored_node
  rclcpp
  std_msgs
  nlohmann_json
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
