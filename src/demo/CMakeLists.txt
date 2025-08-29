cmake_minimum_required(VERSION 3.8)
project(demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(lifecycle_publisher src/lifecycle_publisher.cpp)
ament_target_dependencies(lifecycle_publisher rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(lifecycle_subscriber src/lifecycle_subscriber.cpp)
ament_target_dependencies(lifecycle_subscriber rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs)

add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager rclcpp lifecycle_msgs)

install(TARGETS
  lifecycle_publisher
  lifecycle_subscriber
  lifecycle_manager
  DESTINATION lib/${PROJECT_NAME})

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
