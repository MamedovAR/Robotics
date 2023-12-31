cmake_minimum_required(VERSION 3.8)
project(move_to_goal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
# find_package(service_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED)

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

# Add executables
add_executable(move_to_goal src/move_to_goal.cpp)

# Specify dependencies for the executable
# ament_target_dependencies(move_to_goal geometry_msgs)
ament_target_dependencies(move_to_goal geometry_msgs rclcpp turtlesim)

install(TARGETS

  move_to_goal
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
# ament_export_dependencies(ament_cmake)

ament_package()