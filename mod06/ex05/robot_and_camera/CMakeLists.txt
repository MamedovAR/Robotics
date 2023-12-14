cmake_minimum_required(VERSION 3.5)

project(robot_and_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

add_executable(robot_and_camera src/robot_and_lidar.cpp)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(robot_and_camera sfml-graphics sfml-window sfml-system)

# include_directories(
#     ${cv_bridge_INCLUDE_DIRS}
#     ${vision_opencv_INCLUDE_DIRS}
# )

# target_link_libraries(robot_and_camera
#     ${cv_bridge_LIBRARIES}
#     ${vision_opencv_LIBRARIES}
# )

ament_target_dependencies(robot_and_camera geometry_msgs rclcpp sensor_msgs)

install(TARGETS
  robot_and_camera
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
