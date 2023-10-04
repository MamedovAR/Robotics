cmake_minimum_required(VERSION 3.8)
project(action_turtle_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_turtle_commands REQUIRED)
find_package(turtlesim REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Add the executable for the action server
add_executable(action_turtle_server src/action_turtle_server.cpp)
ament_target_dependencies(action_turtle_server
  rclcpp
  rclcpp_action
  geometry_msgs
  action_turtle_commands
  turtlesim
)
target_include_directories(action_turtle_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(action_turtle_server PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(action_turtle_server ${Boost_LIBRARIES})

# Add the executable for the action client
add_executable(action_turtle_client src/action_turtle_client.cpp)
ament_target_dependencies(action_turtle_client
  rclcpp
  rclcpp_action
  geometry_msgs
  action_turtle_commands
)
target_include_directories(action_turtle_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(action_turtle_client ${Boost_LIBRARIES})

# Install executables
install(TARGETS
  action_turtle_server
  action_turtle_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
