# cmake_minimum_required(VERSION 3.8)
# project(action_turtle_commands)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(turtlesim REQUIRED)
# # find_package(action_turtle_commands REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()
# endif()

# # Add the executable for the action server
# add_executable(action_turtle_server src/action_turtle_server.cpp)
# ament_target_dependencies(action_turtle_server
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   action_turtle_commands
#   turtlesim
# )
# target_include_directories(action_turtle_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_compile_definitions(action_turtle_server PRIVATE "COMPOSITION_BUILDING_DLL")
# target_link_libraries(action_turtle_server ${Boost_LIBRARIES})

# # Add the executable for the action client
# add_executable(action_turtle_client src/action_turtle_client.cpp)
# ament_target_dependencies(action_turtle_client
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   action_turtle_commands
# )
# target_include_directories(action_turtle_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_link_libraries(action_turtle_client ${Boost_LIBRARIES})

# # Install executables
# install(TARGETS
#   action_turtle_server
#   action_turtle_client
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Install launch files
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )

# # ament_package()
# cmake_minimum_required(VERSION 3.5)
# project(action_turtle_commands)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(turtlesim REQUIRED)
# find_package(action_turtle_commands1 REQUIRED)

# # ament_package()

# include_directories(
#   include
# )

# # ament_target_dependencies(action_turtle_commands rclcpp std_msgs geometry_msgs rclcpp_action turtlesim action_turtle_commands1)

# add_executable(action_turtle_server src/action_turtle_server.cpp)
# ament_target_dependencies(action_turtle_server
#   "rclcpp"
#   "rclcpp_action"
#   "geometry_msgs"
#   "std_msgs"
#   "turtlesim"
#   "action_turtle_commands1"
# )

# add_executable(action_turtle_client src/action_turtle_client.cpp)
# ament_target_dependencies(action_turtle_client
#   "rclcpp"
#   "rclcpp_action"
#   "action_turtle_commands1"
# )

# install(TARGETS
#   action_turtle_server
#   action_turtle_client
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_export_dependencies(ament_cmake)
# ament_export_dependencies(rclcpp)
# ament_export_dependencies(rclcpp_action)
# ament_export_dependencies(geometry_msgs)
# ament_export_dependencies(std_msgs)
# ament_export_dependencies(turtlesim)
# ament_export_dependencies(action_turtle_commands1)
# ament_export_dependencies(action_turtle_commands)

# ament_export_include_directories(include)
# # ament_package()
# cmake_minimum_required(VERSION 3.8)
# project(action_turtle_commands)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_action REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(turtlesim REQUIRED)
# find_package(rclcpp_components REQUIRED)
# find_package(action_turtle_commands1 REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()
# endif()

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # add_executable(action_turtle_client src/action_turtle_client.cpp)
# # ament_target_dependencies(action_turtle_client rclcpp std_msgs geometry_msgs rclcpp_action action_turtle_commands1)

# # add_executable(action_turtle_server src/action_turtle_server.cpp)
# # ament_target_dependencies(action_turtle_server rclcpp geometry_msgs rclcpp_action turtlesim action_turtle_commands1)

# add_library(action_server SHARED
#   src/action_turtle_server.cpp)
# target_include_directories(action_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# # target_compile_definitions(action_server
# #   PRIVATE "ACTION_TURTLE_COMMANDS_BUILDING_DLL")
# ament_target_dependencies(action_server
#   "action_turtle_commands1"
#   "rclcpp"
#   "std_msgs"
#   "geometry_msgs"
#   "turtlesim"
#   "rclcpp_components"
#   "rclcpp_action")
# rclcpp_components_register_node(action_server PLUGIN "action_tuturtle_commands1::MessageTurtleCommands" EXECUTABLE action_turtle_server)
# install(TARGETS
#   action_server
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

#   add_library(action_client SHARED
#   src/action_turtle_client.cpp)
# target_include_directories(action_client PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# # target_compile_definitions(action_client
# #   PRIVATE "ACTION_TURTLE_COMMANDS_BUILDING_DLL")
# ament_target_dependencies(action_client
#   "action_turtle_commands1"
#   "rclcpp"
#   "std_msgs"
#   "rclcpp_components"
#   "rclcpp_action")
# rclcpp_components_register_node(action_client PLUGIN "action_tuurtle_command::ActionTurtleClient" EXECUTABLE action_turtle_client)
# install(TARGETS
#   action_client
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

# # install(TARGETS
# #         action_turtle_client
# #         action_turtle_server
# #   DESTINATION lib/${PROJECT_NAME})

# ament_package()
cmake_minimum_required(VERSION 3.5)
project(action_turtle_commands)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(action_turtle_commands1 REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()
# endif()

include_directories(include)

add_library(action_tutorials SHARED
  src/action_turtle_client.cpp
  src/action_turtle_server.cpp)
rclcpp_components_register_node(action_tutorials PLUGIN "action_turtle_commands::ActionTurtleClient" EXECUTABLE action_turtle_client)
rclcpp_components_register_node(action_tutorials PLUGIN "action_turtle_commands::ActionTurtleServer" EXECUTABLE action_turtle_server)
target_compile_definitions(action_tutorials
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_tutorials
  "action_turtle_commands1"
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "turtlesim"
  "rclcpp_components"
  "rclcpp_action")

install(TARGETS
  action_tutorials
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()