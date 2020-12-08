cmake_minimum_required(VERSION 3.5)
project(kia_soul_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED True)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(VEHICLE kia_soul_ev CACHE STRING docstring FORCE)
include(cmake/OsccConfig.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(my_includes 
    include
    oscc/api/include
    oscc/api/src)

add_executable(
    drivekit
    src/commander.cpp
    src/main.cpp
    src/node.cpp
    src/oscc.cpp)

add_executable(
    servitude
    src/steering-servitude.cpp)

target_include_directories(drivekit PUBLIC
  ${my_includes}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(servitude PUBLIC
  ${my_includes}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(drivekit PRIVATE COMMANDER ROS ROS2)

ament_target_dependencies(drivekit rclcpp std_msgs can_msgs)
ament_target_dependencies(servitude rclcpp std_msgs)

install(TARGETS drivekit
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS servitude
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
