cmake_minimum_required(VERSION 2.8)
project(oscc-joystick-commander)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include(oscc/api/OsccConfig.cmake)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(my_includes 
    include
    oscc/api/include
    oscc/api/src
    ${SDL2_INCLUDE_DIRS}
    /opt/ros/melodic/include)

add_executable(
    oscc-joystick-commander
    src/commander.cpp
    src/joystick.cpp
    src/main.cpp
    src/oscc.cpp)

add_executable(
    oscc-joystick-ros
    src/joystick.cpp
    src/main.cpp
    src/oscc.cpp)

add_executable(
    oscc-ros-commander
    src/commander.cpp
    src/main.cpp
    src/oscc.cpp)

target_include_directories(
    oscc-joystick-commander
    PRIVATE
    ${my_includes})

target_link_libraries(
    oscc-joystick-commander
    PRIVATE
    ${SDL2_LIBRARIES}
	m
)

target_include_directories(
    oscc-ros-commander
    PRIVATE
    ${my_includes})

target_link_libraries(
    oscc-ros-commander
    PRIVATE
    /opt/ros/melodic/lib/libroscpp.so
    /opt/ros/melodic/lib/librosconsole.so
    /opt/ros/melodic/lib/libroscpp_serialization.so
    /opt/ros/melodic/lib/libroslib.so
    /opt/ros/melodic/lib/libxmlrpcpp.so)

target_include_directories(
    oscc-joystick-ros
    PRIVATE
    ${my_includes})

target_link_libraries(
    oscc-joystick-ros
    PRIVATE
    ${SDL2_LIBRARIES}
	m
    /opt/ros/melodic/lib/libroscpp.so
    /opt/ros/melodic/lib/librosconsole.so
    /opt/ros/melodic/lib/libroscpp_serialization.so
    /opt/ros/melodic/lib/libroslib.so
    /opt/ros/melodic/lib/libxmlrpcpp.so)

target_compile_definitions(oscc-joystick-commander PRIVATE JOYSTICK COMMANDER)
target_compile_definitions(oscc-ros-commander PRIVATE COMMANDER ROS)
target_compile_definitions(oscc-joystick-ros PRIVATE JOYSTICK ROS)