cmake_minimum_required(VERSION 2.8)
project(oscc-joystick-commander)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

include(oscc/api/OsccConfig.cmake)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(
    oscc-joystick-commander
    src/commander.cpp
    src/joystick.cpp
    src/main.cpp
    oscc/api/src/oscc.cpp)

target_include_directories(
    oscc-joystick-commander
    PRIVATE
    include
    oscc/api/include
    ${SDL2_INCLUDE_DIRS}
    /opt/ros/melodic/include)

target_link_libraries(
    oscc-joystick-commander
    PRIVATE
    ${SDL2_LIBRARIES}
	m
    /opt/ros/melodic/lib/libroscpp.so)
