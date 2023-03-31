cmake_minimum_required(VERSION 3.0.2)
project(tsuten_simulation)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    std_msgs
)

add_executable(four_omni_drive_controller
  src/four_omni_drive_controller.cpp
)
target_include_directories(four_omni_drive_controller
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(four_omni_drive_controller
  ${catkin_LIBRARIES}
)