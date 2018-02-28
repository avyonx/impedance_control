cmake_minimum_required(VERSION 2.8.3)
project(mmuav_impedance_control)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    rospack
    std_msgs
    dynamic_reconfigure
    message_generation
    geometry_msgs
    trajectory_msgs
)

find_package(cmake_modules REQUIRED)

add_message_files(
  FILES
  MRAIController.msg
  Modes.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
   LIBRARIES mmuav_impedance_control
   CATKIN_DEPENDS roscpp rospy roslib rospack
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(Tf2 src/Tf2.cpp)
target_link_libraries(Tf2 ${catkin_LIBRARIES})

add_library(Tf1 src/Tf1.cpp)
target_link_libraries(Tf1 ${catkin_LIBRARIES})

add_library(diff2 src/diff2.cpp)
target_link_libraries(diff2 ${catkin_LIBRARIES})

add_library(median_filter src/median_filter.cpp)
target_link_libraries(median_filter ${catkin_LIBRARIES})

add_library(mraic src/mraic.cpp)
target_link_libraries(mraic ${catkin_LIBRARIES} diff2 Tf2 Tf1 median_filter)

add_executable(impedance_control src/impedance_control.cpp)
target_link_libraries(impedance_control ${catkin_LIBRARIES} yaml-cpp mraic median_filter)