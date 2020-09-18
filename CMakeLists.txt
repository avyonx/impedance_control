cmake_minimum_required(VERSION 2.8.3)
project(impedance_control)

set (CMAKE_CXX_STANDARD 11)

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

generate_dynamic_reconfigure_options(
    config/impedance_control_params.cfg
    )

add_message_files(
  FILES
  #MRAIController.msg
  Modes.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
   LIBRARIES impedance_control
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

add_library(function src/function.cpp)
target_link_libraries(function ${catkin_LIBRARIES})

add_library(median_filter src/median_filter.cpp)
target_link_libraries(median_filter ${catkin_LIBRARIES})

add_library(impedance_control src/impedance_control.cpp)
target_link_libraries(impedance_control ${catkin_LIBRARIES} Tf2)

add_library(aic src/aic.cpp)
target_link_libraries(aic ${catkin_LIBRARIES} Tf1 Tf2 function)

add_executable(impedance_control_node src/impedance_control_node.cpp)
target_link_libraries(impedance_control_node ${catkin_LIBRARIES} yaml-cpp impedance_control aic)

add_executable(force_filter_node src/force_filter_node.cpp)
target_link_libraries(force_filter_node ${catkin_LIBRARIES} Tf2 median_filter Tf1)

add_executable(wall_planner_node src/asap_wall_planner.cpp)
target_link_libraries(wall_planner_node ${catkin_LIBRARIES})
