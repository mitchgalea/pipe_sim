cmake_minimum_required(VERSION 2.8.3)
project(pipe_sim)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  rfid_reader
)

add_definitions( -DANALYSIS=ON )

find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   PipeLineSegment.msg
   PipeLine.msg
   PipeDrawingInstructions.msg
   PipeRobot.msg
   PipeRobots.msg
   RFIDTag.msg
   Point.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   RequestMapDistance.srv
   RequestRFIDTags.srv
   SetRobotVelocity.srv
   RequestSense.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
   rfid_reader
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pipe_sim
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_generation roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Declare a C++ library
add_library( PipeSimLibrary
    src/pipe_map.cpp
    src/pipe_segment.cpp
    src/arc.cpp
    src/line.cpp
    src/create_map.cpp
    src/pipe_sim_drawer.cpp
    src/robot.cpp
    src/particle_filter.cpp
    src/mean_shift.cpp
    src/tag_rssi_map.cpp
    src/point.cpp
    src/rssi_map.cpp
)


## Declare a C++ Executable
add_executable(pipe_map_node src/pipe_map_node.cpp)
add_executable(robot_node src/robot_node.cpp)
add_executable(generate_rssi_map_node src/generate_rssi_map_node.cpp)

add_dependencies(PipeSimLibrary  ${catkin_EXPORTED_TARGETS} pipe_sim_generate_messages_cpp)
add_dependencies(pipe_map_node ${catkin_EXPORTED_TARGETS} pipe_sim_generate_messages_cpp)
add_dependencies(robot_node ${catkin_EXPORTED_TARGETS} pipe_sim_generate_messages_cpp)
add_dependencies(generate_rssi_map_node ${catkin_EXPORTED_TARGETS} pipe_sim_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(PipeSimLibrary ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(pipe_map_node ${catkin_LIBRARIES} PipeSimLibrary ${OpenCV_LIBRARIES} )
target_link_libraries(robot_node ${catkin_LIBRARIES} PipeSimLibrary ${OpenCV_LIBRARIES} )
target_link_libraries(generate_rssi_map_node ${catkin_LIBRARIES} PipeSimLibrary ${YAML_CPP_LIBRARIES}  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pipe_sim.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
