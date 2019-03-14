cmake_minimum_required(VERSION 2.8.3)
project(cable_rob)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(MSG_DEPS
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  genmsg ${MSG_DEPS}
  cmake_modules
  tf
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(VISP REQUIRED)
find_library(VISP_LIBRARIES NAMES visp HINTS ${VISP_LIBRARY_DIRS} )


## Generate added messages and services with any dependencies listed here
# generate_messages( DEPENDENCIES  ${MSG_DEPS} )

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   CartesianTrajectory.msg
   CartesianTrajectoryPoint.msg
   robot_cables_msgs.msg
   cable_msgs.msg
 )


generate_messages( DEPENDENCIES  ${MSG_DEPS} )


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy roslib message_runtime ${MSG_DEPS}
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )


add_library(controller_class src/controller_class.cpp include/cable_rob/controller_class.h )
target_link_libraries(controller_class ${VISP_LIBRARIES})
add_dependencies(controller_class cable_rob_gencpp)


#add_library(${PROJECT_NAME} include/cable_rob/controller_class.h )

## Declare a C++ executable


add_executable(tf_estim src/tf_estim.cpp)
target_link_libraries(tf_estim controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES} )


add_executable(br_controller src/br_controller.cpp)
target_link_libraries(br_controller controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(trajectory_controller src/trajectory_controller.cpp)
target_link_libraries(trajectory_controller controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(cartesian_controller src/cartesian_controller.cpp)
target_link_libraries(cartesian_controller controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(rviz_visual src/rviz_visual.cpp)
target_link_libraries(rviz_visual controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(torque_ctrl src/torque_ctrl.cpp)
target_link_libraries(torque_ctrl controller_class ${catkin_LIBRARIES} ${VISP_LIBRARIES})

