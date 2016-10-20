cmake_minimum_required(VERSION 2.8.3)
project(br_motor_driver)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(MSG_DEPS
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg ${MSG_DEPS}
  cmake_modules
)

find_package(TinyXML REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp  roslib  ${MSG_DEPS} 
  DEPENDS  TinyXML
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
  ${TinyXML_INCLUDE_DIRS}
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(br_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(br_robot src/br_robot.cpp)
add_library(${PROJECT_NAME} src/ros_driver.cpp include/br_motor_driver/br_robot.h )

## Declare a C++ executable


add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver br_robot ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


add_executable(robot_simulator src/robot_simulator.cpp)
target_link_libraries(robot_simulator ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})


