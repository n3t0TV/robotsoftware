cmake_minimum_required(VERSION 3.0.2)
project(joystick)

## Rules to define build platform
if("$ENV{HOME}" STREQUAL "/home/pi")
  set(rasp ON)
  set(jetson OFF)
endif()

if("$ENV{HOME}" STREQUAL "/home/jetson")
  set(rasp OFF)
  set(jetson ON)
endif()

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  roslib
  std_msgs
  message_generation
  modules
  rosconsole
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_generic_node src/joystick_generic_code.cpp)
add_dependencies(joystick_generic_node ${joystick_generic_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joystick_generic_node ${catkin_LIBRARIES} )

set(JOYSTICK_EXEC_INST
  "joystick_generic_node")

install(TARGETS
  joystick_generic_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
