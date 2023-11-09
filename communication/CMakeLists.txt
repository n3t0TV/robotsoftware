cmake_minimum_required(VERSION 2.8.3)
project(communication)

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
  drivers
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(${rasp})
  set ( PROJECT_LINK_LIBS
  	libwiringPi.so
  )
endif()

add_library(i2c include/libraries/i2c/i2c.c)
add_library(spi include/libraries/spi/spi.c)
add_library(serial include/libraries/uart/wiringSerial.c)

link_directories(/usr/local/lib)
link_directories(/usr/lib)
link_directories(/usr/lib/arm-linux-gnueabihf)

add_executable(i2c_node src/i2c_code.cpp)
add_dependencies(i2c_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(i2c_node ${catkin_LIBRARIES} i2c)

## Jetson Nano nodes
if(${jetson})
  add_executable(spi_node src/spi_code.cpp)
  add_dependencies(spi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(spi_node ${catkin_LIBRARIES} spi)
  target_compile_definitions(spi_node PRIVATE JETSON)

  add_executable(uart_node src/uart_code.cpp)
  add_dependencies(uart_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(uart_node ${catkin_LIBRARIES} serial)
  target_compile_definitions(uart_node PRIVATE JETSON)
endif()

## Raspberry Pi nodes
if(${rasp})
  add_executable(spi_node src/spi_code.cpp)
  add_dependencies(spi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(spi_node ${catkin_LIBRARIES} -lwiringPi)
  target_compile_definitions(spi_node PRIVATE RASP)

  add_executable(uart_node src/uart_code.cpp)
  add_dependencies(uart_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(uart_node ${catkin_LIBRARIES} -lwiringPi)
  target_compile_definitions(uart_node PRIVATE RASP)
endif()

install(TARGETS
  i2c_node
  spi_node
  uart_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS i2c spi serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
