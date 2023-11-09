cmake_minimum_required(VERSION 2.8.3)
project(ota)

find_package(catkin REQUIRED COMPONENTS
 rospy
 roscpp
 roslib
 std_msgs
 message_generation
 sensor_msgs
 modules
)

#add_message_files(

   
#)

#add_service_files(  FILES
    
#)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES modules
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(ota_node src/ota_code.cpp)
add_dependencies(ota_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ota_node ${catkin_LIBRARIES} -lmosquittopp)	

add_executable(vehicle_version_node src/vehicle_version_code.cpp)
add_dependencies(vehicle_version_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_version_node ${catkin_LIBRARIES}) 

add_executable(mqtt_manager_node src/mqtt_manager_code.cpp)
add_dependencies(mqtt_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mqtt_manager_node ${catkin_LIBRARIES} -lmosquittopp)	

install(TARGETS
  ota_node
  vehicle_version_node
  mqtt_manager_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
