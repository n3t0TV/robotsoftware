cmake_minimum_required(VERSION 2.8.3)
project(drivers)

set(CMAKE_CXX_STANDARD 17)

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
  sensor_msgs
  modules
  rosconsole
  tf2_geometry_msgs
  tf
)

catkin_python_setup()

add_message_files(
   FILES
   control_light_msg.msg
   control_motion_msg.msg
   control_expcamera_msg.msg
   sensor_light_msg.msg
   sensor_motion_msg.msg
   sensor_picmanager_msg.msg
   gps_msg.msg
   gps_dgnst_msg.msg
   orientation_msg.msg
   rpy.msg
   transcript_msg.msg
)

add_service_files(
  FILES
  firmware_command.srv
  imei_service.srv
  i2c_srv.srv
  spi_test_service.srv
  uart_test_service.srv
  gpio_expander_service.srv
  pwm_service.srv
  string_srv.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
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

  set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=4 ${ROS_COMPILE_FLAGS}")

  set(Boost_USE_STATIC_LIBS        ON)  
  set(Boost_USE_DEBUG_LIBS         ON)
  set(Boost_USE_RELEASE_LIBS       ON)
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME    OFF) 
  find_package(Boost 1.67.0 COMPONENTS log_setup log)
endif()

link_directories(/usr/local/lib)
link_directories(/usr/lib)
link_directories(/usr/lib/arm-linux-gnueabihf)

## Common nodes between platforms
if(${jetson})
  add_executable(vehicle_drivers_node
                 src/vehicle_drivers_code.cpp
                 src/cam_exposure_publisher_jetson.cpp)
endif()

if(${rasp})
  add_executable(vehicle_drivers_node
                 src/vehicle_drivers_code.cpp
                 src/cam_exposure_publisher_raspberry.cpp)
endif()

add_dependencies(vehicle_drivers_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_drivers_node ${catkin_LIBRARIES}) 

add_executable(motion_node src/motion_code.cpp)
add_dependencies(motion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_node ${catkin_LIBRARIES})

add_executable(imu_spi_node src/imu/imu_spi_code.cpp)
add_dependencies(imu_spi_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_spi_node ${catkin_LIBRARIES}) 

add_executable(control_samples_node src/control_samples_code.cpp)
add_dependencies(control_samples_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_samples_node ${catkin_LIBRARIES}) 

add_executable(exception_node src/exception_code.cpp)
add_dependencies(exception_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(exception_node ${catkin_LIBRARIES})

if(NOT (CATKIN_WORKSPACES MATCHES "/devel"))
  file(COPY "npm" DESTINATION ${CMAKE_BINARY_DIR})
  add_custom_target (speech-recognize-target ALL
    COMMAND cd "${CMAKE_BINARY_DIR}/npm" && npm install && npm install -g pkg@5.5.1 && pkg speech.js -t node12-linux-arm64 --public
  )
elseif(NOT (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/npm/node_modules"))
  add_custom_target (speech-recognize-target ALL
    COMMAND test -d "${CMAKE_CURRENT_SOURCE_DIR}/npm/node_modules" || (cd "${CMAKE_CURRENT_SOURCE_DIR}/npm" && npm install)
  )
endif()

## Jetson Nano nodes
if(${jetson})
  add_executable(gps_node src/gps/gps_code.cpp)
  add_dependencies(gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_node -lCppLinuxSerial ${catkin_LIBRARIES})

  add_executable(gps_fusion_node src/gps/gps_fusion_code.cpp)
  add_dependencies(gps_fusion_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_fusion_node ${catkin_LIBRARIES})
  target_compile_definitions(gps_fusion_node PRIVATE JETSON)

  add_executable(light_node src/light_code.cpp)
  add_dependencies(light_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(light_node ${catkin_LIBRARIES} -lgpiod)
  target_compile_definitions(light_node PRIVATE JETSON)

  add_executable(servo_node src/servo/servo_pca9685_jetson.cpp)
  add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(servo_node ${catkin_LIBRARIES} -lgpiod)

  add_executable(audio_player_node src/audio/audio_player.cpp
                 src/audio/mpg123_cmd_player.cpp
                 src/audio/speaker_jetson.cpp
                 src/audio/async_cmd_runner.cc
                 src/audio/async_cmd_runner_exception.cc
                 src/persistence_reader_writer.cpp)
  add_dependencies(audio_player_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(audio_player_node ${catkin_LIBRARIES} -lgpiod)
  target_compile_definitions(audio_player_node PRIVATE JETSON)

  add_executable(picmanager_node src/picmanager_code.cpp)
  add_dependencies(picmanager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(picmanager_node ${catkin_LIBRARIES} -lgpiod)
  target_compile_definitions(picmanager_node PRIVATE JETSON)

  add_executable(pwm_node src/pwm/pwm_code.cpp)
  add_dependencies(pwm_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(pwm_node ${catkin_LIBRARIES})
endif()

## Raspberry Pi nodes
if(${rasp})
  add_executable(gps_node src/gps/gps_rasp_code.cpp)
  add_dependencies(gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(gps_node ${catkin_LIBRARIES} -lwiringPi -lwiringPiDev)

  add_executable(light_node src/light_code.cpp)
  add_dependencies(light_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(light_node ${catkin_LIBRARIES})
  target_compile_definitions(light_node PRIVATE RASP)

  add_executable(picmanager_node src/picmanager_code.cpp)
  add_dependencies(picmanager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(picmanager_node ${catkin_LIBRARIES} -lwiringPi)
  target_compile_definitions(picmanager_node PRIVATE RASP)

  add_executable(camera_exposure_node src/camera_exposure_code.cpp)
  add_dependencies(camera_exposure_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(camera_exposure_node ${catkin_LIBRARIES})

  add_executable(servo_node src/servo/servo_pca9685.cpp)
  add_dependencies(servo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(servo_node ${catkin_LIBRARIES} -lwiringPi -lpthread)

  add_executable(audio_player_node src/audio/audio_player.cpp)
  add_dependencies(audio_player_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(audio_player_node ${catkin_LIBRARIES} -lwiringPi -lwiringPiDev)
  target_compile_definitions(audio_player_node PRIVATE RASP)
endif()

catkin_install_python(PROGRAMS bin/ble_node bin/tts_node
		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

if(NOT (CATKIN_WORKSPACES MATCHES "/devel"))
  install(PROGRAMS
    "${CMAKE_BINARY_DIR}/npm/speech"
    DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
  )
endif()

set(DRIVERS_EXEC_INST
  "vehicle_drivers_node"
  "motion_node"
  "imu_spi_node"
  "control_samples_node"
  "exception_node")

if(${jetson})
  set(DRIVERS_EXEC_INST "${DRIVERS_EXEC_INST}"
    "gps_node"
    "gps_fusion_node"
    "light_node"
    "servo_node"
    "audio_player_node"
    "picmanager_node"
    "pwm_node")
endif()

if(${rasp})
  set(DRIVERS_EXEC_INST "${DRIVERS_EXEC_INST}"
    "gps_node"
    "light_node"
    "picmanager_node"
    "camera_exposure_node"
    "servo_node"
    "audio_player_node")
endif()

install(TARGETS
  ${DRIVERS_EXEC_INST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY mp3
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
