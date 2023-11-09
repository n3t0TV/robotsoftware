^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package drivers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

12.11.1 (2022-10-06)
--------------------
* Disabling card tap audio for further testing. Card tap event tests shows that
  sometimes the loading audio doesn't stop after the transaction response
  arrives or the timeout is done, so there could be a bug in threads or forking
  process.

12.11.0 (2022-10-05)
--------------------
* Loading and alarm audios added (108 and 109).
* Short audios added, success and fail were updated.
* TTS service reconnected to generate audios and language parameter was added
  to the service, the languages supported are English and Spanish (en/es).
* Loop audio function added that allows an audio to repeat every period of time.
* Speaker node refactored to add async audio playback for tap event and
  transactions response (containers v2). Its name was changed to audio_player.

12.10.0 (2022-08-04)
--------------------
* New audios added, 145 (refund) and 146 (ask for items loaded).

12.9.0 (2022-07-04)
-------------------
* Mp3's audio volume normalized & generic audios updated (113, 115 & 121).
* Custom audio folder added, if the folder exist the node looks for matches to play,
  if not just plays the generic audio.

12.8.0 (2022-06-24)
-------------------
* Sound volume control added using mpg123 scale factor (mqtt instruction added).
* Containers reset mechanism encapsulated in a ros service preventing that exception
  node reset the containers twice.
* Fix container status feedback sent by ble node, allowing the reset mechanism to
  reset them right after the connection is established and to handle the reconnection
  instead of killing them.
* Ros service for pair speaker bluetooth created (script called from joystick pkg)

12.7.0 (2022-06-06)
-------------------
* Setting default position to spin camera (-167°) whenjoystick mode is activated in
  order to see the containers lids.

12.6.1 (2022-05-16)
-------------------
* Fix in containers restart mechanism (typo).
* Blinkers light alert & scaling angular rate (soft spins) in FW safe mode.

12.6.0 (2022-04-29)
-------------------
* SPI error commands added for logging & show warning messages with the new
  kangarooless version (Fail safe mode).
* Automatic mp3 download with json added to play dinamic audios.

12.5.0 (2022-04-08)
-------------------
* BLE node killing is handled locally by the exception_node based on the flags of the
  feedback and the timing between messages.
* A fix was done to prevent the ble nodes from dying when there is no mac assigned.
* It is possible to update the mac address of the container from the backend as the
  ble node will wait until the mqtt message arrives and the master will latch last msg
  in order to resend it when the processes are restarted, the macs local file is
  discontinued.
* New mp3 files were added to notify the user the mac address was successfully updated
  and that the container is online.
* Added service to set control pid params, debug of kangarooless.

12.4.1 (2022-03-14)
-----------------------
* Bugs were fixed in the state machine of the container's BT connection.
* Disconnection errors were caught in order to trigger an automatic reconnection 
  mechanism with the containers.
* Connection with containers is restated by killing the BT nodes processes (via 
  an  mqtt command sent from the UI) and relaunching them after 5 seconds (ROS 
  respawn mechanism). This is due to a low level bug when reading feedback from
  the containers.
* Audios from 110 to 142 were substituted for sintetic audios generated with Google’s 
  Text-to-Speech Cloud API to homologate recordings.
* Automatic reconnection with new container when mac address is updated via mqtt message.
* ESP32 firmware version 3.2.0 is now supported. Version 3.2.0 shows version id, mode, 
  sleep and awake times when feedback is requested.
* Control over log level output in BT nodes was implemented.
* Add speaker volume control in the speaker ROS node. It's also possible to retrieve 
  the volume level value.
* Latch ROS messages for the camera exposure publisher for Jetson Nano. The exposure 
  values coming from the backend that are reported way before the camera nodes are fully 
  up and running can be processed with this fix.


12.3.1 (2022-02-21)
-------------------
* Connect the exposure values coming from the tortops UI to the brain vision
  node. The previous version for the Raspberry Pi has been preserved by
  encapsulating the implementation per platform in a Camera Exposure Publisher
  class.
* Add brainvision as a build dependency of drivers to use the CamExposure
  message.
* BT-container’s controller toggle mechanism was replaced by a reconnection
  mechanism.
* FW .bin name is standardized in launch & code.

12.3.0 (2022-02-14)
-------------------
* New implementation of bt node to allow permanent connection when in vending
  machine mode.
* Containers feedback is published to mqtt_publishers topic to reach the UI
  through the mqtt protocol.
* Mechanism for locally storing IMU data was implemented, storage is controlled
  by a flag in imu_spi_node located in test_wagon_drivers.launch.
* New audios for vending machine flow were added and old audios and new audios
  were amplified.
* Json commands files were moved from braintemp to config folder.
* Pass yaw orientation directly to UI & improving accum transactions over spi
  changing cast for better accuracy in floating point.
* Filtered kangaroo errors sent to UI.
* Fixed servo issue: add latching publisher for servo power enable.
* Added battery voltage field to picmanager sensors for debugging.
* Added uart_node to brain for pic programing OTA.
* Three new audios were added.

12.2.1 (2021-12-14)
-------------------
* Fix merge conflicts in PicManagerDriverJetson.h.

12.2.0 (2021-12-14)
-------------------
* Removed joystick node; the node was moved to its own ROS package.
* Turn brake lights off when vehicle is idle (no teleoperation).
* Control the low-battery LED indicator.
* Change the ceiling value of the battery level to 100, previously 128. This
  value is used in the teleoperation to know the battery charge level.
* Don't limit the volume in audios used in the speaker of the wagon.
* Save samples of the IMU magnetometer in CSV files.
* Enable LPM pin to control power sources of UCPER (fixex PIC reset bug).

12.1.0 (2021-11-22)
-------------------
* Use GPIO to enable and disable the speaker for the Jetson Nano.
* Publish data from the magnetometer in the ROS topic "/sensor_topic/mag".
* Use GPIO to enable and disable the UPS for the Jetson Nano.
* Use the calibration JSON to control the PWM for the servo motor of the spin
  camera to the desired rotation angles.

12.0.2 (2021-11-18)
-------------------
* Install the launch folder instead of single launch files in CMake.
* Install the mp3 folder in CMake.
* Changed the path to the mp3 files to be relative to the drivers module, not to
  the catkin development workspace in the Speaker class for both Raspberry Pi
  and Jetson Nano.

12.0.1 (2021-11-09)
-------------------
* Add nodes and launch files as install targets.

12.0.0 (2021-11-08)
-------------------
* The access word for the containers changed, as well as the location of the
  container MACs in the file system.
* Audios were added to cross street.

Only for systems with Raspberry Pi:

* Calibration files were added to position the servo motor for the camera at
  precise angles.

Only for systems with Jetson Nano:

* The PCA servo driver board was to a local PWM pin in the Jetson NANO SoM.
* Added support for two GPS modules, as well as a GPS fusion node.
