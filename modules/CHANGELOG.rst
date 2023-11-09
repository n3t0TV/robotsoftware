^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

12.11.1 (2022-10-06)
--------------------
* Nothing was done in this package for this release.

12.11.0 (2022-10-05)
--------------------
* Nothing was done in this package for this release.

12.10.0 (2022-08-04)
--------------------
* Reducing min control rate to trigger emergency stop due to vehicles being driven
  at low speed.
* Video status switch optimized, now the browser and tabs stay open and only the URL
  changes according to state instead of constantly opening/closing.
* Condition added to open broadcaster url depending on whether the teleoperation is
  direct or from the monitor.

12.9.0 (2022-07-04)
-------------------
* Nothing was done in this package for this release.

12.8.0 (2022-06-24)
-------------------
* Nothing was done in this package for this release.

12.7.0 (2022-06-06)
-------------------
* Video monitoring added to broadcaster (control_qr_node) in joystick mode.
* Fix: adding await on imei service call to always get imei before launch browser.

12.6.1 (2022-05-16)
-------------------
* Added respawn flag to control_qr_node (chromium launcher), if the node died the
  rosmaster will restart the proccess.

12.6.0 (2022-04-29)
-------------------
* Flags for Fail safe mode are sended to the UI (kangarooless).

12.5.0 (2022-04-08)
-------------------
* Speaker_msg and container_msg were extended according to new functionality.
* Mqtt instruction was added to reload broadcaster when is in teleop or vending mode.
* Added services for chromium debugging, screenshot service captures broadcaster page
  & console service receives a text with the name of a variable and return it's value.

12.4.1 (2022-03-14)
-------------------
* Parameter added to heartbeat_node and sent over mqtt to know if brain is
  running on binary or source code.

12.3.1 (2022-02-21)
-----------------------
* Wait for version & imei service in MQTT_hearbeat node was implemented,
  avoid sending empty values to UI.
* Locking the pkg version in CMakelists to v5.5.1 because the new version
  released in January/22 has a bug that doesn't allow compile nodeJS binary.
* Cmakelists custom target added for npm: The custom target will install all
  npm dependencies when it is in development mode (binary to source code
  process).

12.3.0 (2022-02-14)
--------------------
* Added navigation_node and connect inferring with angular rate control.
* Added autopilot control from UI to enable/disable inferring control.
* Containers feedback through UDP was removed (sensor_socket_node).
* Added “vending” status in muscles, mqtt_manager & JS node (control_qr)
  allowing use of spin camera.
* Added batteryVoltage field to sensor UDP msg to show in UI.

12.2.1 (2021-12-14)
-------------------
* Nothing was done in this package for this release.

12.2.0 (2021-12-14)
-------------------
* Slight changes to handle the joystick messages.

12.1.0 (2021-11-22)
-------------------
* No changes registered in this package for this release.

12.0.2 (2021-11-18)
-------------------
* Install the launch folder instead of single launch files in CMake.
* Install the config and urdf folders in CMake.
* Added custom target to build a binary executable for the NodeJS javascript,
  which is used in the Debian package of modules.
* Added an if statement in test-leonano.launch to use the NodeJS script when
  modules runs from the catkin development workspace, and the binary when is
  not, which is when it's installed as a Debian package.

12.0.1 (2021-11-09)
-------------------
* Add nodes and launch files as install targets.

12.0.0 (2021-11-08)
-------------------
* The control JSON has been split in two parts: one is sent via USP, and the
  other via P2P using a binary image. This is mainly done to avoid the
  congestion of messages in the UDP channel.
* The chromium browser is now launched using NodeJS.
