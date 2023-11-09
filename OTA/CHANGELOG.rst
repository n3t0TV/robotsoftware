^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ota
^^^^^^^^^^^^^^^^^^^^^^^^^

12.11.1 (2022-10-06)
--------------------
* Removing audio publish deprecated for bt macs update.

12.11.0 (2022-10-05)
--------------------
* Instrucctions added for card tap and transactions response events.

12.10.0 (2022-08-04)
--------------------
* Mqtt switch interface instruction updated (linux configs changes).
* Bluetooth service reset added when the unpair controller instruction is received
  in order to prevent bugs in pairing procedure.

12.9.0 (2022-07-04)
-------------------
* Sync custom audios instruction added. It will call the sync_audios script which
  downloads the mp3's and puts them in the braintemp folder.
* Mqtt broker switch added (environment).

12.8.0 (2022-06-24)
-------------------
* Mqtt instructions added for camera exposure control and camera switch in monitor
  (joystick mode).
* Mqtt instructions added for speaker & joystick pairing.
* Mqtt instruction added for safe shutdown (brain is stopped before it is applied).

12.7.0 (2022-06-06)
-------------------
* Stop brain service before reboot mqtt command is executed to reduce corruption file
  issues.

12.6.1 (2022-05-16)
-------------------
* Nothing was done in this package for this release.

12.6.0 (2022-04-29)
-------------------
* Nothing was done in this package for this release.

12.5.0 (2022-04-08)
-------------------
* Added try/catch block to on_message function avoiding mqtt_manager node to crash
  when parsing json or on type error.
* mqtt_manager_node triggers mac update, product refill and open container mp3 audios
  based on new flags in the mqtt message.
* Adding support for assign containers from DB over mqtt msg, the macs local file is
  discontinued.

12.4.1 (2022-03-14)
-------------------
* Added BRAIN_DEVELOP env var as ros parameter in ota launch to know if brain
  is running on binary or source code.

12.3.1 (2022-02-21)
-----------------------
* Video_processing launch is deactivated due compilation issues in .deb
  generation (opencv shared library dependencies info is missing).

12.3.0 (2022-02-14)
--------------------
* New MQTT instruction for permanent BT connection between brain and containers.
* New MQTT instruction to activate recordings when in vending machine mode.
* Saving yaml version file over mqtt OTA.
* Get global SW version from yaml & sends to heartbeat mqtt.
* Move letsencrypt.pem cert from braintemp to config folder.
* Added fw_update field in yaml for OTA pic programing.
* Notification through speaker when the vehicle can't connect to the mqtt broker.

12.2.1 (2021-12-14)
-------------------
* Nothing was done in this package for this release.

12.2.0 (2021-12-14)
-------------------
* Remove sudo from the system calls when shutting down or rebooting the system.

12.1.0 (2021-11-22)
-------------------
* Remove the generation of random IMEIs at the brain system boot, and only use
  the pre-registered IMEIs instead.

12.0.2 (2021-11-18)
-------------------
* Install the launch folder instead of single launch files in CMake.

12.0.1 (2021-11-09)
-------------------
* Add nodes and launch files as install targets.

12.0.0 (2021-11-08)
-------------------
* The ROS topic "mqtt_status_topic" was renamed to "status_topic" to simplify
  the propagation of the topic.
* The MQTT manager uses now the MAC in the carrier board as the password to
  connect with the broker.
