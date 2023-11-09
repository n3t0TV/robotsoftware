^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joystick
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

12.11.1 (2022-10-06)
--------------------
* Nothing was done in this package for this release.

12.11.0 (2022-10-05)
--------------------
* Mqtt message publish was added to open containerss v2 from joystick BT.

12.10.0 (2022-08-04)
--------------------
* Nothing was done in this package for this release.

12.9.0 (2022-07-04)
-------------------
* Nothing was done in this package for this release.

12.8.0 (2022-06-24)
-------------------
* Ros service for pair joystick bluetooth updated, using python bluetool library.
  Also the node publish it own controller state (MAC & connected flag) to heartbeat
  node.

12.7.0 (2022-06-06)
-------------------
* Safe lock modified, press & hold function was implemented instead of
  pumping RT button (to enable motors).

12.6.1 (2022-05-16)
-------------------
* Setting a default camera exposure value in joystick mode to avoid poor image
  quality.
* Added safe lock when starts joystick mode.

12.6.0 (2022-04-29)
-------------------
* Connection/Disconnection events are detected, disconnection event is used to apply
  emergency stop (4 seconds  delay) instead of RT mechanism.
* Mac update service was added to save the joystick mac & launch the pairing script
  automatically.

12.5.0 (2022-04-08)
-------------------
* Joystick wireless controller functionallity was added (joystick mode) for local
  control of vehicle, containers and audios.

12.4.1 (2022-03-14)
-------------------
* Nothing was done in this package for this release.

12.3.1 (2022-02-21)
-----------------------
* Nothing was done in this package for this release.

12.3.0 (2022-02-14)
--------------------
* Nothing was done in this package for this release.

12.2.1 (2021-12-14)
-------------------
* Nothing was done in this package for this release.

12.2.0 (2021-12-14)
-------------------
* First release of the joystick packages.
* The code was moved from the code of joystick node in the drivers package.
