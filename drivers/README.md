# README #

## Serial Communication ##

### Procedure ###
1. Clone the git repo onto your local storage.
2. Change into root repo directory:
    ```
    $ cd CppLinuxSerial
    ```
3. Create a new build directory and change into it:
    ```
    $ mkdir build
    $ cd build
    ```
4. Run cmake on the parent directory to generate makefile:
    ```
    $ cmake ..
    ```
5. Run make on the generated makefile to generate the static library libCppLinuxSerial.a and an unit test executable:
    ```
    $ make
    ```
6. To install the headers on your system:
    ```
    $ sudo make install
    ```
6. To link the library to your target: 
    ```
    -lCppLinuxSerial
    ```

If you get errors such as Could not open device /dev/ttyS10. Is the device name correct and do you have read/write permission?
```
    $ sudo apt-get purge modemmanager -y
    $ sudo usermod -a -G dialout $USER
    $ sudo reboot now
```

### Referencias ###
    - https://github.com/gbmhunter/CppLinuxSerial
    - https://askubuntu.com/questions/133235/how-do-i-allow-non-root-access-to-ttyusb0

## Install Geometry2 for geometry_msgs & tf2 ##
	```
	sudo apt-get install ros-melodic-geometry2
	```

## Roboclaw ##
	```
	sudo apt install python-pip
	sudo pip install pyserial
    sudo usermod -a -G dialout $USER
    sudo chmod 666 /dev/ttyACM0
	```
## I2C ##
    ```
    sudo apt-get install libi2c-dev i2c-tools
    ```

## Bluetooth ##

Instalar libreria:
```
    $ sudo apt-get install python-pip libglib2.0-dev
    $ sudo pip install bluepy
```

Agregar al final de /etc/sudoers.tmp (con ```sudo visudo```):
```
jetson ALL = NOPASSWD:SETENV: /home/jetson/catkin_ws/src/brainjetson/drivers/bin/ble_node
```

* Referencias: 
    - https://roboticsbackend.com/ros-import-python-module-from-another-package/

## rviz IMU plugin ##
Dependencia:
```
sudo apt-get install qtbase5-dev
```

```
cd catkin_ws/src
git clone -b melodic https://github.com/ccny-ros-pkg/imu_tools.git
cd ..
catkin_make
```
