#!/usr/bin/env python
# Based on Simple Adafruit BNO055 sensor reading example.
# More info
# https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/software
# 

import rospy
import time
import logging
import sys

import adafruit_bno055
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped

bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
def Initialize():
    
    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        rospy.logerr('Failed to initialize BNO055! Is the sensor connected?')
        exit()
    else:
        rospy.loginfo ('Initialization succeded')
        bno.set_axis_remap(BNO055.AXIS_REMAP_Y, BNO055.AXIS_REMAP_X, BNO055.AXIS_REMAP_Z,x_sign=BNO055.AXIS_REMAP_NEGATIVE)
        
def CheckStatus():
    status, self_test, error = bno.get_system_status()
    rospy.loginfo('System status: {0}'.format(status))
    rospy.loginfo('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        rospy.logerr('System error: {0}'.format(error))
        rospy.logwarn('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    rospy.loginfo('Software version:   {0}'.format(sw))
    rospy.loginfo('Bootloader version: {0}'.format(bl))
    rospy.loginfo('Accelerometer ID:   0x{0:02X}'.format(accel))
    rospy.loginfo('Magnetometer ID:    0x{0:02X}'.format(mag))
    rospy.loginfo('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    print('Reading BNO055 data, press Ctrl-C to quit...')
    
def GetOrientationQuaternion():
    q = Quaternion()
    q.x,q.y,q.z,q.w = bno.read_quaternion()
    return q

def GetAngularVelocity():
    v = Vector3()
    v.x,v.y,v.z = bno.read_gyroscope()
    return v

def GetLinearAcceleration():
    v = Vector3()
    v.x,v.y,v.z = bno.read_linear_acceleration()
    return v

def PrintCalibStatus():
    heading, roll, pitch = bno.read_euler()
    sys, gyro, accel, mag = bno.get_calibration_status()
    # Print everything out.
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading, roll, pitch, sys, gyro, accel, mag))
def main():
    pub = rospy.Publisher('imu', Imu,queue_size=1 )
    rospy.init_node('bno055_node')
    rate = rospy.Rate(10)
    # Raspberry Pi configuration with serial UART and RST connected to GPIO 18:
    Initialize()
    CheckStatus()
    # Print system status and self test result.
    while not rospy.is_shutdown():
        imu = Imu()
        
        imu.header.stamp = rospy.get_rostime()
        imu.header.frame_id = 'imu_link'
        imu.orientation = GetOrientationQuaternion()
        imu.angular_velocity = GetAngularVelocity()
        imu.linear_acceleration = GetLinearAcceleration()
        
        pub.publish(imu)
        PrintCalibStatus()
        rate.sleep()
        
    
