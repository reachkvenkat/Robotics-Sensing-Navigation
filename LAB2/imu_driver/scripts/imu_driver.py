#!/usr/bin/env python
#https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr

'''
    Import libraries
'''
import rospy
import serial
import utm
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import numpy as np
import math
import sys
from tf.transformations import quaternion_from_euler


# Converting string to float values
def trimValue(val_string):
    if val_string.startswith('+'):
        s, val_num = val_string.split('+')
    else: 
        s, val_num = val_string.split('-')
        val_num = -1 * float(val_num)
    return float(val_num)

# Get IMU Data from VNYMR values
def getIMUData(imu_values):
    imu_info = imu_values.split(',')
    imu_raw_data = {}
    roll = math.radians(float(imu_info[3]))
    pitch = math.radians(float(imu_info[2]))
    yaw = math.radians(float(imu_info[1]))
    imu_raw_data['qx'],imu_raw_data['qy'],imu_raw_data['qz'],imu_raw_data['qw'] = quaternion_from_euler(yaw,pitch,roll)   # converting yaw, pitch, roll to quaternion
    imu_raw_data['yaw'] = imu_info[1]
    imu_raw_data['pitch'] = imu_info[2]
    imu_raw_data['roll'] = imu_info[3]
    imu_raw_data['magx'] = imu_info[4]
    imu_raw_data['magy'] = imu_info[5]
    imu_raw_data['magz'] = imu_info[6]
    imu_raw_data['aclx'] = imu_info[7]
    imu_raw_data['acly'] = imu_info[8]
    imu_raw_data['aclz'] = imu_info[9]
    imu_raw_data['gyrx'] = imu_info[10]
    imu_raw_data['gyry'] = imu_info[11]
    imu_raw_data['gyrz'],temp = imu_info[12].split('*')
    return imu_raw_data

# Publish IMU Data through a custom message in ROS
def publishCustomMessage(imu_data, seq):
      imu_message = Imu()
      mgn_message = MagneticField()
      
      imu_message.header.seq = seq
      imu_message.header.frame_id = "IMU Message"
      imu_message.header.stamp = rospy.Time.now()
      
      mgn_message.header.stamp = rospy.Time.now()
      mgn_message.header.frame_id = "Magnetic field data"
      mgn_message.header.seq = seq
      
      imu_message.linear_acceleration.x = trimValue(imu_data['aclx'])
      imu_message.linear_acceleration.y = trimValue(imu_data['acly'])
      imu_message.linear_acceleration.z = trimValue(imu_data['aclz'])
      imu_message.orientation.w = imu_data['qw']
      imu_message.orientation.x = imu_data['qx']
      imu_message.orientation.y = imu_data['qy']
      imu_message.orientation.z = imu_data['qz']
      imu_message.angular_velocity.x = trimValue(imu_data['gyrx'])
      imu_message.angular_velocity.y = trimValue(imu_data['gyry'])
      imu_message.angular_velocity.z = trimValue(imu_data['gyrz'])
      
      mgn_message.magnetic_field.x = trimValue(imu_data['magx'])
      mgn_message.magnetic_field.y = trimValue(imu_data['magy'])
      mgn_message.magnetic_field.z = trimValue(imu_data['magz'])

# Publishing magnetic field data and IMU data
      imu_publisher = rospy.Publisher('sensor'+'/imu',Imu, queue_size = 5)
      mgn_publisher = rospy.Publisher('sensor'+'/mag', MagneticField, queue_size = 5)

      imu_publisher.publish(imu_message)
      mgn_publisher.publish(mgn_message)



if __name__ == '__main__':
    ### Start Rospy and get all the parameters.
    rospy.logdebug("Configuring Serial Port...")
    port = sys.argv[1]
    rospy.init_node('IMU Serial connect', anonymous = True)
    serial_port = rospy.get_param('~port',port)
    serial_baud = rospy.get_param('~baudrate',115200)

    ### Read the serial ports with above parameters
    imu_serial = serial.Serial(serial_port, serial_baud, timeout=3)

    try:
        while not rospy.is_shutdown():
            imu_rdata = str(imu_serial.readline())
           
            if imu_rdata is None:
                rospy.logwarn("Empty String!!!")                                                # returns warning if there is no data
            else:
                if imu_rdata.startswith('$VNYMR'):
                    seq = 0
                    rospy.loginfo("Received IMU raw data!! Seq: " + str(seq))
                    vnymr_data = getIMUData(imu_rdata)
                    publishCustomMessage(vnymr_data, seq)
                    seq = seq + 1

    except rospy.ROSInterruptException:
        imu_serial.close()
        rospy.logwarn("ROS interrupted. Exiting program!!!")
   
    except serial.serialutil.SerialException:
        rospy.loginfo("closing Serial port!!!")