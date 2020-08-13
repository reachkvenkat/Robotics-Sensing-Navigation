#!/usr/bin/env python


'''
    Import libraries
'''
import rospy
import serial
from gnss_driver.msg import GNSS
import utm

# Function calculates latitude by accepting raw latitude data from GPGGA values
def calcLatitude(raw_lat, lat_dir):
    latitude = float(raw_lat[0:2]) + float(raw_lat[2:]) / 60.0
    if lat_dir == 'S':
        latitude = -latitude
    return latitude

# Function calculates longitude by accepting raw longitude data from GPGGA values
def calcLongitude(raw_long, long_dir):
    longitude = float(raw_long[0:3]) + float(raw_long[3:]) / 60.0
    if long_dir == 'W':
        longitude = -longitude
    return longitude

# Function calculates altitude by accepting raw altitude data from GPGGA values
def calcAltitude(raw_altitude, mean_sea_level):
    return (float(raw_altitude) + float(mean_sea_level))

# Conver GPGGA data to UTM values
def calcUtm(real_lat,real_long):
    utm_data = utm.from_latlon(real_lat, real_long)
    utm_values = {}
    utm_values['utm_easting'] = utm_data[0] 
    utm_values['utm_northing'] = utm_data[1]
    utm_values['utm_zone'] = utm_data[2]
    utm_values['utm_letter'] = utm_data[3] 
    return utm_values

# Get GNSS Data from GPGGA values
def getGNSSData(gns_values):
    gnss_info = gns_values.split(',')
    gnss_raw_data = {}
    gnss_raw_data['lat_value'] = gnss_info[2]
    gnss_raw_data['lat_dir'] = gnss_info[3]
    gnss_raw_data['long_val'] = gnss_info[4]
    gnss_raw_data['long_dir'] = gnss_info[5]
    gnss_raw_data['altitude'] = gnss_info[9]
    gnss_raw_data['mean_sea_level'] = gnss_info[11]
    return gnss_raw_data

# Publish GPS and UTM Data through a custom message in ROS
def publishCustomMessage(gps_data, utm_data):
    
    gns_message = GNSS()
    gns_message.header.frame_id = "GNSS message"
    gns_message.header.seq = seq
    gns_message.header.stamp=rospy.Time.now()  
    
    gns_message.latitude = gps_data['latitude']
    gns_message.longitude = gps_data['longitude']
    gns_message.altitude = gps_data['altitude']
    
    gns_message.utm_easting = utm_data['utm_easting']
    gns_message.utm_northing = utm_data['utm_northing']
    gns_message.zone = utm_data['utm_zone']
    gns_message.letter = utm_data['utm_letter']

    gns_publisher = rospy.Publisher('gps'+'/gnss',GNSS, queue_size=5)
    gns_publisher.publish(gns_message)


if __name__ == '__main__':
    ### Start Rospy and get all the parameters.
    rospy.logdebug("Configuring Serial Port...")

    rospy.init_node('gnss_serial_connect', anonymous = True)
    serial_port = rospy.get_param('~port','/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baudrate',4800)

    ### Read the serial ports with above parameters
    gnss_serial = serial.Serial(serial_port, serial_baud, timeout=3)

    try:
        while not rospy.is_shutdown():
            gnss_data = str(gnss_serial.readline())
           
            if gnss_data is None:
                rospy.logwarn("Empty String!!!")                                                # returns warning if there is no data
            else:
                if gnss_data.startswith('$GPGGA'):
                    
                    rospy.loginfo("Received GPGGA data!!")
                    seq = 0
                    gpgga_data = getGNSSData(gnss_data)
                    gps_data = {}
                    gps_data['latitude'] = calcLatitude(gpgga_data['lat_value'],gpgga_data['lat_dir'])
                    gps_data['longitude'] =  calcLongitude(gpgga_data['long_val'],gpgga_data['long_dir'])
                    gps_data['altitude'] = calcAltitude(gpgga_data['altitude'],gpgga_data['mean_sea_level'])
                    utm_data = calcUtm(gps_data['latitude'], gps_data['longitude'])
                    publishCustomMessage(gps_data, utm_data)
                    seq = seq + 1

    except rospy.ROSInterruptException:
        gnss_serial.close()
        rospy.logwarn("ROS interrupted. Exiting program!!!")
   
    except serial.serialutil.SerialException:
        rospy.loginfo("closing Serial port!!!")