'''
NOVA ROVER TEAM

This script retrieves GPS coordinates from the Adafruit Ultimate GPS v3 module. GPS data is sent out
from the module in NMEA 0183 format. Thus, the script parses through the data lines to obtain the GPS
coordinates and publishes them to the NavSatFix message type.

More information on the wiring of the sensor + transmitted data can be found here: 
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-ultimate-gps.pdf

Author: Andrew Stuart (94andrew.stuart@gmail.com)
Last modified: 20/09/2019
'''

#!/usr/bin/env python

import rospy
import serial
from sensor_msgs.msg import NavSatFix

# Frequency at which the main code is repeated
ROS_REFRESH_RATE = 1

class SerialInterface:
    '''
    This class provides a general interface with a serial port. It will open and read information from the 
    port until the end of the buffer.

    Attributes:
        serial_address (str): the name of the serial connection
        baud_rate (int): the baud rate of the serial connection
        timeout (int): the maximum time in milliseconds for receiving bytes before returning
        received_data (str): the received message from serial
        uart (obj): the serial object

    TODO: Implement method to close serial connection

    '''
    def __init__(self, serial_channel, baud, timeout):
        '''
        Arguments:
            serial_channel (str): see attr
            baud (int): see attr
            timeout (int): see attr
        '''
        self.received_data = ""
        self.serial_address = serial_channel
        self.baud_rate = baud
        self.timeout = timeout
        self.uart = serial.Serial(self.serial_address, baudrate=self.baud_rate, timeout=self.timeout)
        

    def __clearReceivedData(self):
        self.received_data = ""

    def readSerialInput(self):
        ''' Removes previous messages and returns latest data '''
        self.__clearReceivedData()
        while(self.uart.inWaiting() > 0):
            self.received_data += self.uart.read()
        return self.received_data

class NMEAParser:
    '''
    This class parses through NMEA formatted data to retrieve GPS coordinates. Only NMEA 0183 
    is supported, though this class can be extended for other standards.

    Attributes:
        NMEA_Version (str): The NMEA version of the data being parsed

    TODO: Extend with other NMEA types

    '''
    def __init__(self, NMEA_type):
        '''
        Arguments:
            NMEA_type (str): The NMEA standard to be parsed
        '''
        self.NMEA_version = NMEA_type
        self.__NMEA_ID = -1 # Initialise ID of NMEA as invalid (<0)      
        if(NMEA_type == 'NMEA_0183'):
            self.__NMEA_ID = 0

    def getNMEAVersion(self):
        ''' Returns internal NMEA ID '''
        return self.__NMEA_ID

    def __convertDMToDD(self, deg, min, direction):
        '''
        Converts a GPS coordinate in Degrees Minutes format to Decimal Degrees

        Arguments:
            deg (int): coordinate degrees
            min (float): coordinate minutes
            direction (char/str): the coordinate direction (N/S/E/W)

        Returns:
            float: the coordinate in decimal degrees format

        TODO: Add range limits
        '''
        direction_modifier = 1.0
        if((direction == "W") or (direction == "S")):
            direction_modifier = -1.0
        return (direction_modifier*(deg + (min/60.0)))

    def __formatGPSData(self, data):
        '''
        Takes in an array containing coordinate information strings in an array.The strings are 
        then converted into int/float for degrees and minutes.

        Arguments:
            data (str array): string array containing coordinate information in the form of
                [latitude, latitude direction, longitude, longitude direction]

        Returns:
            float array: the latitude and longitude in decimal degrees format

        TODO: Uncouple __formatGPSData and __convertDMToDD
        '''
        # For NMEA 0183....
        if(self.__NMEA_ID == 0):
            # Retrieve coordinate strings
            raw_latitude = data[0]
            latitude_direction = data[1]
            raw_longitude = data[2]
            longitude_direction = data[3]

            # Separate coordinate into degrees and minutes
            lat_deg = int(raw_latitude[0:2])
            lat_min = float(raw_latitude[2:])
            long_deg = int(raw_longitude[0:3])
            long_min = float(raw_longitude[3:])

            # Convert each coordinate into decimal degrees
            latitude_DD = self.__convertDMToDD(lat_deg, lat_min, latitude_direction)
            longitude_DD = self.__convertDMToDD(long_deg, long_min, longitude_direction)

            return [latitude_DD, longitude_DD]

    def getGPSLocation(self, data):
        '''
        The main method called - when given NMEA strings, the strings are parsed to find the
        line containing RMC (the recommended minimum coordinates). This lines contains the GPS
        coordiates. The coordinates are then split from the rest of the line and are converted
        into decimal degrees format.

        Arguments:
            data (str): string containing NMEA data lines

        Returns:
            float array: the latitude and longitude in decimal degrees format

        TODO: Add other NMEA types
        '''
        # For NMEA 0183...
        if(self.__NMEA_ID == 0):
            split_lines = []
            split_lines = data.split('$') # Each line starts with $ - we can use this to separate each line

            # Iterate through each line
            for data_line in split_lines:
                if(data_line[2:5] == 'RMC'): # The line containing the GPS coordinates has the identifier RMC
                    split_RMC_line = data_line.split(',')   # Split RMC line into different fields
                    if(len(split_RMC_line) <> 13):  # 13 fields are expected, variations from this point to corruption
                        rospy.logwarn("INVALID RMC READING - CHECK FOR HARDWARE CORRUPTION")
                        return
                    if(split_RMC_line[2] == 'V'):   # V is printed at the end of the line if no fix has been established
                        rospy.logwarn("NO GPS FIX")
                        return
                    elif(split_RMC_line[2] == 'A'): # A indicates a valid GPS fix
                        return self.__formatGPSData(split_RMC_line[3:7]) # Format the GPS data into decimal degrees
                    else:
                        rospy.logwarn("INVALID RMC READING - CHECK FOR HARDWARE CORRUPTION")
                        return



def transmitGPS():
    '''
    This function sets up the ROS node, serial connection and NMEA Parser. It then repeatedly checks for new data received 
    on the serial line and converts this data to GPS coordinates if available. The frequency of checks depends on the 
    global variable ROS_REFRESH_RATE.
    '''
    gps_interface = SerialInterface("/dev/serial0", 9600, 3000)
    pub = rospy.Publisher('ant_gps', NavSatFix, queue_size=10)
    rospy.init_node('antenna_gps', anonymous=True)
    rate = rospy.Rate(ROS_REFRESH_RATE)
    parser = NMEAParser('NMEA_0183')
    while not rospy.is_shutdown():
        received_data = gps_interface.readSerialInput()                 # Get serial data
        [latitude, longitude] = parser.getGPSLocation(received_data)    # Parse data and set message fields
        pub.latitude = latitude                                         # Publish coordinates
        pub.longitude = longitude
        rate.sleep()                                                    # Sleep until next check

if __name__ == '__main__':
    try:
        transmitGPS()
    except rospy.ROSInterruptException:
        pass
