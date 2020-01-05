#!/usr/bin/env python
'''
NOVA ROVER TEAM

This script retrieves GPS coordinates from the Adafruit Ultimate GPS v3 module. GPS data is sent out
from the module in NMEA 0183 format. Thus, the script parses through the data lines to obtain the GPS
coordinates and publishes them to the NavSatFix message type.

More information on the wiring of the sensor + transmitted data can be found here: 
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-ultimate-gps.pdf

Author: Andrew Stuart (94andrew.stuart@gmail.com)
Last modified: 17/12/2019 by Josh Cherubino (josh.cherubino@gmail.com)

Publishes:
    NatSatFix (sensor_msgs) named /gps/gps_data:
        latitude - the latitude of the GPS in decimal degrees
        longitude - the longitude of the GPS in decimal degrees

Subscribes to:
    None
'''

import rospy
import serial
import time
from sensor_msgs.msg import NavSatFix
# Frequency at which the main code is repeated
ROS_REFRESH_RATE = 10

class SerialInterface(object):
    '''
    This class provides a general interface with a serial port. It will open and read information from the 
    port until the end of the buffer.

    Attributes:
        serial_address (str): the name of the serial connection
        baud_rate (int): the baud rate of the serial connection
        timeout (int): the maximum time in milliseconds for receiving bytes before returning
        received_data (str): the received message from serial
        uart (obj): the serial object

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

        #Create and configure serial interface WITHOUT opening it (which is handled in __enter__ method)
        self.uart = serial.Serial()
        self.uart.port = self.serial_address
        self.uart.baudrate = self.baud_rate
        self.uart.timeout = self.timeout

    def __enter__(self):
        '''Method called to open serial connection when using a context manager'''
        self.uart.open()
        return self

    def __exit__(self, *args):
        '''Method called to close serial connection following end of a context manager block'''
        self.uart.close()

    def __clearReceivedData(self):
        self.received_data = ""

    def readSerialInput(self):
        ''' Removes previous messages and returns latest data '''
        self.__clearReceivedData()
        while(self.uart.inWaiting() > 0):
            self.received_data += self.uart.read()
        return self.received_data

class GPSSerialInterface(SerialInterface):
    ''' 
    This class provides a specific serial interface for a Adafruit Ultimate GPS v3 module recieving NMEA data. 

    Attributes:
        serial_address (str): the name of the serial connection
        baud_rate (int): the baud rate of the serial connection
        timeout (int): the maximum time in milliseconds for receiving bytes before returning
        received_data (str): the received message from serial
        uart (obj): the serial object

        refresh_rate (int): the refresh rate of the GPS module between 1 to 10 
        nmea_sentences (list): A list of strings representing the NMEA sentences that should be enabled on the GPS. Valid entries are 'GGL', 'RMC', 'VTG', 'GGA', 'GSA', 'GSV' (any other entries are ignored.
    '''

    def __init__(self, refresh_rate, nmea_sentences, *args, **kwargs):
        '''
        Arguments:
            serial_channel (str): see attr
            baud (int): see attr
            timeout (int): see attr
            refresh_rate (int): see attr
            nmea_sentences (list): see attr
        '''
        if not 1 <= refresh_rate <= 10 or not isinstance(refresh_rate, int):
            raise ValueError('Inappropriate value for refresh_rate, must be an integer between 1 and 10')
        else:
            self.refresh_rate = refresh_rate
        
        self.nmea_sentences = nmea_sentences

        super(GPSSerialInterface, self).__init__(*args, **kwargs)

    def configureGPS(self):
        '''
        Configures GPS to return specified NMEA sentence data and sets its refresh rate to
        value specified by refresh_rate attr
        WARNING: Setting a high refresh rate without limiting what types of data
        are returned will result in data loss
        '''
        
        #Configure for desired NMEA Sentences
        self.send_command(self.__generateNMEASentenceCommand())

        #Need to sleep between commands so GPS accepts them 
        time.sleep(1)

        rate_command = b'PMTK220,' +  bytes(self.__determineMSValue())
        
        self.send_command(rate_command)
    
    def __generateNMEASentenceCommand(self):
        '''
        Helper function to generate the byte command to be sent to the GPS module to configure it to return the desired NMEA sentences as specific if nmea_sentences attr. For explanation see commands datasheet @ cd-shop.adafruit.com/datasheets/PMTK_A11.pdf
        '''
        #base PMTK314 command with all outputs switched off (split into prefix and data fields
        command_prefix = 'PMTK314'
        data_fields = ',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
        
        #dictionary that maps NMEA sentence to the corresponding data field index in the PMTK314
        #command
        nmea_sentence_map = {
                'GGL': 0,
                'RMC': 1,
                'VTG': 2,
                'GGA': 3,
                'GSA': 4,
                'GSV': 5
                }
        
        data_fields = list(data_fields)                            
        
        for nmea_sentence in self.nmea_sentences:
            try:
                #Modify index to account for preceeding commas in data_fields list
                index = nmea_sentence_map[nmea_sentence] * 2 + 1

                #set corresponding data field value to '1' to enable NMEA sentence
                data_fields[index] = '1'

            #ignore invalid entries in nmea_sentences
            except KeyError:
                pass
        data_fields = "".join(data_fields)

        return bytes(command_prefix + data_fields)

    def __determineMSValue(self):
        '''Helper function to determine ms delay value required to set GPS to
        desired refresh rate'''
        return int(1.0/self.refresh_rate * 1000)

    def send_command(self, command):
        '''
        Formats commands sent to configure Adafruit Ultimate GPS v3 module 
        This function is a modified version of adafruits send_command function in their adafruit-gps library
        (https://github.com/adafruit/Adafruit_CircuitPython_GPS/blob/master/adafruit_gps.py) that works in python2

        Arguments: 
            command - command to send to GPS
        '''

        self.uart.write(b'$')
        self.uart.write(command)
        checksum = 0
        for char in command:
            checksum ^= ord(char)
        self.uart.write(b'*')
        self.uart.write(bytes('{:02x}'.format(checksum).upper()))
        self.uart.write(b'\r\n')

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
                        return 0.0, 0.0
		    
                    if(split_RMC_line[2] == 'V'):   # V is printed at the end of the line if no fix has been established
                    	rospy.logwarn("NO GPS FIX")
                        return 0.0, 0.0
                    if not (self.computeChecksum(data_line)):
                        rospy.logwarn("CHECKSUM FAILED - POSSIBLE DATA CORRUPTION")
                        return 0.0, 0.0

                    elif(split_RMC_line[2] == 'A'): # A indicates a valid GPS fix
                    	return self.__formatGPSData(split_RMC_line[3:7]) # Format the GPS data into decimal degrees
                    else:
                        rospy.logwarn("INVALID RMC READING - CHECK FOR HARDWARE CORRUPTION")
                    	return 0.0, 0.0

    def computeChecksum(self,data):
        """compute a char wise XOR checksum of the data and compare it to the hex value after the * in the data string

        Argument:
        data {str} -- String containing comma separated GPS data with checksum result directly after *
        Returns: 
        {bool} -- True if checksum is correct, False otherwise
        """
        try:
            #take a substring between $ and *
            s1 = data.split('*')[0]
        except:
            #if we can't find a * the data is corrupt; return false
            return False

        #compute char wise checksum
        checksum = 0
        for char in s1:
            checksum ^= ord(char)
        #convert to hex for comparison with checksum value in str
        checksum = hex(checksum)
	
        try:
            checksum_str = "0x" + data.split("*")[1]
        except:
            return False
        checksum_int = int(checksum_str, 16)
        hex_checksum = hex(checksum_int)

        if checksum != hex_checksum:
            return False
        else:
            return True	


def transmitGPS():
    '''
    This function sets up the ROS node, serial connection and NMEA Parser. It then repeatedly checks for new data received 
    on the serial line and converts this data to GPS coordinates if available. The frequency of checks depends on the 
    global variable ROS_REFRESH_RATE.
    '''

    with GPSSerialInterface(ROS_REFRESH_RATE, ['RMC'], "/dev/serial0", 9600, 3000) as gps_interface:
        gps_interface.configureGPS() 

        msg = NavSatFix()
        pub = rospy.Publisher('/gps/gps_data', NavSatFix, queue_size=10)
        rospy.init_node('gps_data', anonymous=True)
        rate = rospy.Rate(ROS_REFRESH_RATE)
        parser = NMEAParser('NMEA_0183')

        while not rospy.is_shutdown():
            received_data = gps_interface.readSerialInput()                 # Get serial data
            rospy.loginfo(received_data)
            try:
                    [latitude, longitude] = parser.getGPSLocation(received_data)    # Parse data and set message fields
                    msg.latitude = latitude                                         # Publish coordinates
                    msg.longitude = longitude
                    pub.publish(msg)	
                    rate.sleep()                                                    # Sleep until next check
            except:
                    rate.sleep()
                    pass

if __name__ == '__main__':
    try:
        transmitGPS()
    except rospy.ROSInterruptException:
        pass

