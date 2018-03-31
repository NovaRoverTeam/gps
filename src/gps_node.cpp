/***************************************************************************************************
* NOVA ROVER TEAM - URC2018
* This code is a ROS package which periodically retrieves data from a Waveshare NEO-7M-C GPS module. 
* The data is retrieved via UART, and is parsed through to obtain the latitude and longitude 
* GPS coordinates.
*
* The GPS module being used:
* www.robotshop.com/ca/en/uart-neo-7m-c-gps-module.html
*
* Author: Andrew Stuart
* Last Modified by: Andrew Stuart (30/11/2017)
***************************************************************************************************/

/***************************************************************************************************
* INCLUDES, DECLARATIONS AND GLOBAL VARIABLES
***************************************************************************************************/
#include "ros/ros.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <gps/Gps.h>
#include <sstream>

#define LOOP_HERTZ 1		// GPS sends data every ~1 second

//char GPS_data_array[40];	// Holds data from the GLL line to be parsed
float latitude, longitude;


/**************************************************************************************************
* DEGREES-MINUTES-SECONDS TO DECIMAL DEGREES CONVERSION FUNCTION
*
* Converts a number in decimal-minutes-seconds to decimal degrees and returns it.
*
* Inputs: int deg - the degree value of the coordinate
*         int min - the minutes value of the coordinate
*         float sec - the seconds value of the coordinate, should be a decimal
*         int dir - the direction of the coordinate as a positive or negative multiplier (1/-1).
*
* Output: float DecDeg - the decimal degrees representation of the DMS number
**************************************************************************************************/
float ConvertDMSToDD(int deg, int min, float sec, int dir) {
  float DecDeg = (float)dir * (((float)deg) + ((float)min/60) + (sec/3600));
  return DecDeg;
}

/***************************************************************************************************
* DATA PARSING FUNCTIOM
* This function parses the GLL line sent by module for the GPS coordinates, if it is a valid reading.
* The output of this function is applied directly to the global variables "latitude" and "longitude".
* 
* This function assumes that the GLL output format is fixed.
*
* Inputs: char *GPS_data_array - The character array containing all required data from the GLL line.
*
* todo: modify function to return obtained coordinates using a float array rather than modifying
* global variables directly.
***************************************************************************************************/
void ProcessGPSData(char *GPS_data_array) {
  int lat_deg, lat_min, long_deg, long_min, lat_dir, long_dir;
  float lat_sec, long_sec;

  // Determine latitude in degrees-minutes-seconds (DMS) format.
  // "GPS_data_array[X]-'0'" converts a number in character format to int, allowing for calculations
  lat_deg = ((GPS_data_array[1]-'0')*10) + (GPS_data_array[2]-'0');
  lat_min = ((GPS_data_array[3]-'0')*10) + (GPS_data_array[4]-'0');
  lat_sec = ((GPS_data_array[6]-'0')*10) + (GPS_data_array[7]-'0') + ((float)(GPS_data_array[8]-'0')/10) + ((float)(GPS_data_array[9]-'0')/100) + ((float)(GPS_data_array[10]-'0')/1000);
  
  // Determine latitude direction by checking if character following DMS value is North (N) / South (S).
  if(GPS_data_array[12]=='N') {
    lat_dir = 1;  // Let 1 represent North, -1 represent South
  }
  else {
    lat_dir = -1;
  } 
  
  // Determine longitude in DMS format
  long_deg = ((GPS_data_array[14]-'0')*100) + ((GPS_data_array[15]-'0')*10) + (GPS_data_array[16]-'0');
  long_min = ((GPS_data_array[17]-'0')*10) + (GPS_data_array[18]-'0');
  long_sec = ((GPS_data_array[20]-'0')*10) + (GPS_data_array[21]-'0') + ((float)(GPS_data_array[22]-'0')/10) + ((float)(GPS_data_array[23]-'0')/100) + ((float)(GPS_data_array[24]-'0')/1000);
  
  // Determine longitude direction
  if(GPS_data_array[26]=='E') {
    long_dir = 1;
  }
  else {
    long_dir = -1;
  }
  // Convert DMS to DD
  latitude = ConvertDMSToDD(lat_deg, lat_min, lat_sec, lat_dir);
  longitude = ConvertDMSToDD(long_deg, long_min, long_sec, long_dir);
}

/***************************************************************************************************
* MAIN FUNCTION
* Sets up UART connection and periodically reads data on connection for new GPS data.
* This parsed data is then published via ROS.

* Published messages:
* Valid Reading - Two message are sent: one containing the latitude coordinates (msg.latitude) and
* one which contains the longitude coordinates (msg.longitude). Both coordinates are in decimal 
* degrees format.
* Invalid Reading - Nothing. A warning will be printed to the ROS info stream to inform the user.
***************************************************************************************************/

int main(int argc, char **argv)
{
  setenv("WIRINGPI_GPIOMEM","1",1);	// Set environmental var to allow non-root access to GPIO pins
  ros::init(argc, argv, "gps");		// Initialise ROS package
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<gps::Gps>("/gps/gps_data", 1000);
  ros::Rate loop_rate(LOOP_HERTZ);	// Define loop rate
  int fd;
  char uartChar;			// The character retrieved from the UART RX buffer
  unsigned int enable_data_capture = 0;	// Boolean which is set to 1 (TRUE) when GLL line has been located
  unsigned int data_is_valid, array_index;
  char data_capture_array[40];
  
  // Attempt to open UART
  if((fd=serialOpen("/dev/ttyS0",9600))<0) {	// 9600 baudrate determined from module datasheet
    printf("Unable to open serial device\n");
    return 1;
  }

  // Attempt to setup WiringPi
  if(wiringPiSetup() == -1) {
    printf("Cannot start wiringPi\n");
  }

  while (ros::ok())
  {
    gps::Gps msg;

    while(1){
      // If there is new UART data...
      if(serialDataAvail(fd)) {
        // ... retrieve the next character
        uartChar = serialGetchar(fd);		
        
        // If the character is "L", it must be a GLL line
        if(uartChar=='L') {			
          enable_data_capture = 1;		// So we save the data by enabling data capture
          array_index = 0;
          data_is_valid = 1;			// Assume that the reading is valid until otherwise
        }
        else {
          // If we are in the GLL line...
          if(enable_data_capture) {
            // ... check for EOL char or validity character
            switch(uartChar) {			
              case '\r':			// EOL found, GLL line over; end data capture
                enable_data_capture = 0;
                // If the data is valid...
                if(data_is_valid) {		
                  ProcessGPSData(data_capture_array);  // ...obtain coordinates from the data
                }
                else {
                  // Otherwise, inform user that coordinates could not be obtained
                  ROS_INFO("GPS module cannot locate position.");
                }
                break;
              case 'V':				// If the reading is invalid, the module sends a "V" char
                data_is_valid = 0;		// Set valid reading boolean to 0 (FALSE)
              default:
                data_capture_array[array_index] = uartChar;	// Save all other data if data capture is enabled
                array_index++;
              }
          }
        }
        fflush(stdout);				// Flush buffer
      }
      else {
        // Do not publish message on startup before data has been received
        if((latitude==0)&&(longitude==0)) {
          data_is_valid = 0; 
        }
        break;					// No data available; end loop to free CPU
      }
    }
    
    // Publish readings
    if(data_is_valid) {
      msg.latitude = latitude;
      msg.longitude = longitude;
      ROS_INFO("Latitude: %f, Longitude: %f", latitude, longitude);
      sensor_pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
