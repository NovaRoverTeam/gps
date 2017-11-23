/***************************************************************************************************
* NOVA ROVER TEAM - URC2018
* This code is a ROS package which periodically retrieves data from a Waveshare NEO-7M-C GPS module. 
* The data is retrieved via UART, and is parsed through to obtain the latitude and longitude 
* GPS coordinates.
*
* The GPS module being used:
* www.robotshop.com/ca/en/uart-neo-7m-c-gps-module.html
*
* 
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
#include "std_msgs/String.h"
#include <sstream>

#define LOOP_HERTZ 1		// GPS sends data every ~1 second

char GPS_data_array[40];	// Holds data from the GLL line to be parsed
char proc_data_array[24];       // Holds parsed latitude and longitude data

/***************************************************************************************************
* DATA PARSING FUNCTIOM
* This function parses the GLL line sent by module for the GPS coordinates, if it is a valid reading.
*
* This function does not assume that the data is fixed length and instead loops through all numbers
* in the chance that the module does not send a leading zero digit, thus making it variable length.
***************************************************************************************************/
void ProcessGPSData() {
  unsigned int raw_data_index = 1;	// Index 0 contains a comma, so skip
  unsigned int proc_data_index = 0;

  // Loop through first field to retrieve all latitude data
  while(GPS_data_array[raw_data_index]!=',') {				// Comma indicates end of field
    proc_data_array[proc_data_index] = GPS_data_array[raw_data_index];	// Save required character/digit
    raw_data_index++;							//
    proc_data_index++;							// Inc. index of arrays
  }
  raw_data_index++;
  proc_data_array[proc_data_index] = GPS_data_array[raw_data_index];	// Retrieves letter heading (N/S)

  // Move to longitude data
  raw_data_index += 2;
  proc_data_index = 0;
  while(GPS_data_array[raw_data_index]!=',') {				// ""     ""     "" 
    proc_data_array[proc_data_index] = GPS_data_array[raw_data_index];
    raw_data_index++;
    proc_data_index++;
  }
  raw_data_index++;
  proc_data_array[proc_data_index] = GPS_data_array[raw_data_index];	// Retrieves letter heading (E/W)
}

/***************************************************************************************************
* MAIN FUNCTION
* Sets up UART connection and periodically reads data on connection for new GPS data.
* This parsed data is then published via ROS.

* Published message:
* Valid Reading - xxxx.xxxxxYzzzzz.zzzzzA, where x are numerical latitude coordinates, Y is N/S, z
* are numerical longitude coordinates and A is E/W.
* Invalid Reading - INV, the reading is invalid and should be ignored.
***************************************************************************************************/

int main(int argc, char **argv)
{
  setenv("WIRINGPI_GPIOMEM","1",1);	// Set environmental var to allow non-root access to GPIO pins
  ros::init(argc, argv, "GPS");		// Initialise ROS package
  ros::NodeHandle n;
  ros::Publisher sensor_pub = n.advertise<std_msgs::String>("GPS", 1000);
  ros::Rate loop_rate(LOOP_HERTZ);	// Define loop rate
  int fd;
  char uartChar;			// The character retrieved from the UART RX buffer
  unsigned int enable_data_capture = 0;	// Boolean which is set to 1 (TRUE) when GLL line has been located
  unsigned int data_is_valid, array_index;
  
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
    std::stringstream ss;
    std_msgs::String msg;

    while(1){
      if(serialDataAvail(fd)) {			// If there is new UART data...
        uartChar = serialGetchar(fd);		// ... retrieve the next character
        
        if(uartChar=='L') {			// If the character is "L", it must be a GLL line
          enable_data_capture = 1;		// So we save the data by enabling data capture
          array_index = 0;
          data_is_valid = 1;			// Assume that the reading is valid until otherwise
        }
        else {
          if(enable_data_capture) {		// If we are in the GLL line...
            switch(uartChar) {			// ... check for EOL char or validity character
              case '\r':			// EOL found, GLL line over; end data capture
                enable_data_capture = 0;
                if(data_is_valid) {		// If the data is valid...
                  ProcessGPSData();		// ... parse data for latitude/longitude data
                  ss << proc_data_array;
                }
                else {
                  ss << "INV";			// Otherwise send invalid message reading
                }
                break;
              case 'V':				// If the reading is invalid, the module sends a V
                data_is_valid = 0;		// Set valid reading boolean to 0 (FALSE)
              default:
                GPS_data_array[array_index] = uartChar;	// Save data if data capture is enabled
                array_index++;
              }
          }
        }
        fflush(stdout);				// Flush buffer
      }
      else {
        break;					// No data available; end loop to free CPU
      }
    }
    
    // Publish readings
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    sensor_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
