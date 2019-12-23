# ROS GPS Package
This code is a ROS package which periodically retrieves data from a Adafruit
Ultimate v3 GPS module.
The data is retrieved via UART, and is parsed through to obtain the latitude and longitude GPS coordinates.

The GPS module being used: https://www.adafruit.com/product/746

## Dependencies:

- WiringPi (http://wiringpi.com/download-and-install/)

## Physical Connections:

The module should be connected to the Raspberry Pi as such:

  GPS Module:     Raspberry Pi:
  * Vcc        -->  3.3V
  * GND        -->  Ground
  * TX         -->  UART0 RX
  * RX         -->  UART0 TX
  
For more information about pin assignments, see the package.xml  

## Publications:

Topic:       **/gps/gps_data**<br />
Msg type:    NavSatFix msg  <br />
Description: Provides latitude and longitude readings from the GPS module.
