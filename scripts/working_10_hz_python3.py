#!/usr/bin/python3
'''Test script to set Hz of GPS module to 10 Hz and recieve RMC data only'''

import serial
import time
import adafruit_gps

port = "/dev/serial0"

ser = serial.Serial(port, 9600, timeout=3000)
gps = adafruit_gps.GPS(ser, debug=False)
gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
time.sleep(1)
gps.send_command(b'PMTK220,100')

while True:
    data = ser.readline()
    print(data)

