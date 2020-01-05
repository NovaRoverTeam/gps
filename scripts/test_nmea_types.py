#!/usr/bin/python
'''Test script to set Hz of GPS module to 10 Hz and recieve RMC data only'''

import serial
import time

port = "/dev/serial0"

def send_command(ser, command):
    ser.write(b'$')
    ser.write(command)
    checksum = 0
    for char in command:
        checksum ^= ord(char)

    print(type(checksum))
    ser.write(b'*')
    ser.write(bytes('{:02x}'.format(checksum).upper()))
    ser.write(b'\r\n')

ser = serial.Serial(port, 9600, timeout=3000)

GGL = b'PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
RMC = b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
VTG = b'PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
GGA = b'PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
GSA = b'PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0'
GSV = b'PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0'

send_command(ser, u4)

while True:
    data = ser.readline()
    print(data)
