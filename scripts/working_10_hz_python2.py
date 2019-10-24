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
send_command(ser, b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
#Need to do sleep between commands so GPS accepts all of the commands.
time.sleep(1)
send_command(ser, b'PMTK220,100')

while True:
    data = ser.readline()
    print(data)
