#!/usr/bin/env python3

"""Testing Python Script for receiving Serial data from MCU """

import serial

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyAMA1', 9600, timeout=2)
    ser.flush()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
