#!/usr/bin/env python3
import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    while True:
       led_number = 29
       ser.write(str(led_number).encode('utf-8'))
       line = ser.readline().decode('utf-8').rstrip()
       print(line)
       time.sleep(1)









