#!/usr/bin/env python3
import serial
import time

if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    ser.reset_input_buffer()
    
    while True:
        ser.write(b"1")
        #print("led on")
        time.sleep(3)
        line = ser.readline().decode('utf-8').rstrip()
        print(line)
        
        ser.write(b"0")
        #print("led off")
        time.sleep(3)
        line = ser.readline().decode('utf-8').rstrip()
        print(line)