#!/usr/bin/python
import serial
import peripherals.camera.color_detect as color_detect
import time

ser = serial.Serial('/dev/ttyACM0')
ser.write(b'start')

while True:
    time.sleep(.1)
    color_detect.take_image()
