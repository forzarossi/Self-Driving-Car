#!/usr/bin/python
from io import BytesIO
from picamera import PiCamera
from PIL import Image
from time import sleep
import numpy as np
import serial
import time
import imp

color_detect = imp.load_source('filename','/home/merry_student/Self-Driving-Car/peripherals/camera/color_detect.py')
lane_follow = imp.load_source('filename','/home/merry_student/Self-Driving-Car/peripherals/camera/lane_follow.py')
#lane_follow2 = imp.load_source('filename','/home/merry_student/Self-Driving-Car/peripherals/vision/lane_follow.py')
ser = serial.Serial('/dev/ttyACM0')

ser.write(str('start\n').encode('utf-8')) #start
camera = PiCamera()
camera.resolution = (640,480)
stop_value = False

while True:
   start_time = time.time()
   stream = BytesIO()
   camera.start_preview()
   camera.capture(stream, format='jpeg')
   stream.seek(0)
   image = Image.open(stream)
   image = image.rotate(180)
   finished = time.time()
   print(finished - start_time)
   stop_value = color_detect.process_image(image,stop_value)
   ser = serial.Serial('/dev/ttyACM0')
   if stop_value == True:
     ser.write(str('stop\n').encode('utf-8'))
   else:
     ser.write(str('start\n').encode('utf-8'))
   ser.flushInput()
   #ser.write(str(lane_follow2.crop_image(np.asarray(image)))+'/n'.encode('utf-8'))
   ser.write(str(lane_follow.crop_image(image, stop_value))+'\n'.encode('utf-8'))
