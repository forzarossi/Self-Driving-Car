#!/usr/bin/python
import numpy as np
from PIL import Image
from picamera import PiCamera
import serial

def crop_image(image):
    cropped = image.crop((0, 270, 640, 300))
    image_array = np.asarray(cropped)
    w, h = cropped.size
    calculate_error(image_array, h, w)

def is_black(RGB):
    threshold = 160
    if(RGB[0] < threshold and RGB[1] < threshold and RGB[2] < threshold):
        return True #when black
    return False #when other

def calculate_error(image_array, h, w):
    for y in range(0, h):
        swapped_index = []
        for x in range(w-1, 0, -1):
            if(is_black(image_array[y][x]) != is_black(image_array[y][x-1])):
                swapped_index.append((y,x))
                print("Added")
                if len(swapped_index) == 2:
                    break;
        if len(swapped_index) == 2:
            break;
    distance = swapped_index[0][1] - swapped_index[1][1]
    print(distance)
    send_correction(distance)

def send_correction(image_middle):
    correct_middle = 430
    correction = correct_middle - image_middle
    ser = serial.Serial('/dev/ttyACM0')
    ser.write(correction)

#im = Image.open("/Users/marcrossi/Desktop/Fall 2018/503/Self-Driving-Car/peripherals/camera/road.jpg")
#crop_image(im)
