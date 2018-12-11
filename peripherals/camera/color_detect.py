############################################################
# file: color_detect.py                                          #
# author: Marc Rossi                                       #
# email: marcrossi@umass.edu                               #
#                                                          #
#          MUST BE RUN WITH SUDO AT THE MOMENT             #
############################################################
# from io import BytesIO
# from time import sleep
# from picamera import PiCamera
from PIL import Image
import numpy as np
import serial

##############################
# Takes image using PiCamera #
# Converts image to np.array #
##############################
def take_image():
    stream = BytesIO()
    camera = PiCamera()
    camera.start_preview()
    sleep(2)
    camera.capture(stream, format='jpeg')
    stream.seek(0)
    image = Image.open(stream)
    image = image.rotate(180)
    process_image(image)

def process_image(image):
    image_array = np.asarray(image)
    cropped_image = image.crop(((0, 350, 640, 480)))
    cropped_array = np.asarray(cropped_image)
    
    if(len(find_green_indicies(cropped_array)[0]) > 850):
        print("Go")
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(b'1')
    elif(len(find_red_indicies(cropped_array)[0]) > 40000):
        print("Stop")
        ser = serial.Serial('/dev/ttyACM0')
        ser.write(b'stop')

########################################################################
#    Check where the pixel is equal to 0,0,0 with a threshold of 50    #
########################################################################
def find_green_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array > (200, 200, 200), image_array < (250, 255, 250)) == True, axis=-1))

def find_black_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array >(0,0,0), image_array < (50,50,50)) == True, axis=-1))

def find_red_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array > (150, 50, 50), image_array < (255, 150, 150)) == True, axis=-1))

def find_yellow_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array >(215,215,50), image_array < (250,250,100))==True, axis=-1))

########################################################################
# Saves image with specific color filtered out for visualization       #
########################################################################
def to_filtered_image(image_array, color):
    if color is "black":
        alpha = ~np.all(np.logical_and(image_array > (0, 0, 0), image_array < (70, 70, 70)) == False, axis=-1) * 255
        rgba = np.dstack((np.all(np.logical_and(image_array > (0, 0, 0), image_array < (70, 70, 70)) == False, axis=-1),
                          alpha)).astype(
            np.uint8)

        Image.fromarray(rgba).save(color + ".png")

    if color is "yellow":
        alpha = ~np.all(np.logical_and(image_array > (215,215,50), image_array < (250,250,100)) == False, axis=-1) * 255
        rgba = np.dstack((np.all(np.logical_and(image_array > (220,220, 50), image_array < (250,250, 180)) == False, axis=-1), alpha)).astype(
            np.uint8)

        Image.fromarray(rgba).save(color + ".png")

    if color is "red":
        alpha = ~np.all(np.logical_and(image_array > (150, 50, 50), image_array < (255, 150, 150)) == True, axis=-1) * 255
        rgba = np.dstack((np.all(np.logical_and(image_array > (150, 50, 50), image_array < (255, 150, 150)) == True, axis=-1),
                          alpha)).astype(
            np.uint8)

        Image.fromarray(rgba).save(color + "isolated.png")

#take_image()




