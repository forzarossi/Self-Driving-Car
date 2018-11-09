############################################################
# file: color_detect.py                                          #
# author: Marc Rossi                                       #
# email: marcrossi@umass.edu                               #
#                                                          #
#          MUST BE RUN WITH SUDO AT THE MOMENT             #
############################################################
from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import numpy as np

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
    image_array = np.asarray(image)
    process_image(image_array)

def process_image(image_array):
    find_black_indicies(image_array)
    find_yellow_indicies(image_array)

    # if(len(find_red_indicies(image_array))>100):
    #     stopsign
    to_filtered_image(image_array, "red")

########################################################################
#    Check where the pixel is equal to 0,0,0 with a threshold of 50    #
########################################################################
def find_black_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array >(0,0,0), image_array < (50,50,50))==True, axis=-1))

def find_red_indicies(image_array):
    return np.where(np.all(np.logical_and(image_array >(0,0,0), image_array < (50,50,50))==True, axis=-1))

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
        alpha = ~np.all(np.logical_and(image_array > (215,215,50), image_array < (250,250,100)) == True, axis=-1) * 255
        rgba = np.dstack((np.all(np.logical_and(image_array > (220,220, 50), image_array < (250,250, 180)) == True, axis=-1), alpha)).astype(
            np.uint8)

        Image.fromarray(rgba).save(color + ".png")

    if color is "red":
        alpha = ~np.all(np.logical_and(image_array > (100, 0, 0), image_array < (200, 70, 70)) == False, axis=-1) * 255
        rgba = np.dstack((np.all(np.logical_and(image_array > (100, 0, 0), image_array < (200, 70, 70)) == False, axis=-1),
                          alpha)).astype(
            np.uint8)

        Image.fromarray(rgba).save(color + ".png")

take_image()




