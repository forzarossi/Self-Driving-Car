############################################################
# file: camera.py                                          #
# author: Marc Rossi                                       #
# email: marcrossi@umass.edu                               #
#                
#          MUST BE RUN WITH SUDO AT THE MOMENT     
############################################################

from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import numpy as np

##############################
# Takes image using PiCamera #
##############################
stream = BytesIO()
camera = PiCamera()
camera.start_preview()
sleep(2)
camera.capture(stream, format='jpeg')

##############################
# Converts image to np.array #
##############################
stream.seek(0)
image = Image.open(stream)
image_array = np.asarray(image)

######################################################
# check where the pixel is equal to 255(black)       #
# TODO: Check where values > some threshold of black #
######################################################
indices = np.where(image_array == [255])

##########################################################
# Records indicies and the coordinates of those indicies #
##########################################################
print indices
coordinates = zip(indices[0], indices[1])
print coordinates
