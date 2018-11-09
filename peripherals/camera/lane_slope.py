import numpy as np
from PIL import Image
from picamera import PiCamera

def white_lane(array):
    threshold = 190
    if (array[0] > threshold and array[1] > threshold and array[2] > threshold):
        return True
    return False

def lane_coords(image, coords):
    h = image.shape[0]
    w = image.shape[1]
    for y in range(0, h):
        first = False
        for x in range(0, w):
            GBR = image[y, x]
            if (white_lane(GBR) and first == False):
                coords.append(x)
                coords.append(y)
                first = True

white_line = []
stream = BytesIO()
camera = PiCamera()
camera.start_preview()
sleep(2)
camera.capture(stream, format='jpeg')
stream.seek(0)
image = Image.open(stream)
image.crop((400,440,50,500)).save
image_array = np.asarray(image)
lane_coords(image_array, white_line)

x0, y0, x1, y1 = white_line[0], white_line[1], white_line[len(white_line)-2], white_line[len(white_line)-1]
m = float(y1-y0)/float(x1-x0)
b = y1 - (m * x1)

print ("Slope: {} \nY-Intercept: {}".format(m,b))