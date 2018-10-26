from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import numpy as np

stream = BytesIO()
camera = PiCamera()
camera.start_preview()
sleep(2)
camera.capture(stream, format='jpeg')
stream.seek(0)
image = Image.open(stream)
image_array = numpy.asarray(image)

zeros = np.zeros((100, 100), dtype=np.uint8)
zeros[:5,:5] = 255

indices = np.where(zeros == [255])
print indices
coordinates = zip(indices[0], indices[1])
print coordinates
