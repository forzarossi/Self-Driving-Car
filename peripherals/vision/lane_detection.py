from io import BytesIO
from time import sleep
from picamera import PiCamera
from PIL import Image
import numpy as np

'''
sudo chmod 777 /dev/vchiq
'''

def camera():
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
	image.save('lane.png')
	image_array = np.asarray(image)
	im = Image.fromarray(image_array[::-1,::-1,:])
	im.save('up_side_down.png')
	return image_array

def lane(img, bar_width = 20, bar_pos = 160, gamma = 0.5):
	'''
	This function reads an image from camera, detects the lanes on the road,
	and return the distance between the current location and the expected
	location.

	Args:
	img: a numpy array of shape (H,W)

	Returns:
	delta: An integer representing the pixel difference between the current 
	location and the expected location. Positive value means the current 
	location is on the right side of the expected location, and negative 
	value means the current location is on the left side of the expected 
	location.
	'''
	H,W,_ = img.shape
	# make the bar
	bar = img[H-(bar_pos+bar_width):H-bar_pos,:,:]
	# some variables to record the lane positions
	n_left = 0
	n_right = 0
	left = -1
	right = -1
	cur = -1
	pre = -1
	# threshold: if the sum of a column values is larger than the 
	# threshold, then that column is in the white lane
	threshold = 255*bar_width*3*gamma
	# goes over the bar from right to left
	for col in range(W-1,-1,-1):
		# check sum and store boolean value
		if np.sum(bar[:,col]) < threshold:
			cur = 0
		else:
			cur = 1
		# record the number of such edges we have seen
		if pre == 0 and cur == 1:
			n_left += 1
		if pre == 1 and cur == 0:
			n_right += 1
		# the first (pre=1,cur=0) edge is the right lane 
		if n_right == 1:
			right = col
		# the second (pre=0,cur=1) edge is the left lane
		if n_left == 2:
			left = col
		# now the current value is the previous value
		pre = cur
	mid = W/2
	return left,right, mid - (left + (right-left)/2)

if __name__ == '__main__':
	img = camera()
	#print lane(img)

