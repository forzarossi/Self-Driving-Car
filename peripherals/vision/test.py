import cv2
from lane_detection import lane
import matplotlib.pyplot as plt

img = cv2.imread('lane.png')
plt.imshow(img)
plt.show()
x = lane(img)
print(x)