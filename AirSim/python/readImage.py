import cv2
import numpy as np
import time
 
img = cv2.imread('result.png',0)
img = (255 - cv2.bitwise_not(img))/255