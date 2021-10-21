import sys
import os

file_path = os.path.join(os.path.dirname(__file__), '..')
file_dir = os.path.dirname(os.path.realpath('__file__'))
sys.path.insert(0, os.path.abspath(file_path))

import cv2
import numpy as np
import realsense_depth.image_grid as gi

image_path = r'images/image.jpg'

img = cv2.imread(image_path)
cv2.imshow('normal', img)

images, grid = gi.createGrid(img, 2, 2)
cv2.imshow('images[0][0]', images[0][0])
cv2.imshow('images[0][1]', images[0][1])
cv2.imshow('grid', grid)

cv2.waitKey(0)
cv2.destroyAllWindows()