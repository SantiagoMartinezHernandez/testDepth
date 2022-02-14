import cv2
import numpy as np

def gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def threshold(img):
    _, threshold = cv2.threshold(img, 127, 255, cv2.THRESH_OTSU)
    return threshold

def edges(img):
    canny = cv2.Canny(cv2.GaussianBlur(gray(img), (5,5), cv2.BORDER_DEFAULT), 50, 200)
    return np.bitwise_or(img, canny[:,:,np.newaxis])
