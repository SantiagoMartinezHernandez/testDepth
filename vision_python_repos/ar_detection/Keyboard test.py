import cv2
import numpy as np


cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    if img is None:
        break
    originalImg = img.copy()
    _, img = cap.read()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    sharpen = cv2.filter2D(blur, -1, sharpen_kernel)

    thresh = cv2.threshold(sharpen,160,255, cv2.THRESH_BINARY_INV)[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    close = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    cnts = cv2.findContours(close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]

    min_area = 0
    max_area = 1500
    image_number = 0
    for c in cnts:
        area = cv2.contourArea(c)
        if area > min_area and area < max_area:
            x,y,w,h = cv2.boundingRect(c)
            ROI = img[y:y+h, x:x+w]
            cv2.imwrite('ROI_{}.png'.format(image_number), ROI)
            cv2.rectangle(img, (x, y), (x + w, y + h), (36,255,12), 2)
            image_number += 1

    cv2.imshow('sharpen', sharpen)
    cv2.imshow('close', close)
    cv2.imshow('thresh', thresh)
    cv2.imshow('image', img)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break