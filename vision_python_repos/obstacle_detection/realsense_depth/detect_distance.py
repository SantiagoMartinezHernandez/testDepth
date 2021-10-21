import sys
import os

file_path = os.path.join(os.path.dirname(__file__), '..')
file_dir = os.path.dirname(os.path.realpath('__file__'))
sys.path.insert(0, os.path.abspath(file_path))

import cv2
import numpy as np
import pyrealsense2 as rs
from realsense_depth.depth_module import *
from realsense_depth.image_grid import *

grilla_cols = 16
grilla_rows = 16
umbral_distancia = 850
umbral_proporcion = 0.6

def proporcion(imagen):
    """
    Calcula la proporción de profundidad de un depth stream por encima de cierto umbral de distancia

    Parameters
    ----------
    imagen : array
        La imagen (un solo canal de profundidad) al cual se le calcula la proporción mediante funciones de numpy

    Returns
    -------
    int : La proporción de profundidad en la imagen
    """
    tot = len(imagen)*len(imagen[0])
    imagenNumpy = np.array(imagen)
    count = np.sum(np.logical_and(imagenNumpy <= umbral_distancia, imagenNumpy != 0))
    return count/tot
    
def verificacion(prop):
    """
    Verifica si la proporción
    """
    if prop >= umbral_proporcion:
        return True
    return False

dc = DepthCamera()

ret, depth_frame, color_frame, colorized_depth = dc.get_frame()
x = len(color_frame[0])
y = len(color_frame)

print(f"Resolución: {x}x{y}")

"""
# Lo siguiente se utiliza para guardar registro de las capturas

size1 = (len(depth_frame[0]), len(depth_frame))
result1 = cv2.VideoWriter('depth.avi', 
                         cv2.VideoWriter_fourcc('M','J','P','G'),
                         10, size1)

size2 = (len(color_frame[0]), len(color_frame))
result2 = cv2.VideoWriter('color.avi', 
                         cv2.VideoWriter_fourcc('M','J','P','G'),
                         10, size2)
"""

while True:
    ret, depth_frame, color_frame, colorized_depth = dc.get_frame()
    cv2.imshow('normal', color_frame)
    depth_lst, depth_grid = createGrid(depth_frame,grilla_rows,grilla_cols)
    cv2.imshow('colorized depth', colorized_depth)
    for i in range(grilla_rows):
        for j in range(grilla_cols):
            proporcion_img = proporcion(depth_lst[i][j])
            # print(f"Matriz: {len(depth_lst[0])}x{len(depth_lst)}")
            if verificacion(proporcion_img):       
                print(f"proporcion [{i}][{j}] -> {proporcion_img}")
                cv2.rectangle(color_frame, ((x//grilla_cols)*j,(y//grilla_rows)*i), ((x//grilla_cols)*(j+1),(y//grilla_rows)*(i+1)), (0,0,255), 5)


    """
    # Con esta función se guardan los frames

    result1.write(cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET))
    result2.write(color_frame)
    """

    # Descomentar lo siguiente en modo debug

    # cv2.imshow("depth frame", cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET))
    # cv2.imshow("depth colorizer frame", colorized_depth)

    cv2.imshow("Color frame", color_frame)
    key = cv2.waitKey(1)
    if key == 27:
        """
        result1.release()
        result2.release()
        """
        break

cv2.destroyAllWindows()
