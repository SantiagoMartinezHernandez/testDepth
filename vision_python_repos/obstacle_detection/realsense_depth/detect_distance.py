# -*- coding: utf-8 -*-
"""detect_distance

Cristóbal Arroyo

Este es un código de la cámara de profundidad Depth IntelRealsense D435i
Es necesario configurar los parámetros del código desde la límea 22. Estos son

Parámetros
----------

MOUSE_DEBUG
    Inicializa la lectura de profundidad con un mouse, puede hacer más accesible
    la lectura de datos manual de la profundida.

CHARACTERIZATION
    Si está activa, las columnas se verán todo el tiempo

point
    El lugar donde comienza el punto si MOUSE_DEBUG está activo.

grilla_cols
    El número de columnas de la grilla en la transmisión de la cámara

grilla_rows
    El número de filas de la grilla en la transmisión de la cámara

umbral_distancia
    La distancia mínima (en mm) para determinar si un pixel podría hacer
    parte de un obstáculo o no

umbral_proporcion
    La proporción de pixeles que deben estar por encima del umbral de distancia
    para que un segmento de la partición de la grilla sea considerado un obstáculo.
    Es decir, que si de 800 pixeles, 600 superan el umbral de distancia entonces
    se mostrará dicho segmento como un obstáculo (proporción de 0.75)

Ejemplo
-------

Para correr el código es necesario tener instaladas las librerías OpenCV y Pyrealsense y la cámara
IntelRealsense D435i conectada al dispositivo. Para ello, abra una terminal powershell o CMD normal

(Win+X -> Windows Powershell)

Y ejectue los comandos:

python -m pip install opencv-python
python -m pip install pyrealsense2

Posteriormente es posible ejecutar el código.

"""

import sys
import os

file_path = os.path.join(os.path.dirname(__file__), '..')
file_dir = os.path.dirname(os.path.realpath('__file__'))
sys.path.insert(0, os.path.abspath(file_path))

capture_path = os.path.dirname(__file__)

import cv2
import numpy as np
import pyrealsense2 as rs
import math
from realsense_depth.depth_module import *
from realsense_depth.image_grid import *

MOUSE_DEBUG = True
CHARACTERIZATION = True
point = (400,300)
cap_index = 0

grilla_cols = 16
grilla_rows = 1
umbral_distancia = 850
umbral_proporcion = 0.6

def show_distance(event, x, y, args, params):
    global point
    point = (x, y)

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

# Se crea un evento de mouse
cv2.namedWindow("color_frame")
cv2.setMouseCallback("color_frame", show_distance)

while True:
    # Se obtiene la información de la depth
    ret, depth_frame, color_frame, colorized_depth = dc.get_frame()
    cv2.imshow("original_color_frame", color_frame)

    # Se crea una grilla para hacer los calculos por cada segmento de manera independiente
    depth_lst, depth_grid = createGrid(depth_frame,grilla_rows,grilla_cols)
    cv2.imshow("colorized_depth", colorized_depth)

    # Se calcula la proporción de cada una, y se muestra el obstáculo en pantalla

    for i in range(grilla_rows):
        for j in range(grilla_cols):
            if CHARACTERIZATION:
                cv2.rectangle(color_frame, ((x//grilla_cols)*j,(y//grilla_rows)*i), ((x//grilla_cols)*(j+1),(y//grilla_rows)*(i+1)), (0,0,255), 5)
            else:
                proporcion_img = proporcion(depth_lst[i][j])
                if verificacion(proporcion_img):       
                    # print(f"proporcion [{i}][{j}] -> {proporcion_img}")
                    cv2.rectangle(color_frame, ((x//grilla_cols)*j,(y//grilla_rows)*i), ((x//grilla_cols)*(j+1),(y//grilla_rows)*(i+1)), (0,0,255), 5)


    # A continuación se muestra la distancia hacia un punto específico
    if (MOUSE_DEBUG):
        cv2.circle(color_frame, point, 4, (0, 0, 255))
        distance = depth_frame[point[1], point[0]]

        cv2.putText(color_frame, "{}mm".format(distance), (point[0], point[1]), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 2)
    cv2.imshow("color_frame", color_frame)

    """
    # Con esta función se guardan los frames

    result1.write(cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET))
    result2.write(color_frame)
    """

    # Descomentar lo siguiente en modo debug

    # cv2.imshow("depth frame", cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET))
    # cv2.imshow("depth colorizer frame", colorized_depth)
    key = cv2.waitKey(1)
    if key == ord('s'):
        print(f"Guardando en {capture_path}\\captures\\")
        cv2.imwrite(capture_path+f'\\captures\\color_{cap_index}.jpg',color_frame)
        cv2.imwrite(capture_path+f'\\captures\\depth_{cap_index}.jpg',colorized_depth)
        cap_index+=1
    if key == ord('q') or key == 27:
        break

cv2.destroyAllWindows()
