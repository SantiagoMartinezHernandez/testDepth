"""characterization

Cristóbal Arroyo

Para correr el código es necesario haber instalado anteriormente OpenCV.
Para ello, abra una terminal powershell o CMD normal

(Win+X -> Windows Powershell)

Y ejectue el comando:

python -m pip install opencv-python
"""

import image_grid
import cv2

cap = cv2.VideoCapture(0)

cap_index = 0
while cap.isOpened():
    _, frame = cap.read()
    _, frame = image_grid.createGrid(frame,1,20)
    cv2.imshow('Capture',frame)

    key = cv2.waitKey(1) & 0xFF

    """
    Al presionar la tecla 's' se guardará en la carpeta de captures. Debe tener en cuenta
    que es necesario que el código esté en el directorio raíz del workspace para que funcione

    IMPORTANTE:
    Por cada ejecución se sobreescriben las fotos anteriores, así que debe tener mucho cuidado
    con volver a correrlo y no cambiar el nombre de las fotos
    """
    if key == ord('s'):
        print("Guardando")
        cv2.imwrite(f'./captures/{cap_index}.jpg',frame)
        cap_index+=1
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break
    
