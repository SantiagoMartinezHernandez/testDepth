import cv2 as cv
import numpy as np

def createGrid(img, num_rows, num_columns):
    """
    Se utiliza para crear una grilla sobre una imagen

    Parameters
    ----------
    img : array
        La imagen (debe ser una matriz numpy o arreglo común)
    num_rows : int
        El número de filas por las que se desea partir  
    num_columns : int
        El número de columnas por las que se desea partir

    Returns
    -------
    (images, grid_img)
    images : array
        Una matriz donde los datos corresponden a la imagen respectiva con respecto a la grilla
    grid_img : array
        La imagen original con una grilla pintada de amarillo

    """

    # versión anterior

    images = []                                             # Crea las matriz de imagenes a guardar
    new_img = img.copy()                                    # Copia la imagen (pues se va a modificar)

    heigth = len(new_img)//num_rows                         # Define la altura de cada una de las imagenes de la grilla

    while len(new_img) % num_rows != 0:                     # Este ciclo se utiliza ya que la función np.hsplit y np.split
        new_img = np.delete(new_img, len(new_img)-1, 0)     # dependen de que se pueda partir la imagen exactamente en los parametros
        heigth = len(new_img)//num_rows                     # especificados, por lo cual se cortan los bordes de la imagen

    width = len(new_img[0])//num_columns 

    while len(new_img[0]) % num_columns != 0:               # Lo mismo sucede con el ancho
        new_img = np.delete(new_img, len(new_img[0])-1, 1)
        width = (len(new_img[0]))//num_columns

    pre_images = np.split(new_img, num_rows)                # Se parte la imagen de manera vertical
    for row_img in pre_images:
        n_im = np.hsplit(row_img, num_columns)
        rows_images = []
        for col_image in n_im:
            rows_images.append(col_image)
        images.append(rows_images)                          # Se guarda la grilla en una matriz

    grid_img = new_img.copy()                               # A continuación se crea una copia de la imagen
    for i in range(num_rows+1):
        cv.line(grid_img, (0,i*heigth), (len(new_img[0]), (i*heigth)), (0,255,255), thickness=2)
    for j in range(num_columns+1):
        cv.line(grid_img, (j*width, 0), (j*width, len(new_img)), (0,255,255), thickness=2)

    return images, grid_img

    # x = len(img[0])
    # y = len(img)

    # # [x//cols*i : x//cols*(i+1)]

    # h = y//num_columns
    # l = x//num_rows

    # images = []
    # new_img = np.array(img)
    # grid_img = np.array(img)

    # for i in range(num_rows):
    #     row = []
    #     for j in range(num_columns):
    #         row.append(new_img[h*i:h*(i+1),l*j:l*(j+1)])
    #     images.append(row)

    # for i in range(num_rows+1):
    #     cv.line(grid_img, (0,h*i), (x,h*i), (0,255,255), thickness=2)
    # for j in range(num_columns+1):
    #     cv.line(grid_img, (l*j,0), (l*j,y), (0,255,255), thickness=2)


    # return images, grid_img