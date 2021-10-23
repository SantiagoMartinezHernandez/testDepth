import cv2 
import numpy as np
import image_grid as ig

AR_DICT = {'0000000010000001000000100000010000001011100000000': 'ARUCO_1',
            '0000000010000001000000100000010000000100100000000': 'ARUCO_2',
            '0000000010000001000000100000010000000111000000000': 'ARUCO_3',
            '0000000010000001000000100000010111001000000000000': 'ARUCO_4',
            '0000000010000001000000100000010111001011100000000': 'ARUCO_5',
            '0000000010000001000000100000010111000100100000000': 'ARUCO_6',
            '0000000010000001000000100000010111000111000000000': 'ARUCO_7',
            '0000000010000001000000100000001001001000000000000': 'ARUCO_8',
            '0000000010000001000000100000001001001011100000000': 'ARUCO_9'}

# TEST_AR_DICT = {'0000000001110001001000110000010011001101000000000': 'ARUCO_00',
#                 '0000000010010001101100101100010011001010100000000': 'ARUCO_01',
#                 '0000000000101001110000111000001011000011000000000': 'ARUCO_02',
#                 '0000000011010000010100101100010110000000000000000': 'ARUCO_10',
#                 '0000000010011001100100110110010000000001100000000': 'ARUCO_11',
#                 '0000000010011000001000011110011101000010100000000': 'ARUCO_12',
#                 '0000000011101000100000000100000001000110100000000': 'ARUCO_20',
#                 '0000000011010001111000000110010110001110100000000': 'ARUCO_21',
#                 '0000000010100000101100011000010101001110000000000': 'ARUCO_22'}

def ordenar_puntos(puntos):
    """
    Ordena los puntos de la siguiente forma:
    n_puntos[0]: Esquina superior izquierda
    n_puntos[1]: Esquina superior derecha
    n_puntos[2]: Esquina inferior izquierda
    n_puntos[3]: Esquina inferior derecha
    """
    n_puntos = np.concatenate([puntos[0], puntos[1], puntos[2], puntos[3]]).tolist()
    y_order = sorted(n_puntos, key=lambda n_puntos: n_puntos[1])

    x1_order =  y_order[:2]
    x1_order = sorted(x1_order, key=lambda x1_order: x1_order[0])

    x2_order =  y_order[2:]
    x2_order = sorted(x2_order, key=lambda x2_order: x2_order[0])

    return [x1_order[0], x1_order[1], x2_order[0], x2_order[1]]

# img = cv2.imread('panel3.jpeg')

cap = cv2.VideoCapture(0)

while True:
    success, img = cap.read()
    if img is None:
        break
    originalImg = img.copy()
    _, img = cap.read()

    # originalImg = cv2.imread('panel3.jpeg')
    # originalImg = img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (7,7), 0)
    mask = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY_INV,9,2)

    canny = cv2.Canny(mask, 10, 150)
    canny = cv2.dilate(canny, None, iterations=1)

    conts = cv2.findContours(canny, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    conts = sorted(conts, key=cv2.contourArea, reverse=True)[:10]
    ars = []
    for c in conts:
        epsilon = 0.01* cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, epsilon, True)

        if len(approx) == 4:
            cv2.drawContours(img, [approx], 0, (0,255,255), 2)

            puntos = ordenar_puntos(approx)

            pts1 = np.float32(puntos)
            pts2 = np.float32([[0,0], [270,0], [0,310], [270,310]]) # Define el tamaño de la imagen de salida

            Mat = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(originalImg, Mat, (270,310))
            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            _, image_result = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)


            grid = ig.createGrid(image_result,7,7)

            ar_id = ""

            for row in grid[0]:
                for pixel in row:

                    pixel = pixel[5:-5,5:-5]

                    tot = len(pixel)*len(pixel[0])
                    count = np.sum(np.array(pixel) >= 127)
                    prop = count/tot
                    if prop >= 0.5:
                        ar_id += "1"
                    else:
                        ar_id += "0"
                    # print(prop)
            
                # ar_id += '\n'

            # print(ar_id)
            ars.append((grid[0],grid[1],dst,ar_id))

    cv2.imshow('img', img)
    # cv2.imshow('blur', blur)
    # cv2.imshow('mask',mask)
    # cv2.imshow('canny', canny)
    for i in range(len(ars)):
        try:
            cv2.imshow(f'{AR_DICT[ars[i][3]]}', ars[i][1])
        except KeyError as ke:
            print(f'Reconocido {i} :',ke)
            # cv2.imshow(f'{i}', ars[i][1])

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        break