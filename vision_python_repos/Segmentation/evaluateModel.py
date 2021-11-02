import tensorflow_advanced_segmentation_models as tasm
import cv2
from tensorflow import keras
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt

HEIGHT = 240
WIDTH = 320
N_classes = 3
backbone_name = "efficientnetb1"
weights = "imagenet"

#path_weights = "C:/Users/SEBASTIAN/OneDrive - Universidad de los Andes/OctavoSemestre/Robocol_vision/Modelo_3_clases/modelo1/"
path_video = "C:/Users/Imagine/Videos/output.avi"

#path_model = "C:/Users/Imagine/Documents/Robocol_vision/Data_set_split/Modelo_2/modelo.h5"
path_weights = "C:/Users/Imagine/Documents/Robocol_vision/Data_set_split/Modelo_3/"
#Model
base_model, layers, layer_names = tasm.create_base_model(name=backbone_name, weights=weights, height=HEIGHT, width=WIDTH, include_top=False, pooling=None)
BACKBONE_TRAINABLE = False
model = tasm.DANet(n_classes=N_classes, base_model=base_model, output_layers=layers, backbone_trainable=BACKBONE_TRAINABLE)


#Load_weights
prueba = model.load_weights(path_weights)
model.build((None, HEIGHT, WIDTH, 3))

#show video
vid_capture = cv2.VideoCapture(path_video)
while(vid_capture.isOpened()):
    ret,frame = vid_capture.read()
    if ret == True:
        frame = cv2.resize(frame,(320,240))
        img_array = np.array(frame)
        #img_array = img_array/255
        img_array = np.expand_dims(img_array,axis=0)
        pred = model.predict(img_array)
        pred = pred[0]
        cv2.imshow('Prediccion',pred)
        cv2.imshow('Frame',frame)
        key = cv2.waitKey(20)

        if key == ord('q'):
            break
    else:
        break

vid_capture.release()
cv2.destroyAllWindows()
