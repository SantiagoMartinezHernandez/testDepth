from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.callbacks import CSVLogger
from tensorflow.keras.callbacks import EarlyStopping
from tensorflow.keras.optimizers import RMSprop
from tensorflow.keras.applications import ResNet50V2
import os
import numpy as np
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.utils import Sequence
from tensorflow.keras.preprocessing.image import load_img
import random 


#Paths
Path_img = "C:/Users/SEBASTIAN/OneDrive - Universidad de los Andes/Documentos/img_aug/x_aug/"
Path_mask = "C:/Users/SEBASTIAN/OneDrive - Universidad de los Andes/Documentos/img_aug/y_aug/"

weights_path = "C:/Users/SEBASTIAN/OneDrive - Universidad de los Andes/Documentos/img_aug/weights/"
path_csv = "C:/Users/SEBASTIAN/OneDrive - Universidad de los Andes/Documentos/img_aug/weights/logger.csv"

#Path list images
img_paths = [
    os.path.join(Path_img, fname)
    for fname in os.listdir(Path_img)
]

mask_paths = [
    os.path.join(Path_mask, fname)
    for fname in os.listdir(Path_mask)    
]

#train_frames_paths = [
 #       os.path.join(Path_Train_Frames, fname)
  #      for fname in os.listdir(Path_Train_Frames)
   # ]


#Split images
cant_imagenes = 20000 #Cuantas imagenes tenemos.

random.Random(40).shuffle(img_paths)
random.Random(40).shuffle(mask_paths)
train_img_paths = img_paths[:-int(cant_imagenes*0.6)]
train_mask_paths = mask_paths[:-int(cant_imagenes*0.6)]
val_img_paths = img_paths[-int(cant_imagenes*0.6):-int(cant_imagenes*0.8)]
val_mask_paths = mask_paths[-int(cant_imagenes*0.6):-int(cant_imagenes*0.8)]
test_img_paths = img_paths[-int(cant_imagenes*0.8):]
test_mask_paths = img_paths[-int(cant_imagenes*0.8):]

#Class for the images
class images(Sequence):
    def __init__(self, batch_size, img_size, input_img_paths,mask_img_paths):
        self.batch_size = batch_size
        self.img_size = img_size
        self.input_img_paths = input_img_paths
        self.mask_img_paths = mask_img_paths
    
    def __len__(self):
        return len(self.mask_img_paths) // self.batch_size
    
    def __getitem__(self, idx):
        i = idx * self.batch_size
        batch_input_img_paths = self.input_img_paths[i:i + self.batch_size]
        batch_mask_img_paths = self.mask_img_paths[i:i + self.batch_size]

        x = np.zeros((self.batch_size,) + self.img_size + (3,), dtype = "float32")
        for j, path in enumerate(batch_input_img_paths):
            img = load_img(path, target_size=self.img_size)
            mx = np.max(img)
            img = (img/mx) + 1 #Normalize the images.
            x[j] = img
        y = np.zeros((self.batch_size,) + self.img_size + (1,), dtype = "uint8")
        for j, path in enumerate(batch_mask_img_paths):
            img = load_img(path, target_size=self.img_size,color_mode="grayscale")
            y[j] = np.expand_dims(img,2)
        
        return x,y

#Parameters
No_Epochs = 15
Batch_Size = 2
Batch_Size_val = 2
img_size = (480,640)


#Base Model
base_model = ResNet50V2(weights='imagenet',
                        include_top=False,
                        input_shape=(480,640,3))
base_model.trainable = False

#Callbacks
checkpoint = ModelCheckpoint(weights_path,save_best_only=True)
csv_logger = CSVLogger(path_csv,separator=';',append=True)
earlyStopping = EarlyStopping(min_delta=0.01,patience=3)

callbacks_list = [checkpoint,csv_logger,earlyStopping]

#Top Model
input = keras.Input(shape=(480,640,3))

#Inferior of the model
x = base_model(input,training = False)

for filters in [2048,1024,512,256,128,64]:
    x = layers.Activation("relu")(x)
    x = layers.Conv2DTranspose(filters,3,padding="same")(x)
    x = layers.BatchNormalization()(x)
    if filters != 64:
        x = layers.UpSampling2D(2)(x)


output = layers.Conv2D(9,3,activation="softmax",padding="same")(x)

model = keras.Model(input,output)

model.summary()

#Batch organized images
train_gen = images(Batch_Size, img_size, train_img_paths, train_mask_paths)
val_gen = images(Batch_Size_val, img_size, val_img_paths, val_mask_paths)

#Model compile and fit
model.compile(
    optimizer = 'SGD',
    loss = 'sparse_categorical_crossentropy',
    metrics = [keras.metrics.SparseCategoricalCrossentropy()]
)

model.fit(train_gen,epochs=No_Epochs,validation_data=val_gen, callbacks=callbacks_list)

model.predict()