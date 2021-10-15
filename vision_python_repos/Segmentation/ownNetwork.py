import os
from tensorflow.keras.utils import Sequence
from tensorflow import keras
import numpy as np
from tensorflow.keras.preprocessing.image import load_img
from tensorflow.keras import layers
import random 
from tensorflow.keras.callbacks import ModelCheckpoint
from tensorflow.keras.callbacks import CSVLogger
from tensorflow.keras.callbacks import EarlyStopping

input_dir = "C:/Users/Imagine/Documents/Robocol_vision/img_aug/x_aug/"
target_dir = "C:/Users/Imagine/Documents/Robocol_vision/img_aug/y_aug/"
weights_path = "C:/Users/Imagine/Documents/Robocol_vision/img_aug/weights/"
path_csv = "C:/Users/Imagine/Documents/Robocol_vision/img_aug/weights/logger.csv"

No_Epochs = 15
img_size = (480,640)
num_classes = 9
Batch_Size = 4
Batch_Size_val = 4

#Path list images
img_paths = [
    os.path.join(input_dir, fname)
    for fname in os.listdir(input_dir)
]

mask_paths = [
    os.path.join(target_dir, fname)
    for fname in os.listdir(target_dir)    
]


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


def get_model(img_size, num_classes):
    inputs = keras.Input(shape=img_size + (3,))

    ### [First half of the network: downsampling inputs] ###

    # Entry block
    x = layers.Conv2D(32, 3, strides=2, padding="same")(inputs)
    x = layers.BatchNormalization()(x)
    x = layers.Activation("relu")(x)

    previous_block_activation = x  # Set aside residual

    # Blocks 1, 2, 3 are identical apart from the feature depth.
    for filters in [64, 128, 256]:
        x = layers.Activation("relu")(x)
        x = layers.SeparableConv2D(filters, 3, padding="same")(x)
        x = layers.BatchNormalization()(x)

        x = layers.Activation("relu")(x)
        x = layers.SeparableConv2D(filters, 3, padding="same")(x)
        x = layers.BatchNormalization()(x)

        x = layers.MaxPooling2D(3, strides=2, padding="same")(x)

        # Project residual
        residual = layers.Conv2D(filters, 1, strides=2, padding="same")(
            previous_block_activation
        )
        x = layers.add([x, residual])  # Add back residual
        previous_block_activation = x  # Set aside next residual

    ### [Second half of the network: upsampling inputs] ###

    for filters in [256, 128, 64, 32]:
        x = layers.Activation("relu")(x)
        x = layers.Conv2DTranspose(filters, 3, padding="same")(x)
        x = layers.BatchNormalization()(x)

        x = layers.Activation("relu")(x)
        x = layers.Conv2DTranspose(filters, 3, padding="same")(x)
        x = layers.BatchNormalization()(x)

        x = layers.UpSampling2D(2)(x)

        # Project residual
        residual = layers.UpSampling2D(2)(previous_block_activation)
        residual = layers.Conv2D(filters, 1, padding="same")(residual)
        x = layers.add([x, residual])  # Add back residual
        previous_block_activation = x  # Set aside next residual

    # Add a per-pixel classification layer
    outputs = layers.Conv2D(num_classes, 3, activation="softmax", padding="same")(x)

    # Define the model
    model = keras.Model(inputs, outputs)
    return model


# Free up RAM in case the model definition cells were run multiple times
keras.backend.clear_session()

# Build model
model = get_model(img_size, num_classes)
model.summary()


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

#Batch organized images
train_gen = images(Batch_Size, img_size, train_img_paths, train_mask_paths)
val_gen = images(Batch_Size_val, img_size, val_img_paths, val_mask_paths)


#Callbacks
checkpoint = ModelCheckpoint(weights_path,save_best_only=True)
csv_logger = CSVLogger(path_csv,separator=';',append=True)
earlyStopping = EarlyStopping(min_delta=0.01,patience=3)

callbacks_list = [checkpoint,csv_logger,earlyStopping]

#Model compile and fit

opt = keras.optimizers.RMSprop(learning_rate=1e-6)
model.compile(optimizer = opt,loss = 'sparse_categorical_crossentropy')

model.fit(train_gen,epochs=No_Epochs,validation_data=val_gen, callbacks=callbacks_list)